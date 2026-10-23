#include "mem/cache/pic/read_post_process.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICReadPost.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

// Scala: ReadPostProcess (Module) -- builds the ports standing in for
// io.req, io.unaligned_dataIn, io.accessBank and io.dataToBuf*.
ReadPostProcess::ReadPostProcess(const Params &p) :
    ClockedObject(p),
    reqSidePort(p.name + ".req_side", this),
    dataSidePort(p.name + ".data_side", this),
    bankSidePort(p.name + ".bank_side", this),
    bufSidePort(p.name + ".buf_side", this),
    requestorId(p.system->getRequestorId(this)),
    sendEvent([this]{ sendWord(); }, name() + ".sendEvent")
{
}

// gem5 only: map Python port names to the C++ ports.
Port &
ReadPostProcess::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "req_side")
        return reqSidePort;
    if (if_name == "data_side")
        return dataSidePort;
    if (if_name == "bank_side")
        return bankSidePort;
    if (if_name == "buf_side")
        return bufSidePort;
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Ports (gem5 only: valid/ready handshakes become timing req/resp + retry)
// ---------------------------------------------------------------------------

AddrRangeList
ReadPostProcess::InPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

bool
ReadPostProcess::InPort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(this, pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

bool
ReadPostProcess::OutPort::recvTimingResp(PacketPtr pkt)
{
    owner->handleAck(pkt);
    return true;
}

void
ReadPostProcess::OutPort::recvReqRetry()
{
    owner->outRetry();
}

// gem5 only: send a request to the code for the port it came in on.
bool
ReadPostProcess::handleRequest(const InPort *port, PacketPtr pkt)
{
    if (port == &reqSidePort)
        return handleSetup(pkt);
    return handleBeat(pkt);
}

// ---------------------------------------------------------------------------
// Setup / data in
// ---------------------------------------------------------------------------

// Scala: state sIdle, io.req.fire -- latch bank_baseAddr, bank_endAddr,
// padInfo and dataDir; go to setPad if padding is needed, else
// wait_finish. Get ready for one load's data before it arrives.
bool
ReadPostProcess::handleSetup(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(ReadPostProcessReq),
             "ReadPostProcess: expected a %zu-byte setup, got %u",
             sizeof(ReadPostProcessReq), pkt->getSize());

    if (state != State::Idle)
        return false;

    const auto *req = pkt->getConstPtr<ReadPostProcessReq>();
    panic_if(req->padding && req->padSize > 0,
             "ReadPostProcess: padding (PaddingUnit) is not implemented");
    panic_if(req->dataDir > static_cast<uint32_t>(DataDir::P2SRT),
             "ReadPostProcess: bad dataDir %u", req->dataDir);

    setupPkt = pkt;
    arrayAddr = req->bankBaseAddr;
    endAddr = req->bankEndAddr;
    dataDir = static_cast<DataDir>(req->dataDir);
    alignData = 0;
    alignBytes = 0;
    state = State::WaitFinish;

    DPRINTF(PICReadPost, "setup dataDir %u bank addr %#x..%#x\n",
            req->dataDir, arrayAddr, endAddr);
    return true;
}

// Scala: io.unaligned_dataIn -> padUnit -> aligner.io.in. Take one beat
// from DMA_read_page_split while the Aligner has room (in.ready).
bool
ReadPostProcess::handleBeat(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(StreamBeat),
             "ReadPostProcess: expected a %zu-byte StreamBeat, got %u",
             sizeof(StreamBeat), pkt->getSize());

    // One beat makes at most two words, so take it only with room for
    // both.
    if (state != State::WaitFinish || words.size() >= 2)
        return false;

    StreamBeat beat;
    pkt->writeData(reinterpret_cast<uint8_t *>(&beat));
    align(beat);

    pkt->makeResponse();
    dataSidePort.schedTimingResp(pkt, clockEdge(Cycles(1)));

    if (!sendEvent.scheduled())
        schedule(sendEvent, clockEdge(Cycles(1)));
    return true;
}

// Scala: Aligner (DMA_core_dependent.scala) -- pack the kept bytes of
// each beat back to back into 64-bit words, carrying leftovers into the
// next beat; on last, flush whatever is left as a short final word.
void
ReadPostProcess::align(const StreamBeat &beat)
{
    panic_if(beat.keep == 0, "ReadPostProcess: Aligner cannot handle an "
             "empty flit");

    for (unsigned lane = 0; lane < 8; ++lane) {
        if (!(beat.keep & (1u << lane)))
            continue;
        const uint64_t byte = (beat.data >> (8 * lane)) & 0xff;
        alignData |= byte << (8 * alignBytes);
        if (++alignBytes == 8) {
            words.push_back({alignData, false});
            alignData = 0;
            alignBytes = 0;
        }
    }

    if (beat.last) {
        if (alignBytes > 0) {
            words.push_back({alignData, true});
            alignData = 0;
            alignBytes = 0;
        } else {
            words.back().last = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Data out
// ---------------------------------------------------------------------------

// Scala: io.accessBank (dataDir == bank: addr = baseAddr_Array_reg,
// optype WRITE, data = aligner output) or io.dataToBufAvailableP2S* +
// io.dataToBuf (dataDir == p2s*). Send the next aligned word to where
// this load's data goes.
void
ReadPostProcess::sendWord()
{
    if (waitingAck || outRetryPkt || words.empty())
        return;

    const Word &w = words.front();
    PacketPtr pkt;
    OutPort *port;

    if (dataDir == DataDir::Bank) {
        P2SWritePayload payload{arrayAddr, w.data};
        RequestPtr req = std::make_shared<Request>(
            0, sizeof(payload), 0, requestorId);
        pkt = new Packet(req, MemCmd::WriteReq);
        pkt->allocate();
        pkt->setData(reinterpret_cast<const uint8_t *>(&payload));
        port = &bankSidePort;
        DPRINTF(PICReadPost, "bank write addr %#x data %#x%s\n",
                arrayAddr, w.data, w.last ? " (last)" : "");
    } else {
        RequestPtr req = std::make_shared<Request>(
            0, sizeof(w.data), 0, requestorId);
        pkt = new Packet(req, MemCmd::WriteReq);
        pkt->allocate();
        pkt->setLE<uint64_t>(w.data);
        port = &bufSidePort;
        DPRINTF(PICReadPost, "buffer word %#x%s\n", w.data,
                w.last ? " (last)" : "");
    }

    waitingAck = true;
    if (!port->sendTimingReq(pkt))
        outRetryPkt = pkt;
}

// gem5 only: resend the word the receiver refused.
void
ReadPostProcess::outRetry()
{
    panic_if(!outRetryPkt, "ReadPostProcess: unexpected retry");
    OutPort *port =
        dataDir == DataDir::Bank ? &bankSidePort : &bufSidePort;
    if (port->sendTimingReq(outRetryPkt))
        outRetryPkt = nullptr;
}

// Scala: accessBank.fire / dataToBufAvailable.fire -- the word is taken:
// baseAddr_Array_reg advances by one, and the load is done once the last
// word is out (dataDirBankFinish / dataDirBufFinish).
void
ReadPostProcess::handleAck(PacketPtr pkt)
{
    delete pkt;
    panic_if(!waitingAck || words.empty(),
             "ReadPostProcess: ack with nothing outstanding");

    const Word w = words.front();
    words.pop_front();
    waitingAck = false;

    if (dataDir == DataDir::Bank) {
        if (w.last && arrayAddr != endAddr) {
            warn("ReadPostProcess: last bank write at %#x, setup said "
                 "end %#x", arrayAddr, endAddr);
        }
        ++arrayAddr;
    }

    if (dataSidePort.needRetry && words.size() < 2) {
        dataSidePort.needRetry = false;
        dataSidePort.sendRetryReq();
    }

    if (w.last)
        finish();
    else if (!words.empty() && !sendEvent.scheduled())
        schedule(sendEvent, clockEdge(Cycles(1)));
}

// Scala: wait_finish -> sIdle, io.busy falls. Answer Load_ctl's setup so
// its wait_post_process_finish can complete the load.
void
ReadPostProcess::finish()
{
    DPRINTF(PICReadPost, "load done\n");

    panic_if(!words.empty(), "ReadPostProcess: words left after last");
    state = State::Idle;
    setupPkt->makeResponse();
    reqSidePort.schedTimingResp(setupPkt, clockEdge(Cycles(1)));
    setupPkt = nullptr;

    if (reqSidePort.needRetry) {
        reqSidePort.needRetry = false;
        reqSidePort.sendRetryReq();
    }
}

}  // namespace gem5
