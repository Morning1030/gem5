#include "mem/cache/pic/load_ctl.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICLoadCtl.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/system.hh"

namespace gem5
{

// Scala: Load_ctl (Module) -- builds the ports standing in for the five
// client inputs (load_req, im2col_ld_req, p2sL/R/R_T_req), dma_read_req,
// post_process_req and, for P2S, the dataToBuf wires.
Load_ctl::Load_ctl(const Params &p) :
    ClockedObject(p),
    cmdSidePort(p.name + ".cmd_side", this, kGeneralLoad),
    bufSidePort(p.name + ".buf_side", this, -1),
    dmaSidePort(p.name + ".dma_side", this),
    postSidePort(p.name + ".post_side", this),
    requestorId(p.system->getRequestorId(this)),
    stepEvent([this]{ step(); }, name() + ".stepEvent")
{
    const int n = p.port_p2s_side_connection_count;
    fatal_if(n != 0 && n != 3,
             "Load_ctl: p2s_side needs 0 or 3 connections (P2S_L, P2S_R, "
             "P2S_R_T), got %d", n);
    for (int i = 0; i < n; ++i) {
        p2sPorts.emplace_back(std::make_unique<InPort>(
            p.name + csprintf(".p2s_side[%d]", i), this, kP2SL + i));
    }
}

// gem5 only: map Python port names to the C++ ports.
Port &
Load_ctl::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cmd_side")
        return cmdSidePort;
    if (if_name == "p2s_side" && static_cast<size_t>(idx) < p2sPorts.size())
        return *p2sPorts[idx];
    if (if_name == "buf_side")
        return bufSidePort;
    if (if_name == "dma_side")
        return dmaSidePort;
    if (if_name == "post_side")
        return postSidePort;
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Ports (gem5 only: valid/ready handshakes become timing req/resp + retry)
// ---------------------------------------------------------------------------

AddrRangeList
Load_ctl::InPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

bool
Load_ctl::InPort::recvTimingReq(PacketPtr pkt)
{
    const bool ok = id < 0 ? owner->handleBufWord(pkt)
                           : owner->handleClientRequest(id, pkt);
    if (!ok)
        needRetry = true;
    return ok;
}

bool
Load_ctl::OutPort::recvTimingResp(PacketPtr pkt)
{
    owner->handleResponse(this, pkt);
    return true;
}

void
Load_ctl::OutPort::recvReqRetry()
{
    owner->handleRetry(this);
}

Load_ctl::InPort &
Load_ctl::clientPort(int client)
{
    if (client == kGeneralLoad)
        return cmdSidePort;
    return *p2sPorts[client - kP2SL];
}

// gem5 only: a client whose request was refused may try again once its
// arbiter slot is free.
void
Load_ctl::retryClients()
{
    for (int c = 0; c < kNumClients; ++c) {
        if (c == kIm2colLoad || pendingPkt[c])
            continue;
        if (c >= kP2SL && static_cast<size_t>(c - kP2SL) >= p2sPorts.size())
            continue;
        InPort &port = clientPort(c);
        if (port.needRetry) {
            port.needRetry = false;
            port.sendRetryReq();
        }
    }
}

// ---------------------------------------------------------------------------
// Arbiter inputs
// ---------------------------------------------------------------------------

// Scala: arbiter.io.in(client) <> io.load_req / io.p2sL_req / ... -- a
// client holds one Load_req valid until the arbiter takes it. Turn the
// client's packet into a Load_req and park it in its arbiter slot.
bool
Load_ctl::handleClientRequest(int client, PacketPtr pkt)
{
    if (pendingPkt[client] || activeClient == client)
        return false;

    LoadReq req;
    if (client == kGeneralLoad) {
        // Scala: CMD_resolver_new.enq_LOAD -- src -> baseAddr_DRAM,
        // dst -> baseAddr_Array, dataDir = bank, no padding.
        panic_if(pkt->getSize() != sizeof(LSPayload),
                 "Load_ctl: expected a %zu-byte LSPayload, got %u",
                 sizeof(LSPayload), pkt->getSize());
        const auto *ls = pkt->getConstPtr<LSPayload>();
        req.bytePerRow = ls->byte_per_row;
        req.row = ls->row;
        req.offset = ls->offset;
        req.baseAddrDram = ls->src;
        req.baseAddrArray = ls->dst;
        req.dataDir = DataDir::Bank;
    } else {
        // Scala: P2S_*_ctl loadReq -- baseAddr_Array 0, dataDir = its
        // own buffer. gem5's P2S asks for one row per request.
        panic_if(!pkt->isRead(), "Load_ctl: P2S sent a non-read request");
        req.bytePerRow = pkt->getSize();
        req.row = 1;
        req.offset = pkt->getSize();
        req.baseAddrDram = pkt->getAddr();
        req.baseAddrArray = 0;
        req.dataDir = static_cast<DataDir>(
            static_cast<unsigned>(DataDir::P2SL) + (client - kP2SL));
    }

    DPRINTF(PICLoadCtl, "client %d: load vaddr %#x %u x %u B stride %u\n",
            client, req.baseAddrDram, req.row, req.bytePerRow, req.offset);

    pendingPkt[client] = pkt;
    pendingReq[client] = req;
    if (state == State::Idle)
        scheduleStep();
    return true;
}

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------

// gem5 only: one FSM transition per clock edge, like the RTL registers.
void
Load_ctl::scheduleStep()
{
    if (!stepEvent.scheduled())
        schedule(stepEvent, clockEdge(Cycles(1)));
}

// Scala: switch(load_state) -- run the current state.
void
Load_ctl::step()
{
    switch (state) {
      case State::Idle:
        arbitrate();
        break;
      case State::SetPostProcess:
        sendPostProcessReq();
        break;
      case State::ReqDma:
        sendDmaReq();
        break;
      case State::WaitPostProcessFinish:
        if (postDone)
            finishLoad();
        break;
      case State::WaitDmaFinish:
        break;
    }
}

// Scala: state load_idle -- RRArbiter picks the next client after the
// last winner (arbiter.io.out.ready only in load_idle). Latch the
// request, merge contiguous rows into one (one_shot), and work out the
// last array address the load will write (endAddr_Array_reg, including
// padding).
void
Load_ctl::arbitrate()
{
    for (unsigned i = 1; i <= kNumClients; ++i) {
        const unsigned c = (lastGrant + i) % kNumClients;
        if (!pendingPkt[c])
            continue;

        activeClient = c;
        activePkt = pendingPkt[c];
        cur = pendingReq[c];
        pendingPkt[c] = nullptr;
        lastGrant = c;

        panic_if(cur.row == 0 || cur.bytePerRow == 0,
                 "Load_ctl: empty load from client %u", c);

        // Rows back to back in DRAM: fetch them as one row.
        const bool oneShot = cur.bytePerRow == cur.offset;
        bytesPerRow = oneShot ? uint64_t(cur.bytePerRow) * cur.row
                              : cur.bytePerRow;
        rowNum = oneShot ? 1 : cur.row;
        dramAddr = cur.baseAddrDram;

        // Array addresses count 64-bit words (bitlineNums / 8 bytes).
        const uint64_t pad = cur.padding ? cur.padSize : 0;
        const uint64_t bytesAfterPad =
            (cur.row + 2 * pad) * (cur.bytePerRow + 2 * pad);
        const uint64_t endBytes = (cur.baseAddrArray << 3) + bytesAfterPad;
        endAddrArray = (endBytes >> 3) - ((endBytes & 7) == 0 ? 1 : 0);

        bufData.clear();
        postDone = false;

        DPRINTF(PICLoadCtl, "grant client %u: %u row(s) x %u B%s, array "
                "%#x..%#x\n", c, rowNum, bytesPerRow,
                oneShot ? " (one shot)" : "", cur.baseAddrArray,
                endAddrArray);

        state = State::SetPostProcess;
        scheduleStep();
        retryClients();
        return;
    }
}

// Scala: state set_post_process -- io.post_process_req with
// bank_baseAddr, bank_endAddr, padding, padInfo, dataDir. Tell
// ReadPostProcess where this load's data goes before any of it arrives.
void
Load_ctl::sendPostProcessReq()
{
    ReadPostProcessReq setup{};
    setup.bankBaseAddr = cur.baseAddrArray;
    setup.bankEndAddr = endAddrArray;
    setup.padding = cur.padding;
    setup.padSize = cur.padSize;
    setup.dataDir = static_cast<uint32_t>(cur.dataDir);

    RequestPtr req = std::make_shared<Request>(0, sizeof(setup), 0,
                                               requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(reinterpret_cast<const uint8_t *>(&setup));

    if (!postSidePort.sendTimingReq(pkt)) {
        outRetryPkt = pkt;
        return;
    }
    state = State::ReqDma;
    scheduleStep();
}

// Scala: state req_dma -- io.dma_read_req (nByte = bytes_per_row_reg,
// baseAddr_DRAM); on fire, baseAddr_DRAM += offset and row_num -= 1.
// Hand the next row to DMA_read_page_split.
void
Load_ctl::sendDmaReq()
{
    // Scala: io.lastRow = (row_num_reg === 0), seen after the decrement.
    DMAReadReq row{bytesPerRow, dramAddr, rowNum == 1};

    RequestPtr req = std::make_shared<Request>(0, sizeof(row), 0,
                                               requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(reinterpret_cast<const uint8_t *>(&row));

    DPRINTF(PICLoadCtl, "row read vaddr %#x %u B (%u left)\n", dramAddr,
            bytesPerRow, rowNum - 1);

    dramAddr += cur.offset;
    --rowNum;
    state = State::WaitDmaFinish;
    if (!dmaSidePort.sendTimingReq(pkt))
        outRetryPkt = pkt;
}

// gem5 only: resend the setup or row read that was refused.
void
Load_ctl::handleRetry(const OutPort *port)
{
    panic_if(!outRetryPkt, "Load_ctl: unexpected retry");
    OutPort *p = port == &dmaSidePort ? &dmaSidePort : &postSidePort;
    if (!p->sendTimingReq(outRetryPkt))
        return;
    outRetryPkt = nullptr;
    if (state == State::SetPostProcess) {
        state = State::ReqDma;
        scheduleStep();
    }
}

// Scala: state wait_dma_finish (io.dma_busy falls -> next row, or
// wait_post_process_finish after the last one) and io.process_busy
// falling. The two responses that move the FSM on.
void
Load_ctl::handleResponse(const OutPort *port, PacketPtr pkt)
{
    delete pkt;
    if (port == &dmaSidePort) {
        panic_if(state != State::WaitDmaFinish,
                 "Load_ctl: row done while not waiting for one");
        state = rowNum == 0 ? State::WaitPostProcessFinish : State::ReqDma;
    } else {
        postDone = true;
    }
    if (state == State::ReqDma || state == State::WaitPostProcessFinish)
        scheduleStep();
}

// Scala: io.dataToBuf + dataToBufAvailableP2S*.valid from
// ReadPostProcess. Collect one 64-bit word of the active P2S row.
bool
Load_ctl::handleBufWord(PacketPtr pkt)
{
    panic_if(activeClient < kP2SL,
             "Load_ctl: P2S buffer word with no P2S load active");

    const uint8_t *data = pkt->getConstPtr<uint8_t>();
    bufData.insert(bufData.end(), data, data + pkt->getSize());

    pkt->makeResponse();
    bufSidePort.schedTimingResp(pkt, clockEdge(Cycles(1)));
    return true;
}

// Scala: state wait_post_process_finish -> load_idle. The load is done:
// answer the client (P2S gets its row's bytes; the Scheduler, which in
// the RTL watches io.busy, gets a plain ack) and take the next one.
void
Load_ctl::finishLoad()
{
    PacketPtr pkt = activePkt;
    const int client = activeClient;
    activePkt = nullptr;
    activeClient = -1;
    state = State::Idle;

    if (client >= kP2SL) {
        panic_if(bufData.size() < pkt->getSize(),
                 "Load_ctl: P2S row has %zu of %u bytes", bufData.size(),
                 pkt->getSize());
        pkt->makeResponse();
        pkt->setData(bufData.data());
        DPRINTF(PICLoadCtl, "client %d: row vaddr %#x done\n", client,
                pkt->getAddr());
        DDUMP(PICLoadCtl, bufData.data(), pkt->getSize());
    } else {
        pkt->makeResponse();
        DPRINTF(PICLoadCtl, "client %d: load done\n", client);
    }
    clientPort(client).schedTimingResp(pkt, clockEdge(Cycles(1)));

    retryClients();
    for (PacketPtr p : pendingPkt) {
        if (p) {
            scheduleStep();
            break;
        }
    }
}

}  // namespace gem5
