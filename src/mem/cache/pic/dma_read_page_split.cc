#include "mem/cache/pic/dma_read_page_split.hh"

#include <algorithm> // std::min

#include "base/logging.hh" // gem5 handle panic, fatal, warn
#include "base/trace.hh" // gem5 handle DPRINTF
#include "debug/PICDMA.hh" //come up after compile with debug flags
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{
constexpr Addr PageBytes = 4096; //default RISC-V page size, used for page splitting
}  // namespace, only visible in this file, wont conflict with other outer files

std::deque<AddrChunk>
splitRequests(const DMADescriptor &desc, uint32_t maxChunkBytes,
              uint32_t blockBytes)
{
    std::deque<AddrChunk> out;

    if (desc.bytesPerRow == 0 || desc.numRows == 0)
        return out;

    if (desc.rowStrideBytes != 0 &&
        desc.rowStrideBytes < desc.bytesPerRow) {
        warn("splitRequests: rowStride %llu < bytesPerRow %u",
             (unsigned long long)desc.rowStrideBytes, desc.bytesPerRow);
        return out;
    }

    for (uint32_t row = 0; row < desc.numRows; ++row) // outer loop : every row
    {
        const Addr rowStart = desc.dramVaddr + row * desc.rowStrideBytes; // the starting addr of this row (why we need stride)
        uint32_t done = 0;

        while (done < desc.bytesPerRow) // inner loop : every chunk in a row
        {
            const Addr v = rowStart + done;

            // Three independent ceilings.  Whichever bites first wins.
            const Addr pageEnd  = (v & ~(PageBytes - 1)) + PageBytes;
            const Addr blockEnd = (v & ~Addr(blockBytes - 1)) + blockBytes;

            uint32_t n = desc.bytesPerRow - done;
            n = std::min<uint64_t>(n, pageEnd - v);
            n = std::min<uint64_t>(n, blockEnd - v);
            n = std::min(n, maxChunkBytes);

            out.push_back(AddrChunk{v, n, row, done}); // store the chunk info into the deque, including vaddr, size, rowIdx, rowOffset
            done += n;
        }
    }

    return out;
}


// mod: p2s_arbiter
// ---------------------------------------------------------------------------
// DMA_read_page_split -- construction, ports, getPort
// ---------------------------------------------------------------------------

// Scala: DMA_read_page_split (LazyModule) -- the module and its
// StreamReader. Builds the ports that stand in for io.req, io.dataOut,
// the TLB query wires and the StreamReader's TileLink node.
DMA_read_page_split::DMA_read_page_split(const Params &p) :
    ClockedObject(p),
    reqSidePort(p.name + ".req_side", this),
    dataSidePort(p.name + ".data_side", this),
    tlbSidePort(p.name + ".tlb_side", this),
    memSidePort(p.name + ".mem_side", this),
    requestorId(p.system->getRequestorId(this)),
    stepEvent([this]{ step(); }, name() + ".stepEvent"),
    sendEvent([this]{ sendBeats(); }, name() + ".sendEvent"),
    chunkBytes(p.chunk_bytes),
    blockBytes(p.block_bytes),
    maxOutstanding(p.max_outstanding),
    skipSamePage(p.skip_same_page)
{
    fatal_if(chunkBytes == 0 || chunkBytes > blockBytes,
             "DMA_read_page_split: chunk_bytes must be in 1..block_bytes");
    fatal_if(maxOutstanding == 0,
             "DMA_read_page_split: max_outstanding must be > 0");
}

// gem5 only: map Python port names to the C++ ports.
Port &
DMA_read_page_split::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "req_side")
        return reqSidePort;
    if (if_name == "data_side")
        return dataSidePort;
    if (if_name == "tlb_side")
        return tlbSidePort;
    if (if_name == "mem_side")
        return memSidePort;
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Ports (gem5 only: valid/ready handshakes become timing req/resp + retry)
// ---------------------------------------------------------------------------

AddrRangeList
DMA_read_page_split::ReqSidePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

// Scala: io.req.ready -- refuse while a row is in progress.
bool
DMA_read_page_split::ReqSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

// gem5 only: Load_ctl refused our "row done" response earlier.
void
DMA_read_page_split::ReqSidePort::recvRespRetry()
{
    owner->reqRespRetry();
}

bool
DMA_read_page_split::OutPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(this, pkt);
}

void
DMA_read_page_split::OutPort::recvReqRetry()
{
    owner->handleRetry(this);
}

// gem5 only: send each response to the code for the port it came in on.
bool
DMA_read_page_split::handleResponse(const OutPort *port, PacketPtr pkt)
{
    if (port == &tlbSidePort)
        return handleTlbResponse(pkt);
    if (port == &memSidePort)
        return handleMemResponse(pkt);
    // data_side: ReadPostProcess acks every beat; nothing to do.
    delete pkt;
    return true;
}

// gem5 only: resend whatever the port refused.
void
DMA_read_page_split::handleRetry(const OutPort *port)
{
    if (port == &dataSidePort) {
        dataBlocked = false;
        if (!sendEvent.scheduled())
            schedule(sendEvent, clockEdge(Cycles(1)));
    } else if (port == &tlbSidePort) {
        panic_if(!tlbRetryPkt, "DMA_read_page_split: unexpected tlb retry");
        if (tlbSidePort.sendTimingReq(tlbRetryPkt))
            tlbRetryPkt = nullptr;
    } else {
        panic_if(!memRetryPkt, "DMA_read_page_split: unexpected mem retry");
        if (memSidePort.sendTimingReq(memRetryPkt)) {
            memRetryPkt = nullptr;
            scheduleStep();
        }
    }
}

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------

// Scala: state main_idle -- arbiter.io.out.fire latches nByte,
// baseAddr_DRAM and v_addr_ptr. Accept one row read from Load_ctl.
bool
DMA_read_page_split::handleRequest(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(DMAReadReq),
             "DMA_read_page_split: expected a %zu-byte DMAReadReq, got %u",
             sizeof(DMAReadReq), pkt->getSize());

    if (state != State::Idle || reqSidePort.blockedResp)
        return false;

    const auto *req = pkt->getConstPtr<DMAReadReq>();
    panic_if(req->nByte == 0, "DMA_read_page_split: zero-byte read");

    activeReq = pkt;
    vaddrPtr = req->baseAddrDram;
    finalVaddr = req->baseAddrDram + req->nByte;
    lastRow = req->lastRow != 0;

    DPRINTF(PICDMA, "row read vaddr %#x nByte %u%s\n", vaddrPtr,
            req->nByte, lastRow ? " (last row)" : "");

    state = State::PreProcess;
    scheduleStep();
    return true;
}

// gem5 only: one FSM transition per clock edge, like the RTL registers.
void
DMA_read_page_split::scheduleStep()
{
    if (!stepEvent.scheduled())
        schedule(stepEvent, clockEdge(Cycles(1)));
}

// Scala: switch(main_state) -- run the current state.
void
DMA_read_page_split::step()
{
    switch (state) {
      case State::PreProcess:
        preProcess();
        break;
      case State::QueryTLB:
        sendTlbQuery();
        break;
      case State::DMAReading:
        issueReads();
        break;
      case State::Idle:
      case State::QueryWaitResp:
        break;
    }
}

// Scala: state pre_process -- if bytes remain, cut the next segment at
// the page boundary (cur_block_len) and go translate it; otherwise the
// row is done and the module goes back to main_idle.
void
DMA_read_page_split::preProcess()
{
    if (vaddrPtr >= finalVaddr) {
        finishRequest();
        return;
    }

    const Addr nextPage = (vaddrPtr / PageBytes + 1) * PageBytes;
    curBlockLen = std::min(finalVaddr, nextPage) - vaddrPtr;

    DMADescriptor seg;
    seg.dramVaddr = vaddrPtr;
    seg.bytesPerRow = curBlockLen;
    seg.numRows = 1;
    chunks = splitRequests(seg, chunkBytes, blockBytes);
    nextIssue = 0;
    nextDrain = 0;

    // RTL: can_skip_ptw is commented out, so it always queries the TLB.
    if (skipSamePage && pageValid && vpn == vaddrPtr / PageBytes) {
        DPRINTF(PICDMA, "segment vaddr %#x len %u, same page, skip TLB\n",
                vaddrPtr, curBlockLen);
        state = State::DMAReading;
    } else {
        DPRINTF(PICDMA, "segment vaddr %#x len %u\n", vaddrPtr,
                curBlockLen);
        state = State::QueryTLB;
    }
    scheduleStep();
}

// Scala: state query_TLB -- io.query_tlb_req.valid with vaddr =
// v_addr_ptr. Ask TLB_req_schedular for this segment's page.
void
DMA_read_page_split::sendTlbQuery()
{
    RequestPtr req = std::make_shared<Request>(vaddrPtr, 8, 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();

    state = State::QueryWaitResp;
    if (!tlbSidePort.sendTimingReq(pkt))
        tlbRetryPkt = pkt;
}

// Scala: state query_wait_resp -- on io.paddr_valid latch p_page_id and
// v_page_valid_id, raise paddr_received, and move on to the DRAM read.
bool
DMA_read_page_split::handleTlbResponse(PacketPtr pkt)
{
    panic_if(state != State::QueryWaitResp,
             "DMA_read_page_split: TLB response while not waiting for one");

    const Addr paddr = pkt->getLE<uint64_t>();
    vpn = vaddrPtr / PageBytes;
    ppn = paddr / PageBytes;
    pageValid = true;
    delete pkt;

    DPRINTF(PICDMA, "TLB vpn %#x -> ppn %#x\n", vpn, ppn);

    state = State::DMAReading;
    scheduleStep();
    return true;
}

// Scala: state dma_request -- dmaReaderIO.req with address
// (v_addr_ptr & 0xFFF) + (p_page_id << 12) and length cur_block_len.
// StreamReader then splits that into TileLink reads; this issues those
// reads itself, one per cycle, at most max_outstanding in flight.
void
DMA_read_page_split::issueReads()
{
    if (memRetryPkt || nextIssue >= chunks.size() ||
        outstanding >= maxOutstanding)
        return;

    const AddrChunk &c = chunks[nextIssue];
    const Addr paddr = ppn * PageBytes + c.vaddr % PageBytes;

    RequestPtr req = std::make_shared<Request>(paddr, c.size, 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    pkt->pushSenderState(new PICDMASenderState(nextIssue));

    DPRINTF(PICDMA, "read paddr %#x size %u (chunk %u)\n", paddr, c.size,
            nextIssue);

    ++nextIssue;
    ++outstanding;
    if (!memSidePort.sendTimingReq(pkt)) {
        memRetryPkt = pkt;
        return;
    }
    if (nextIssue < chunks.size())
        scheduleStep();
}

// Scala: state dma_finish -- StreamReader output goes into data_queue;
// when the segment's last beat is out, v_addr_ptr moves to the next page
// and the FSM returns to pre_process.
bool
DMA_read_page_split::handleMemResponse(PacketPtr pkt)
{
    auto *ss = safe_cast<PICDMASenderState *>(pkt->popSenderState());
    arrived.emplace(ss->chunkIdx, pkt);
    delete ss;
    --outstanding;

    drainChunks();

    if (nextDrain == chunks.size()) {
        vaddrPtr = (vaddrPtr / PageBytes + 1) * PageBytes;
        state = State::PreProcess;
    }
    scheduleStep();
    return true;
}

// Scala: StreamReader returns data in address order. DRAM responses can
// come back out of order here, so hold them until the next one is in.
void
DMA_read_page_split::drainChunks()
{
    const bool lastSegment =
        vaddrPtr + curBlockLen >= finalVaddr;

    for (auto it = arrived.find(nextDrain); it != arrived.end();
         it = arrived.find(nextDrain)) {
        PacketPtr pkt = it->second;
        arrived.erase(it);

        const bool lastChunk =
            lastRow && lastSegment && nextDrain + 1 == chunks.size();
        pushBeats(pkt->getAddr(), pkt->getConstPtr<uint8_t>(),
                  pkt->getSize(), lastChunk);
        delete pkt;
        ++nextDrain;
    }

    if (!beatQueue.empty() && !dataBlocked && !sendEvent.scheduled())
        schedule(sendEvent, clockEdge(Cycles(1)));
}

// Scala: StreamReader.out (StreamChannel: data, keep, last) entering
// data_queue. Beats sit on 8-byte boundaries of the physical address;
// keep marks which bytes are real. data_queue.enq.bits.last is
// out.last & io.lastRow & (last page segment).
void
DMA_read_page_split::pushBeats(Addr paddr, const uint8_t *data,
                               uint32_t size, bool lastChunk)
{
    const Addr end = paddr + size;
    for (Addr a = paddr; a < end; ) {
        const Addr base = a & ~Addr(7);
        const Addr stop = std::min(base + 8, end);

        StreamBeat beat{};
        for (Addr b = a; b < stop; ++b) {
            const unsigned lane = b - base;
            beat.data |= uint64_t(data[b - paddr]) << (8 * lane);
            beat.keep |= uint8_t(1u << lane);
        }
        beat.last = lastChunk && stop == end;
        beatQueue.push_back(beat);
        a = stop;
    }
}

// Scala: io.dataOut <> data_queue.io.deq -- one beat per cycle to
// ReadPostProcess, stalling while it is not ready.
void
DMA_read_page_split::sendBeats()
{
    if (dataBlocked || beatQueue.empty())
        return;

    RequestPtr req = std::make_shared<Request>(0, sizeof(StreamBeat), 0,
                                               requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(reinterpret_cast<const uint8_t *>(&beatQueue.front()));

    if (!dataSidePort.sendTimingReq(pkt)) {
        delete pkt;
        dataBlocked = true;
        return;
    }
    beatQueue.pop_front();
    if (!beatQueue.empty())
        schedule(sendEvent, clockEdge(Cycles(1)));
}

// Scala: back to main_idle, io.busy falls -- Load_ctl's wait_dma_finish
// sees it and issues the next row. Here that is the response on req_side.
void
DMA_read_page_split::finishRequest()
{
    DPRINTF(PICDMA, "row done\n");

    PacketPtr pkt = activeReq;
    activeReq = nullptr;
    state = State::Idle;

    pkt->makeResponse();
    if (!reqSidePort.sendTimingResp(pkt)) {
        reqSidePort.blockedResp = pkt;
        return;
    }
    if (reqSidePort.needRetry) {
        reqSidePort.needRetry = false;
        reqSidePort.sendRetryReq();
    }
}

// gem5 only: resend the "row done" response Load_ctl refused.
void
DMA_read_page_split::reqRespRetry()
{
    PacketPtr pkt = reqSidePort.blockedResp;
    panic_if(!pkt, "DMA_read_page_split: unexpected response retry");
    reqSidePort.blockedResp = nullptr;
    if (!reqSidePort.sendTimingResp(pkt)) {
        reqSidePort.blockedResp = pkt;
        return;
    }
    if (reqSidePort.needRetry) {
        reqSidePort.needRetry = false;
        reqSidePort.sendRetryReq();
    }
}

}  // namespace gem5
