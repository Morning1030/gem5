#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "base/trace.hh"
#include "debug/CacheController.hh"
#include "learning_gem5/PIC/Switch/pic_payloads.hh"

namespace gem5
{

CacheController::CacheController(const CacheControllerParams &params)
    : Cache(params),
    flushLookupLat(params.flush_lookup_lat),
    flushMissDoneLat(params.flush_miss_done_lat),
    flushReleaseAckLat(params.flush_release_ack_lat),
    flushReleaseDataAckLat(params.flush_release_data_ack_lat),
    flushScheduleEvent([this] { processFlushSchedule(); }, name() + ".flushScheduleEvent"),
    flushDoneEvent([this] { processFlushDone(); }, name() + ".flushDoneEvent"),
    flushProbeLat(params.flush_probe_lat),
    flushProbeDataLat(params.flush_probe_data_lat),
    flushRequestorId(params.system->getRequestorId(this, "pic_flush")),
    flushReleaseEvent([this] { processFlushRelease(); }, name() + ".flushReleaseEvent"),
    flushWbWaitEvent([this] { processFlushWbWait(); }, name() + ".flushWbWaitEvent"),
    picCtrlPort(params.name + ".pic_ctrl_port", this),
    picFlushPort(params.name + ".pic_flush_port", this),
    sinkAReplayEvent([this] { processSinkAReplay(); },
                     name() + ".sinkAReplayEvent")
{
    // picTags() 用 static_cast，型別在這裡檢查一次
    fatal_if(!dynamic_cast<PICLLCTags*>(tags),
             "%s: tags must be PICLLCTags", name());
}

bool
CacheController::isSinkCLike(const PacketPtr pkt)
{
    // RTL sinkC：Release / ReleaseData（L1 evict）。cacheResponding 和
    // express snoop 是 gem5 coherence 機制本身的封包，也不能擋
    return pkt->isEviction() || pkt->cmd == MemCmd::WriteClean ||
           pkt->cacheResponding() || pkt->isExpressSnoop();
}

void
CacheController::recvTimingReq(PacketPtr pkt)
{
    // switch 期間（或還有沒放完的）A 類請求排隊，保持原本順序
    if (!isSinkCLike(pkt) && (switchBlocked || !heldSinkA.empty())) {
        heldSinkA.push_back(pkt);
        DPRINTF(CacheController, "SinkA held %s addr=%#x (held=%u)\n",
                pkt->cmdString(), pkt->getAddr(), (unsigned)heldSinkA.size());
        return;
    }
    Cache::recvTimingReq(pkt);
}

void
CacheController::processSinkAReplay()
{
    // RTL: sinkA 每拍收一筆
    if (switchBlocked || heldSinkA.empty())
        return;
    PacketPtr pkt = heldSinkA.front();
    heldSinkA.pop_front();
    Cache::recvTimingReq(pkt);
    if (!switchBlocked && !heldSinkA.empty() && !sinkAReplayEvent.scheduled())
        schedule(sinkAReplayEvent, clockEdge(Cycles(1)));
}

Port &
CacheController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "pic_ctrl_port") return picCtrlPort;
    if (if_name == "pic_flush_port") return picFlushPort;
    return Cache::getPort(if_name, idx);
}

bool
CacheController::handleQueryWayState(PacketPtr pkt)
{
    // RTL: query fire 這拍 directory.io.ready = 0，Scheduler 不能接受 flushReq
    const Tick fireTick = clockEdge();
    lastDirQueryTick = fireTick;

    // 同一 tick 但 flush 先被處理（gem5 事件順序）→ 那筆 flush 其實沒被接受，順延 1 拍
    if (flushTxn.pkt && flushAcceptTick == fireTick &&
        flushScheduleEvent.scheduled()) {
        flushAcceptTick += clockPeriod();
        reschedule(flushScheduleEvent,
                   flushScheduleEvent.when() + clockPeriod());
    }

    const auto *q = pkt->getConstPtr<CacheWayQueryPayload>();
    const uint32_t setID    = q->setID;
    const uint32_t beginWay = q->beginWay;
    const uint32_t endWay   = q->endWay;

    panic_if(beginWay > endWay ||
             endWay >= (uint32_t)picTags()->getWayAllocationMax() ||
             endWay - beginWay + 1 > PIC_MAX_WAYS,          // ← D-7
             "QueryWayState: bad range set=%u ways=[%u,%u]",
             setID, beginWay, endWay);

    pkt->makeResponse();
    auto *resp = pkt->getPtr<CacheWayQueryRespPayload>();
    *resp = CacheWayQueryRespPayload{};

    // RTL 只回報 state != INVALID 的 way
    for (uint32_t w = beginWay; w <= endWay; ++w) {
        CacheBlk *blk = picTags()->getBlk(setID, w);
        if (blk->isValid()) {
            resp->wayID[resp->numValid] = w;
            resp->tag[resp->numValid]   = blk->getTag();
            resp->numValid++;
        }
    }

    DPRINTF(CacheController, "QueryWayState: set=%u ways=[%u,%u] valid=%u\n",
            setID, beginWay, endWay, resp->numValid);

    // 快照在 save_regout 那拍（T+1）送到；SwitchController 的 tick 優先權較低，
    // 同一 tick 一定會先收到 response，T+2 的 tick 就開始掃描
    picCtrlPort.schedTimingResp(pkt, clockEdge(Cycles(1)));
    return true;
}

bool
CacheController::handleFlushReq(PacketPtr pkt)
{
    panic_if(flushTxn.pkt != nullptr, "%s second outstanding FlushReq", name());
    flushTxn.pkt = pkt;

    // RTL FlushReqRouter：deal_idle 收下（c0）→ send_flush_req 下一拍才送進 Scheduler
    // RTL: 和 query fire 同一拍 → flushReq.ready = 0，再下一拍才被接受
    Tick accept = clockEdge(Cycles(1));
    if (accept == lastDirQueryTick)
        accept += clockPeriod();
    flushAcceptTick = accept;

    // 接受(c0) 讀 dir → c1 MSHR latch meta → c2 schedule
    schedule(flushScheduleEvent, accept + cyclesToTicks(flushLookupLat));
    return true;
}

CacheBlk *
CacheController::findFlushBlk(uint32_t setID, Addr tag)
{
    for (uint32_t w = 0; w < (uint32_t)picTags()->getWayAllocationMax(); ++w) {
        CacheBlk *b = picTags()->getBlk(setID, w);
        if (b->isValid() && b->getTag() == tag)
            return b;
    }
    return nullptr;
}

void
CacheController::processFlushDone()
{
    PacketPtr pkt = flushTxn.pkt;
    flushTxn.pkt = nullptr;
    pkt->makeResponse();
    picFlushPort.schedTimingResp(pkt, curTick());  // SourceX → FlushReqRouter → flushDone
}

bool
CacheController::handlePicCtrlReq(PacketPtr pkt)
{
    // SwitchController 只送 WayQuery；way mode、drain、switch busy 都是
    // 直接呼叫（setPicWays / isScheNoOtherWorks / blockForSwitch），不走 port
    const auto reg = static_cast<PicCtrlReg>(pkt->getAddr() - PicCtrlBase);
    panic_if(reg != PicCtrlReg::WayQuery, "%s unsupported dir offset %#llx",
             name(), (unsigned long long)(pkt->getAddr() - PicCtrlBase));
    // WayQuery 要拿資料回來，用 ReadReq
    panic_if(!pkt->isRead(), "%s wrong command %s for WayQuery", name(),
             pkt->cmdString());

    return handleQueryWayState(pkt);
}

bool
CacheController::innerHasCopy(Addr addr, bool secure)
{
    if (!forwardSnoops)
        return false;                       // 上面沒有 coherent cache

    RequestPtr req = std::make_shared<Request>(
        addr, blkSize, secure ? Request::SECURE : 0, flushRequestorId);
    Packet snoop(req, MemCmd::CleanEvict);
    snoop.setExpressSnoop();
    snoop.senderState = nullptr;
    cpuSidePort.sendTimingSnoopReq(&snoop);  // eviction snoop 不會產生 response
    return snoop.isBlockCached();
}

bool
CacheController::probeInnerToN(CacheBlk *blk, Addr addr)
{
    RequestPtr req = std::make_shared<Request>(
        addr, blkSize, blk->isSecure() ? Request::SECURE : 0, flushRequestorId);
    Packet probe(req, MemCmd::ReadExReq);    // invalidating + needsResponse
    probe.allocate();

    // L1：一律 invalidate；只有 dirty 才 respond（把資料填進 probe）
    cpuSidePort.sendAtomicSnoop(&probe);

    if (probe.cacheResponding()) {           // ProbeAckData
        probe.writeDataToBlock(blk->data, blkSize);
        blk->setCoherenceBits(CacheBlk::DirtyBit);   // RTL: meta.dirty := true
        return true;
    }
    return false;                            // ProbeAck（無資料）
}

PacketPtr
CacheController::makeCleanEvict(CacheBlk *blk)
{
    assert(!writebackClean);
    assert(blk->isValid() && !blk->isSet(CacheBlk::DirtyBit));

    RequestPtr req = std::make_shared<Request>(
        regenerateBlkAddr(blk), blkSize, 0, Request::wbRequestorId);
    if (blk->isSecure())
        req->setFlags(Request::SECURE);
    req->taskId(blk->getTaskId());

    PacketPtr pkt = new Packet(req, MemCmd::CleanEvict);
    pkt->allocate();
    return pkt;
}

void
CacheController::processFlushSchedule()
{
    const auto *f = flushTxn.pkt->getConstPtr<CacheFlushPayload>();
    CacheBlk *blk = findFlushBlk(f->setID, f->tag);

    if (!blk) {                               // !meta.hit：只剩 s_flush
        schedule(flushDoneEvent, clockEdge(flushMissDoneLat));
        return;
    }

    const Addr addr = regenerateBlkAddr(blk);

    // RTL: hit && clients == 0 → w_rprobeackfirst 已滿足，同一拍 Release
    if (!innerHasCopy(addr, blk->isSecure())) {
        processFlushRelease();
        return;
    }

    // RTL: s_rprobe → SourceB Probe(toN) → 等 ProbeAck[Data]
    const bool gotData = probeInnerToN(blk, addr);

    // L1 的 writeback 還在路上（在 L1 write buffer，atomic probe 看不到）：
    // RTL MSHR 會等這筆 Release 進來；這裡下一拍重來，屆時 block 已被更新成 dirty
    if (innerHasCopy(addr, blk->isSecure())) {
        DPRINTF(CacheController, "Flush addr=%#x wait in-flight writeback\n",
                addr);
        schedule(flushScheduleEvent, clockEdge(Cycles(1)));
        return;
    }

    DPRINTF(CacheController, "Flush probe addr=%#x %s\n",
            addr, gotData ? "ProbeAckData (dirty promoted)" : "ProbeAck");

    schedule(flushReleaseEvent,
             clockEdge(gotData ? flushProbeDataLat : flushProbeLat));
}

void
CacheController::processFlushRelease()
{
    const auto *f = flushTxn.pkt->getConstPtr<CacheFlushPayload>();
    CacheBlk *blk = findFlushBlk(f->setID, f->tag);

    if (!blk) {
        // probe 期間被下層 snoop 拿走（類似 RTL nestedwb b_toN）
        schedule(flushDoneEvent, clockEdge(flushMissDoneLat));
        return;
    }

    const bool dirty = blk->isSet(CacheBlk::DirtyBit);
    PacketPtr wb = (dirty || writebackClean) ? writebackBlk(blk) : makeCleanEvict(blk);

    // 記下位址要在 invalidate 之前
    flushTxn.wbAddr   = regenerateBlkAddr(blk);
    flushTxn.wbSecure = blk->isSecure();
    flushTxn.wbDirty  = dirty;

    invalidateBlock(blk);                       // dir write INVALID（和 Release 同一拍）
    allocateWriteBuffer(wb, clockEdge());

    DPRINTF(CacheController, "Flush release addr=%#x %s\n", flushTxn.wbAddr, dirty ? "ReleaseData" : "Release");

    // 不再直接排 FlushDone：先等下游接受
    schedule(flushWbWaitEvent, clockEdge(Cycles(1)));
}

void
CacheController::processFlushWbWait()
{
    // 還在 write buffer → 下游還沒接受（含 DRAM write queue 滿造成的反壓）
    if (writeBuffer.findMatch(flushTxn.wbAddr, flushTxn.wbSecure)) {
        schedule(flushWbWaitEvent, clockEdge(Cycles(1)));
        return;
    }

    DPRINTF(CacheController, "Flush release accepted addr=%#x\n",
            flushTxn.wbAddr);

    // outer → ReleaseAck → w_releaseack → s_flush (SourceX)
    schedule(flushDoneEvent,
             clockEdge(flushTxn.wbDirty ? flushReleaseDataAckLat
                                        : flushReleaseAckLat));
}

} // namespace gem5
