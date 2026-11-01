#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "base/trace.hh"
#include "debug/CacheController.hh"
#include "learning_gem5/PIC/Switch/pic_payloads.hh"
#include "learning_gem5/PIC/Switch/switchCtrl.hh"
#include "mem/cache/mshr.hh"

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
    maxHeldSinkA(params.max_held_sink_a),
    picStats(this),
    sinkAReplayEvent([this] { processSinkAReplay(); },
                     name() + ".sinkAReplayEvent"),
    bankArb(params.bank_arb),
    bankBusyDelay(params.bank_busy_delay),
    beatBytes(params.bank_beat_bytes)
{
    // picTags() 用 static_cast，型別在這裡檢查一次
    fatal_if(!dynamic_cast<PICLLCTags*>(tags),
             "%s: tags must be PICLLCTags", name());
    fatal_if(beatBytes == 0, "%s: bank_beat_bytes must be > 0", name());
    bankBusyCnt.assign(picTags()->bankCount(), 0);
}

CacheController::PicStats::PicStats(statistics::Group *parent)
    : statistics::Group(parent, "pic"),
      ADD_STAT(heldSinkAPeak, statistics::units::Count::get(),
               "Most sinkA requests held at once during a switch")
{
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
    if (!isSinkCLike(pkt) && (sinkABlocked() || !heldSinkA.empty())) {
        heldSinkA.push_back(pkt);
        DPRINTF(CacheController, "SinkA held %s addr=%#x (held=%u)\n",
                pkt->cmdString(), pkt->getAddr(), (unsigned)heldSinkA.size());
        if (heldSinkA.size() > picStats.heldSinkAPeak.value())
            picStats.heldSinkAPeak = heldSinkA.size();
        panic_if(maxHeldSinkA && heldSinkA.size() > maxHeldSinkA,
                 "%s: %u sinkA requests held during a switch "
                 "(max_held_sink_a = %u)", name(),
                 (unsigned)heldSinkA.size(), maxHeldSinkA);
        return;
    }
    Cache::recvTimingReq(pkt);
}

void
CacheController::processSinkAReplay()
{
    // RTL: sinkA 每拍收一筆
    if (sinkABlocked() || heldSinkA.empty())
        return;                 // 下次 unblockForSwitch() 再排
    PacketPtr pkt = heldSinkA.front();
    heldSinkA.pop_front();
    DPRINTF(CacheController, "SinkA replay %s addr=%#x (held=%u)\n",
            pkt->cmdString(), pkt->getAddr(), (unsigned)heldSinkA.size());
    Cache::recvTimingReq(pkt);
    if (!sinkABlocked() && !heldSinkA.empty() && !sinkAReplayEvent.scheduled())
        schedule(sinkAReplayEvent, clockEdge(Cycles(1)));
}

Port &
CacheController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "pic_ctrl_port") return picCtrlPort;
    if (if_name == "pic_flush_port") return picFlushPort;
    return Cache::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Cache / PIC bank 衝突（RTL: BankSellPIC L71、L112-116）
//   cache 路徑用某個 bank 的那拍 picMem_ready = false；AccessBankArb 看到
//   bank 不 ready 會進 Block、凍結所有 client，每拍自己 poll，不用叫醒它。
//   beat = 8B、bank = beat % 4：從 Chipyard 預設參數推得（writeBytes=8、
//   portFactor=4），待確認實際 config。
//   假設 CacheController 和 AccessBankArb 在同一個 clock domain。
// ---------------------------------------------------------------------------

void
CacheController::occupyBanks(Addr offset, unsigned size)
{
    if (!bankArb || size == 0)
        return;

    const unsigned numBanks = picTags()->bankCount();
    const Addr firstBeat = offset / beatBytes;
    const Addr lastBeat  = (offset + size - 1) / beatBytes;
    const Tick start = clockEdge(bankBusyDelay);

    // 第 i 個 beat 在 start 之後第 i 拍，佔用 bank (beat % numBanks) 一拍
    for (Addr beat = firstBeat; beat <= lastBeat; ++beat) {
        const unsigned bank = beat % numBanks;
        const Tick t = start + cyclesToTicks(Cycles(beat - firstBeat));
        DPRINTF(CacheController, "BankBusy: bank=%u beat=%llu tick=%llu\n",
                bank, (unsigned long long)beat, (unsigned long long)t);
        markBank(bank, t, t + clockPeriod());
    }
}

void
CacheController::markBank(unsigned bank, Tick start, Tick end)
{
    // 優先權比 AccessBankArb 的 tickEvent（Default_Pri）早：arbiter 同一拍
    // 就看得到。event 執行完自動刪除（第三個參數 true）
    constexpr Event::Priority pri = Event::Default_Pri - 1;

    auto *busy = new EventFunctionWrapper([this, bank] {
        if (++bankBusyCnt[bank] == 1)
            bankArb->setBankReady(bank, false);
    }, name() + ".bankBusyEvent", true, pri);

    auto *ready = new EventFunctionWrapper([this, bank] {
        assert(bankBusyCnt[bank] > 0);
        if (--bankBusyCnt[bank] == 0)
            bankArb->setBankReady(bank, true);
    }, name() + ".bankFreeEvent", true, pri);

    schedule(busy, start);
    schedule(ready, end);
}

void
CacheController::satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                                bool deferred_response,
                                bool pending_downgrade)
{
    // 命中的讀寫，以及 miss 之後回資料給 L1（serviceMSHRTargets 也走這裡）。
    // parent 會把 pkt 改成 response，先記下有沒有搬資料；Upgrade /
    // Invalidate 沒有資料，不算
    const bool movesData = pkt->isRead() || pkt->isWrite();
    const Addr offset = pkt->getOffset(blkSize);
    const unsigned size = pkt->getSize();

    Cache::satisfyRequest(pkt, blk, deferred_response, pending_downgrade);

    if (movesData)
        occupyBanks(offset, size);
}

void
CacheController::recvTimingResp(PacketPtr pkt)
{
    // DRAM 的資料填進 blk（RTL: refill 寫 BankedStore）。handleFill 不是
    // virtual，只能在這裡判斷。parent 會 pop senderState 並 delete pkt，
    // 所以要在呼叫之前判斷；條件和 BaseCache::recvTimingResp 的 is_fill
    // 一致，allocate=false 時資料只進 tempBlock，不算
    bool fill = false;
    if (!pkt->isWrite() && pkt->hasData() && !pkt->isError()) {
        const MSHR *mshr = dynamic_cast<const MSHR*>(pkt->senderState);
        fill = mshr && !mshr->isForward && mshr->allocOnFill() &&
               (pkt->isRead() || mshr->wasWholeLineWrite);
    }

    Cache::recvTimingResp(pkt);

    if (fill)
        occupyBanks(0, blkSize);

    // MSHR 只在 BaseCache::recvTimingResp 裡釋放：最後一筆做完時通知
    // switch（pre_check 在等 drain 的話它會重新檢查）
    if (switchCtrl && mshrQueue.isEmpty())
        switchCtrl->notifyMshrDrained();
}

bool
CacheController::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                        PacketList &writebacks)
{
    // 只算 L1 writeback / WriteClean 寫進 blk（RTL: sinkC ReleaseData 寫
    // BankedStore）。命中已經在 satisfyRequest 算過，這裡不重複
    const bool wbData = pkt->hasData() &&
        (pkt->isWriteback() || pkt->cmd == MemCmd::WriteClean);
    const Addr offset = pkt->getOffset(blkSize);
    const unsigned size = pkt->getSize();

    const bool satisfied = Cache::access(pkt, blk, lat, writebacks);

    // blk 是 nullptr = 沒 allocate，資料往下送，沒寫進這層
    if (wbData && blk != nullptr)
        occupyBanks(offset, size);
    return satisfied;
}

PacketPtr
CacheController::evictBlock(CacheBlk *blk)
{
    // 被替換掉的 dirty block 讀出來寫回（RTL: SourceC 讀 BankedStore）
    PacketPtr pkt = Cache::evictBlock(blk);
    if (pkt && pkt->hasData())
        occupyBanks(0, blkSize);
    return pkt;
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
        occupyBanks(0, blkSize);             // RTL: ProbeAckData 寫回 BankedStore
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
    // ReleaseData 要從 BankedStore 讀出資料；Release（CleanEvict）沒有資料
    if (wb->hasData())
        occupyBanks(0, blkSize);

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
