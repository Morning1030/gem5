#include "learning_gem5/PIC/Switch/switchCtrl.hh"
#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

SwitchController::SwitchController(const SwitchControllerParams &params)
    :ClockedObject(params),
    instPort(name() + ".inst_port", this),
    requestorId(params.system->getRequestorId(this, "SwitchController")),
    cache(params.cache),
    numSets(params.num_sets),
    numWays(params.num_ways),
    nWayPerLevel(params.n_way_per_level),
    picAvailLevels(params.pic_avail_levels),
    totalMatNum(params.total_mat_num),
    totalLevels(params.n_way_per_level ?
                params.num_ways / params.n_way_per_level : 0),
    nMatPerLevel(totalLevels ? params.total_mat_num / totalLevels : 0),
    cacheModeEndWay(params.num_ways - 1),
    picLevels(0),
    flushCtl(this),
    // 較低優先權：同一 tick 的 port response 先處理，event 才看得到
    preCheckEvent([this] { processPreCheckEvent(); },
                  name() + ".preCheckEvent", false, SwitchEventPri),
    scanEvent([this] { processScanEvent(); },
              name() + ".scanEvent", false, SwitchEventPri),
    respEvent([this] { processRespEvent(); },
              name() + ".respEvent", false, SwitchEventPri),
    unblockEvent([this] { processUnblockEvent(); },
                 name() + ".unblockEvent", false, SwitchEventPri),
    // Default_Pri：和 requester 自己送 request 的時機一樣（同拍 unblock 之前）
    instRetryEvent([this] { processInstRetryEvent(); },
                   name() + ".instRetryEvent")
{
    cacheSidePorts.reserve(NumCacheSide);
    for (PortID i = 0; i < NumCacheSide; ++i) {
        cacheSidePorts.emplace_back(
            csprintf("%s.cache_side[%d]", name(), i), this, i);
    }

    panic_if(numWays == 0 || numSets == 0, "%s bad cache geometry", name());
    panic_if(nWayPerLevel == 0 || numWays % nWayPerLevel != 0, "%s num_ways %u not divisible by n_way_per_level %u", name(), numWays, nWayPerLevel);
    panic_if(picAvailLevels >= numWays / nWayPerLevel, "%s pic_avail_levels must leave at least one cache level", name());
    panic_if(numWays > PIC_MAX_WAYS, "%s num_ways %u > PIC_MAX_WAYS", name(), numWays);
    panic_if(cache == nullptr, "%s cache param not set", name());
    panic_if(picAvailLevels != totalLevels - 1, "%s pic_avail_levels %u != total_levels %u - 1", name(), picAvailLevels, totalLevels);
    panic_if(totalMatNum == 0 || totalMatNum % totalLevels != 0, "%s total_mat_num %u not divisible by total_levels %u", name(), totalMatNum, totalLevels);
}

void
SwitchController::init()
{
    ClockedObject::init();
    for (const auto &port : cacheSidePorts) {
        fatal_if(!port.isConnected(), "%s not connected", port.name());
    }
    cache->setSwitch(this);            // MSHR drain 時通知這裡
    // RTL: runtime_PIC_Mode_levels 是 RegInit(0)。有接 Switch 時 PIC way
    // 只能透過 ALLOC 產生，tags 的 initial_pic_ways 必須是 0
    fatal_if(cache->picTags()->activePICWays() != 0,
             "%s: tags start with PIC ways set; with a SwitchController, "
             "PIC ways must come from ALLOC (initial_pic_ways = 0)", name());
}

SwitchController::CacheSidePort::CacheSidePort(
    const std::string &name, SwitchController *owner, PortID id)
    : RequestPort(name, owner, id), owner(owner)
{
}

void
SwitchController::CacheSidePort::sendPacket(PacketPtr pkt)
{
    // FSM 保證每個 idx 一次只有一個 outstanding request
    panic_if(blockedPkt != nullptr,
             "%s second outstanding request", name());

    if (!sendTimingReq(pkt)) {
        blockedPkt = pkt;       // 等 recvReqRetry()，不自己重試
    }
}

void
SwitchController::CacheSidePort::recvReqRetry()
{
    panic_if(blockedPkt == nullptr,
             "%s retry without a blocked request", name());

    if (sendTimingReq(blockedPkt)) {
        blockedPkt = nullptr;
    }
}

bool
SwitchController::CacheSidePort::recvTimingResp(PacketPtr pkt)
{
    switch (id) {
      case DirQuery:
        return owner->handleDirResponse(pkt);
      case Flush:
        return owner->handleFlushResponse(pkt);
      default:
        panic("%s response on unknown idx %d", name(), id);
    }
}

SwitchController::InstPort::InstPort(
    const std::string &name, SwitchController *owner)
    : ResponsePort(name, owner), owner(owner)
{
}

Tick
SwitchController::InstPort::recvAtomic(PacketPtr pkt)
{
    panic("%s atomic access unsupported", name());
}

void
SwitchController::InstPort::recvFunctional(PacketPtr pkt)
{
    panic("%s functional access unsupported", name());
}

bool
SwitchController::InstPort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleControlRequest(pkt);
}

void
SwitchController::InstPort::recvRespRetry()
{
    assert(blockedResponse != nullptr);

    PacketPtr pkt = blockedResponse;
    if (sendTimingResp(pkt)) {
        blockedResponse = nullptr;
    }
}

AddrRangeList
SwitchController::InstPort::getAddrRanges() const
{
    return {};
}

void
SwitchController::InstPort::sendResponse(PacketPtr pkt)
{
    panic_if(blockedResponse != nullptr,
             "%s attempted to queue two blocked control responses",
             name());

    if (!sendTimingResp(pkt)) {
        blockedResponse = pkt;
    }
}

bool
SwitchController::handleControlRequest(PacketPtr pkt)
{
    // RTL: switch_req.ready = (state === switch_idle)：忙碌時 stall。
    // 回 false，回到 switch_idle 那拍再 sendRetryReq()（requester 要保留
    // 封包，在 recvReqRetry 重送）
    if (switchState != State::Idle || pendingInstPkt) {
        DPRINTF(SwitchCtrl, "SwitchReq while busy: stall, retry at idle\n");
        needInstRetry = true;
        return false;
    }

    panic_if(!pkt->isWrite(), "%s switch request must be a WriteReq", name());
    panic_if(pkt->getSize() != sizeof(SwitchReqPayload),
             "%s switch payload size mismatch: got=%u expected=%u",
             name(), pkt->getSize(),
             static_cast<unsigned>(sizeof(SwitchReqPayload)));

    const auto *p = pkt->getConstPtr<SwitchReqPayload>();
    isAlloc   = p->opType;
    reqLevels = p->nLevels;
    pendingInstPkt = pkt;

    DPRINTF(SwitchCtrl, "SwitchReq: %s nLevels=%u\n",
            isAlloc ? "Cache2PIC" : "PIC2Cache", (unsigned)reqLevels);

    // RTL: switch_req.ready = (state === switch_idle)，switch_resp.fire 的
    // 下一拍才回到 idle。在那之前收到的 request（同步送來的）當成 idle 那拍
    // 才 fire；fire 的下一拍進 pre_check。idle 那拍的 unblock 照常做
    // （RTL switchIdle = 1）
    const Tick fireTick = std::max(clockEdge(), reqReadyAt);
    switchState = isAlloc ? State::ActPreCheck : State::DeactPreCheck;
    querySetPtr = 0;
    // pre_check 那拍一開始 switchIdle 就是 0
    cache->blockForSwitchFrom(fireTick + clockPeriod());
    schedule(preCheckEvent, fireTick + clockPeriod());
    return true;
}

Port &
SwitchController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") return instPort;
    if (if_name == "cache_side") {
        panic_if(idx < 0 || idx >= NumCacheSide,
                 "%s cache_side[%d] out of range", name(), idx);
        return cacheSidePorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

// =================================================================
//  Event-driven switch FSM
//    每個 event 只在狀態會變的那拍執行，照 RTL 語意：
//    先用本拍開始時的暫存器值算出組合訊號，再一次更新暫存器。
//    flush 的 event 優先權較低（FlushEventPri），所以這裡讀到的
//    flush queue / flush 狀態都是本拍開始時的值。
// =================================================================

void
SwitchController::processPreCheckEvent()
{
    switch (switchState) {
      case State::ActPreCheck:
        cache->blockForSwitch();               // RTL: switchIdle = 0 → sinkA 被擋
        needUnblock = true;
        if (picLevels + reqLevels > picAvailLevels) {
            opSuccess = false;                 // respMatIdBegin 保留舊值
            switchState = State::RespOpRes;
            schedule(respEvent, clockEdge(Cycles(1)));
        } else if (cache->isScheNoOtherWorks()) {
            opSuccess = true;
            respMatIdBegin =
                totalMatNum - (picLevels + reqLevels) * nMatPerLevel;
            picLevels += reqLevels;
            endWay   = cacheModeEndWay;
            beginWay = cacheModeEndWay - nWayPerLevel * reqLevels + 1;
            cache->setPicWays(beginWay, endWay, true);
            DPRINTF(SwitchCtrl, "Cache2PIC: ways=[%u,%u]\n", beginWay, endWay);
            switchState = State::ActQueryDir;
            schedule(scanEvent, clockEdge(Cycles(1)));
        } else if (!cache->mshrEmpty()) {
            // 還有 MSHR：等 CacheController 在最後一筆做完時
            // notifyMshrDrained()，不用每拍問
            waitingMshrDrain = true;
        } else {
            // 只剩 writeback / flush：這兩個沒有 hook，下一拍再檢查（輪詢）
            schedule(preCheckEvent, clockEdge(Cycles(1)));
        }
        break;

      case State::DeactPreCheck:
        cache->blockForSwitch();
        needUnblock = true;
        opSuccess = (picLevels >= reqLevels);
        if (opSuccess) {
            const uint32_t left = picLevels - reqLevels;
            const uint32_t matBegin = (totalLevels - left) * nMatPerLevel;
            // 全部釋放時會等於 totalMatNum（溢位），RTL 改成 totalMatNum - 1
            respMatIdBegin =
                matBegin == totalMatNum ? totalMatNum - 1 : matBegin;
            beginWay = cacheModeEndWay + 1;
            endWay   = cacheModeEndWay + nWayPerLevel * reqLevels;
            picLevels -= reqLevels;
            cacheModeEndWay = endWay;
            cache->setPicWays(beginWay, endWay, false);
            DPRINTF(SwitchCtrl, "PIC2Cache: ways=[%u,%u]\n", beginWay, endWay);
        }
        switchState = State::RespOpRes;
        schedule(respEvent, clockEdge(Cycles(1)));
        break;

      default:
        panic("%s preCheckEvent in state %d", name(), (int)switchState);
    }
}

void
SwitchController::processScanEvent()
{
    // ===== 1. 組合訊號：全部用本拍開始時的狀態 =====
    const bool dirBusy  = dirState != DirState::Idle;          // isDirQuerying
    const bool wayValid = dirState == DirState::Resp &&
                          snapIdx < snap.numValid &&
                          snap.wayID[snapIdx] == dirWayPtr;
    const Addr curTag   = wayValid ? snap.tag[snapIdx] : 0;
    const bool resFire  = wayValid && switchState == State::ActDirResp &&
                          flushCtl.queueNotFull();             // enq.ready
    const bool queryFire = switchState == State::ActQueryDir &&
                           dirState == DirState::Idle;         // dir ready 只在 idle

    DPRINTF(SwitchCtrl, "scan: sw=%d dir=%d set=%u wayPtr=%u valid=%d "
            "resFire=%d queryFire=%d flush=%d\n",
            (int)switchState, (int)dirState, querySetPtr, dirWayPtr,
            wayValid, resFire, queryFire, (int)flushCtl.getState());

    // flush queue 滿：Directory 和 SwitchCtl 這拍都不動，等 deq 叫醒
    if (wayValid && switchState == State::ActDirResp && !resFire) {
        scanWaitingQueue = true;
        return;
    }

    // ===== 2. Directory 掃描 FSM =====
    switch (dirState) {
      case DirState::Idle:
        if (queryFire)
            dirState = DirState::SaveRegout;
        break;

      case DirState::SaveRegout:
        panic_if(!snapValid, "%s snapshot missing at save_regout", name());
        dirWayPtr = beginWay;
        snapIdx   = 0;
        dirState  = DirState::Resp;
        break;

      case DirState::Resp:
        if (!wayValid || resFire) {            // INVALID 直接跳過；VALID 要等 fire
            if (wayValid)
                ++snapIdx;
            if (dirWayPtr == endWay) {
                dirState  = DirState::Idle;
                snapValid = false;
            } else {
                ++dirWayPtr;
            }
        }
        break;
    }

    // ===== 3. SwitchCtl FSM（掃描段）=====
    switch (switchState) {
      case State::ActQueryDir:
        if (queryFire) {
            sendWayQuery();
            switchState = State::ActDirResp;
        }
        break;

      case State::ActDirResp:
        if (resFire) {
            flushCtl.enqueueFlush(querySetPtr, curTag);
            switchState = State::CheckFinish;
        } else if (!dirBusy) {
            switchState = State::CheckFinish;
        }
        break;

      case State::CheckFinish:
        if (querySetPtr == numSets - 1 && !dirBusy) {
            cacheModeEndWay -= nWayPerLevel * reqLevels;
            switchState = State::RespOpRes;
        } else if (dirBusy) {
            switchState = State::ActDirResp;
        } else {
            ++querySetPtr;
            switchState = State::ActQueryDir;
        }
        break;

      default:
        panic("%s scanEvent in state %d", name(), (int)switchState);
    }

    // ===== 4. 下一拍 =====
    if (switchState == State::RespOpRes)
        schedule(respEvent, clockEdge(Cycles(1)));
    else
        schedule(scanEvent, clockEdge(Cycles(1)));
}

void
SwitchController::processRespEvent()
{
    panic_if(switchState != State::RespOpRes,
             "%s respEvent in state %d", name(), (int)switchState);
    // !flush_queue_not_empty && flush_idle；還沒好就等 flushAllDone()
    if (!flushCtl.isFlushQueueEmpty())
        return;

    // 先回 Idle 再回應：Scheduler 可能在 response 裡同步送下一個 request
    switchState = State::Idle;
    reqReadyAt = clockEdge(Cycles(1));         // RTL: 下一拍才是 switch_idle
    if (needInstRetry)                         // 被 stall 的 request 在 idle 那拍重送
        schedule(instRetryEvent, reqReadyAt);
    picLevelsForAssert = picLevels;            // RTL: switch_resp.fire 才更新
    cache->commitPicWays();                    // Bank 的 PIC 狀態同一拍更新
    if (needUnblock)                           // RTL: 回到 switch_idle 時 sinkA 才放行
        schedule(unblockEvent, clockEdge(Cycles(1)));
    respondSwitch();
}

void
SwitchController::processUnblockEvent()
{
    if (needUnblock) {
        DPRINTF(SwitchCtrl, "switch_idle: unblock sinkA\n");
        cache->unblockForSwitch();
        needUnblock = false;
    }
}

void
SwitchController::processInstRetryEvent()
{
    // requester 在 recvReqRetry 裡同步重送 → handleControlRequest 這拍 fire
    needInstRetry = false;
    instPort.sendRetryReq();
}

void
SwitchController::notifyMshrDrained()
{
    // 輪詢時 pre_check 在 MSHR 釋放那拍（CPU_Tick_Pri，cache 事件之後）
    // 就看得到；這裡排在同一拍，結果一樣。pre_check 會重新檢查全部條件
    if (waitingMshrDrain) {
        waitingMshrDrain = false;
        if (!preCheckEvent.scheduled())
            schedule(preCheckEvent, clockEdge());
    }
}

void
SwitchController::flushDequeued()
{
    // deq 這拍騰出位置，下一拍的 enq.ready 才看得到
    if (scanWaitingQueue) {
        scanWaitingQueue = false;
        schedule(scanEvent, clockEdge(Cycles(1)));
    }
}

void
SwitchController::flushAllDone()
{
    // flush 這拍回到 idle，下一拍的 resp_op_res 才看得到
    if (switchState == State::RespOpRes && !respEvent.scheduled())
        schedule(respEvent, clockEdge(Cycles(1)));
}

void
SwitchController::sendWayQuery()
{
    snapValid = false;
    const size_t sz = std::max(sizeof(CacheWayQueryPayload),
                               sizeof(CacheWayQueryRespPayload));
    RequestPtr req = std::make_shared<Request>(
        picCtrlAddr(PicCtrlReg::WayQuery), sz, 0, requestorId);
    // 送查詢參數、拿快照回來 → ReadReq（同 P2S / AutoLoadL 的用法）
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    *pkt->getPtr<CacheWayQueryPayload>() =
        CacheWayQueryPayload{querySetPtr, beginWay, endWay};

    DPRINTF(SwitchCtrl, "Query: set=%u ways=[%u,%u]\n",
            querySetPtr, beginWay, endWay);
    cacheSidePorts[DirQuery].sendPacket(pkt);
}

bool
SwitchController::handleDirResponse(PacketPtr pkt)
{
    const auto reg = static_cast<PicCtrlReg>(pkt->getAddr() - PicCtrlBase);
    panic_if(reg != PicCtrlReg::WayQuery,
             "%s unexpected dir response offset %#llx", name(),
             (unsigned long long)(pkt->getAddr() - PicCtrlBase));
    snap = *pkt->getConstPtr<CacheWayQueryRespPayload>();   // save_regout
    snapValid = true;
    delete pkt;
    return true;
}

bool
SwitchController::handleFlushResponse(PacketPtr pkt)
{
    panic_if(!flushCtl.handleFlushResponse(pkt),
             "FlushResp while FlushCtl state=%d", (int)flushCtl.getState());
    return true;                     // 完成條件由 respEvent 判斷
}

void
SwitchController::respondSwitch()
{
    DPRINTF(SwitchCtrl, "SwitchResp: success=%d matBegin=%u picLevels=%u\n",
            opSuccess, respMatIdBegin, picLevels);
    PacketPtr pkt = pendingInstPkt;
    pendingInstPkt = nullptr;
    panic_if(pkt->getSize() < sizeof(SwitchRespPayload),
             "%s switch request too small for response", name());
    pkt->makeResponse();
    // RTL: switch_resp.bits = respReg{op_success, avail_MatID_begin}
    *pkt->getPtr<SwitchRespPayload>() =
        SwitchRespPayload{static_cast<uint8_t>(opSuccess ? 1 : 0),
                          static_cast<uint8_t>(respMatIdBegin)};
    instPort.sendResponse(pkt);
}

} // namespace gem5
