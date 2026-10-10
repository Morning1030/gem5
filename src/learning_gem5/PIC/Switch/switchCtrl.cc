#include "learning_gem5/PIC/Switch/switchCtrl.hh"
#include "learning_gem5/PIC/cache_controller.hh"

#include <algorithm>

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
    cacheCtrlPort(name() + ".cache_ctrl_port", this),
    flushPort(name() + ".flush_port", this),
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
    // 較低優先權：同一 tick 的 port response 先處理，tick() 才看得到
    tickEvent([this] { tick(); }, name() + ".tickEvent",
              false, Event::CPU_Tick_Pri)
{
    panic_if(numWays == 0 || numSets == 0, "%s bad cache geometry", name());
    panic_if(nWayPerLevel == 0 || numWays % nWayPerLevel != 0, "%s num_ways %u not divisible by n_way_per_level %u", name(), numWays, nWayPerLevel);
    panic_if(picAvailLevels >= numWays / nWayPerLevel, "%s pic_avail_levels must leave at least one cache level", name());
    panic_if(numWays > PIC_MAX_WAYS, "%s num_ways %u > PIC_MAX_WAYS", name(), numWays);
    panic_if(cache == nullptr, "%s cache param not set", name());
    panic_if(picAvailLevels != totalLevels - 1, "%s pic_avail_levels %u != total_levels %u - 1", name(), picAvailLevels, totalLevels);
    panic_if(totalMatNum == 0 || totalMatNum % totalLevels != 0, "%s total_mat_num %u not divisible by total_levels %u", name(), totalMatNum, totalLevels);
}

SwitchController::CacheCtrlPort::CacheCtrlPort(
    const std::string &name, SwitchController *owner)
    : RequestPort(name, owner), owner(owner)
{
}

void
SwitchController::CacheCtrlPort::sendPacket(PacketPtr pkt)
{
    // FSM 保證一次只有一個 outstanding query
    panic_if(blockedPkt != nullptr,
             "%s second outstanding dir request", name());

    if (!sendTimingReq(pkt)) {
        blockedPkt = pkt;       // 等 recvReqRetry()，不自己重試
    }
}

void
SwitchController::CacheCtrlPort::recvReqRetry()
{
    panic_if(blockedPkt == nullptr,
             "%s retry without a blocked request", name());

    if (sendTimingReq(blockedPkt)) {
        blockedPkt = nullptr;
    }
}

bool
SwitchController::CacheCtrlPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleDirResponse(pkt);
}

SwitchController::FlushPort::FlushPort(
    const std::string &name, SwitchController *owner)
    : RequestPort(name, owner), owner(owner)
{
}

void
SwitchController::FlushPort::sendPacket(PacketPtr pkt)
{
    // flush FSM 保證一次只有一個 outstanding FlushReq
    panic_if(blockedPkt != nullptr,
             "%s second outstanding flush request", name());

    if (!sendTimingReq(pkt)) {
        blockedPkt = pkt;       // 等 recvReqRetry()，不自己重試
    }
}

void
SwitchController::FlushPort::recvReqRetry()
{
    panic_if(blockedPkt == nullptr,
             "%s retry without a blocked request", name());

    if (sendTimingReq(blockedPkt)) {
        blockedPkt = nullptr;
    }
}

bool
SwitchController::FlushPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleFlushResponse(pkt);
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
    panic_if(switchState != State::Idle || pendingInstPkt,
             "%s got a switch request while busy", name());

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

    // RTL: switch_req.fire（idle 那拍）→ 下一拍進 pre_check
    switchState = isAlloc ? State::ActPreCheck : State::DeactPreCheck;
    querySetPtr = 0;
    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
    return true;
}

Port &
SwitchController::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") return instPort;
    if (if_name == "cache_ctrl_port") return cacheCtrlPort;
    if (if_name == "flush_port") return flushPort;
    return ClockedObject::getPort(if_name, idx);
}

// =================================================================
//  tick — 每拍一次，照 RTL 語意：
//    先用本拍開始時的暫存器值算出所有組合訊號，再一次更新所有暫存器
// =================================================================

void
SwitchController::tick()
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
    const bool flushAllIdle = flushCtl.isFlushQueueEmpty();    // resp_op_res 條件

    DPRINTF(SwitchCtrl, "tick: sw=%d dir=%d set=%u wayPtr=%u valid=%d "
            "resFire=%d queryFire=%d flush=%d\n",
            (int)switchState, (int)dirState, querySetPtr, dirWayPtr,
            wayValid, resFire, queryFire, (int)flushCtl.getState());

    // ===== 2. flush FSM（只看本拍開始時的 queue，本拍新 enqueue 的下一拍才看得到）=====
    flushCtl.tick();

    // ===== 3. Directory 掃描 FSM =====
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

    // ===== 4. SwitchCtl FSM =====
    switch (switchState) {
      case State::Idle:
        if (needUnblock) {                     // RTL: 回到 switch_idle 時 sinkA 才放行
            cache->unblockForSwitch();
            needUnblock = false;
        }
        break;

      case State::ActPreCheck:
        cache->blockForSwitch();               // RTL: switchIdle = 0 → sinkA 被擋
        needUnblock = true;
        if (picLevels + reqLevels > picAvailLevels) {
            opSuccess = false;                 // respMatIdBegin 保留舊值
            switchState = State::RespOpRes;
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
        }                                      // 否則留在 pre_check，下一拍再檢查
        break;

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
        break;

      case State::RespOpRes:
        if (flushAllIdle) {                    // !flush_queue_not_empty && flush_idle
            // 先回 Idle 再回應：Scheduler 可能在 response 裡同步送下一個 request
            switchState = State::Idle;
            picLevelsForAssert = picLevels;    // RTL: switch_resp.fire 才更新
            respondSwitch();
        }
        break;
    }

    // ===== 5. 有事要做就繼續跑下一拍 =====
    if ((switchState != State::Idle || dirState != DirState::Idle ||
         !flushCtl.isFlushQueueEmpty() || needUnblock) &&
        !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
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
    cacheCtrlPort.sendPacket(pkt);
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
    return true;                     // 完成條件由 tick() 的 RespOpRes 判斷
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
