#include "learning_gem5/PIC/Switch/switchCtrl.hh"

#include <algorithm>

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
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
    numSets(params.num_sets),
    numWays(params.num_ways),
    nWayPerLevel(params.n_way_per_level),
    picAvailLevels(params.pic_avail_levels),
    switchState(State::Idle),
    cacheModeEndWay(params.num_ways - 1),
    picLevels(0),
    setID(0), beginWay(0), endWay(0), reqLevels(0), opSuccess(false),
    currSwitchType(SwitchType::PIC2Cache),
    flushCtl(this),
    drainQueryEvent([this] { processDrainQueryEvent(); }, name() + ".drainQueryEvent"),
    queryEvent([this] { processQueryEvent(); }, name() + ".queryEvent"),
    switch2PICEvent([this] { processSwitch2PICEvent(); }, name() + ".switch2PICEvent"),
    switch2CacheEvent([this] { processSwitch2CacheEvent(); }, name() + ".switch2CacheEvent")
{
    panic_if(numWays == 0 || numSets == 0, "%s bad cache geometry", name());
    panic_if(nWayPerLevel == 0 || numWays % nWayPerLevel != 0, "%s num_ways %u not divisible by n_way_per_level %u", name(), numWays, nWayPerLevel);
    panic_if(picAvailLevels >= numWays / nWayPerLevel, "%s pic_avail_levels must leave at least one cache level", name());
    panic_if(numWays > PIC_MAX_WAYS, "%s num_ways %u > PIC_MAX_WAYS", name(), numWays);
}

SwitchController::CacheCtrlPort::cacheCtrlPort(
    const std::string &name, SwitchController *owner)
    : RequestPort(name, owner), owner(owner)
{
}

void
SwitchController::CacheCtrlPort::sendPacket(PacketPtr pkt)
{
    // FSM 保證一次只有一個 outstanding：drain → waymode → query 迴圈
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
    // FSM 保證一次只有一個 outstanding：drain → waymode → query 迴圈
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
SwitchController::InstPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleFlushResponse(pkt);
}

SwitchController::InstPort::ControlPort(
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

void
SwitchController::processSwitchEvent(bool allocate, uint8_t nLevels)
{
    assert(isIdle());
    currSwitchType = allocate ? SwitchType::Cache2PIC : SwitchType::PIC2Cache;
    reqLevels = nLevels;
    setID = 0;
    const uint32_t nWays = nWayPerLevel * nLevels;

    if (allocate) {
        opSuccess = (picLevels + nLevels <= picAvailLevels);
        if (!opSuccess) { switchState = State::Idle; return; } // TODO 回報失敗

        endWay = cacheModeEndWay;
        beginWay = cacheModeEndWay - nWays + 1;

        switchState = State::WaitCacheIdle;
        schedule(drainQueryEvent, clockEdge(Cycles(1)));
    } else {
        opSuccess = (picLevels >= nLevels);
        if (!opSuccess) { switchState = State::Idle; return; }

        beginWay = cacheModeEndWay + 1;
        endWay = cacheModeEndWay + nWays;
        picLevels -= nLevels;
        cacheModeEndWay += nWays;

        schedule(switch2CacheEvent, clockEdge(Cycles(1)));
    }
}

bool
SwitchController::handleControlRequest(PacketPtr pkt)
{
    panic_if(!isIdle(), "%s got a switch request while busy", name());

    const auto *p = pkt->getConstPtr<SwitchPayload>();
    processSwitchEvent(p->opType, p->nLevels);

    pkt->makeResponse();
    instPort.sendResponse(pkt);     // 或先存著，等 switch 做完再回
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

void
SwitchController::processDrainQueryEvent()
{
    const size_t pktSize = std::max(sizeof(DrainQueryPayload),
                                    sizeof(DrainQueryRespPayload));

    RequestPtr req = std::make_shared<Request>(
        picCtrlAddr(PicCtrlReg::DrainQuery), pktSize, 0, requestorId
    );

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();

    pkt->getPtr<DrainQueryPayload>()->wayID = beginWay;

    DPRINTF(SwitchCtrl, "DrainQuery: ways=[%u,%u]\n", beginWay, endWay);
    cacheCtrlPort.sendPacket(pkt);        // state 留在 WaitCacheIdle
}

void
SwitchController::processQueryEvent()
{
    const size_t pktSize = std::max(sizeof(CacheWayQueryPayload),
                                    sizeof(CacheWayQueryRespPayload));

    RequestPtr req = std::make_shared<Request>(picCtrlAddr(PicCtrlReg::WayQuery), pktSize, 0, requestorId);

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();

    *pkt->getPtr<CacheWayQueryPayload>() = CacheWayQueryPayload{setID, beginWay, endWay};

    DPRINTF(SwitchCtrl, "Query: set=%u ways=[%u,%u]\n", setID, beginWay, endWay);
    switchState = State::WaitDirResult;
    cacheCtrlPort.sendPacket(pkt);
}

bool
SwitchController::handleDirResponse(PacketPtr pkt)
{
    const auto reg = static_cast<PicCtrlReg>(pkt->getAddr() - PicCtrlBase);

    switch (reg) {
      case PicCtrlReg::DrainQuery: {
        const bool drained =
            pkt->getConstPtr<DrainQueryRespPayload>()->isDrained != 0;
        delete pkt;
        if (drained) {
            picLevels += reqLevels;
            schedule(switch2PICEvent, clockEdge(Cycles(1)));
        } else {
            schedule(drainQueryEvent, clockEdge(Cycles(10)));
        }
        return true;
      }

      case PicCtrlReg::WayMode: {
        delete pkt;
        if (currSwitchType == SwitchType::Cache2PIC) {
            schedule(queryEvent, clockEdge(Cycles(1)));
        } else {
            switchState = State::Idle;   // TODO 回 switch_resp
        }
        return true;
      }

      case PicCtrlReg::WayQuery: {
        const auto *resp = pkt->getConstPtr<CacheWayQueryRespPayload>();
        for (uint32_t i = 0; i < resp->numValid; ++i) {
            flushCtl.enqueueFlush(setID, resp->tag[i]);
        }
        delete pkt;
        processNextSet();
        return true;
      }

      default:
        panic("%s unexpected dir response offset %#llx", name(),
              (unsigned long long)(pkt->getAddr() - PicCtrlBase));
    }
}

bool
SwitchController::handleFlushResponse(PacketPtr pkt)
{
    panic_if(!flushCtl.handleFlushResponse(pkt),
             "FlushResp arrived while FlushCtl state=%d",
             (int)flushCtl.getState());

    if (switchState == State::WaitFlush && flushCtl.isFlushQueueEmpty()) {
        switchState = State::Idle;   // TODO 回 switch_resp
    }
    return true;
}

void
SwitchController::sendWayMode(bool picMode)
{
    RequestPtr req = std::make_shared<Request>(picCtrlAddr(PicCtrlReg::WayMode), sizeof(WayModePayload), 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();

    *pkt->getPtr<WayModePayload>() = WayModePayload{beginWay, endWay, picMode ? 1u : 0u};

    switchState = State::WaitWayMode;
    cacheCtrlPort.sendPacket(pkt);
}

void
SwitchController::processNextSet()
{
    setID++;
    if (setID < numSets) {
        switchState = State::CheckFinish;
        schedule(queryEvent, clockEdge(Cycles(1)));
        return;
    }
    // RTL should_finish: runtime_cache_Mode_endWayID -= nWays
    cacheModeEndWay -= nWayPerLevel * reqLevels;

    switchState = State::WaitFlush;           // RTL: resp_op_res
    if (flushCtl.isFlushQueueEmpty()) {
        switchState = State::Idle;            // TODO 通知 Scheduler
    }
}

void
SwitchController::processSwitch2PICEvent()
{
    sendWayMode(true);
}

void
SwitchController::processSwitch2CacheEvent()
{
    sendWayMode(false);
}


} // namespace gem5