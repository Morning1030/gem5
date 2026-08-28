#include "learning_gem5/PIC/scheduler.hh"
#include "mem/request.hh"
#include <algorithm>
#include <cassert>
#include "debug/Scheduler.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"


namespace gem5
{
Scheduler::Scheduler(const SchedulerParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    cacheControllerPort(params.name + ".mem_side_cc", this, DownstreamPortID::CC),
    p2sLPort(params.name + ".mem_side_p2sl", this, DownstreamPortID::P2SL),
    p2sRPort(params.name + ".mem_side_p2sr", this, DownstreamPortID::P2SR),
    p2sRTPort(params.name + ".mem_side_p2srt", this, DownstreamPortID::P2SRT),
    cacheBankPort(params.name + ".mem_side_cb", this, DownstreamPortID::CB),
    requestorId(params.system->getRequestorId(this, "Scheduler")),
    taskScheduler(this),
    switchController(this),
    decodeEvent([this]{this->processDecodeEvent();}, name() + ".decodeEvent")
{
}

Port&
Scheduler::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    else if (if_name == "mem_side_cc") {
        return cacheControllerPort;
    }
    else if (if_name == "mem_side_p2sl") {
        return p2sLPort;
    }
    else if (if_name == "mem_side_p2sr") {
        return p2sRPort;
    }
    else if (if_name == "mem_side_p2srt") {
        return p2sRTPort;
    }
    else if (if_name == "mem_side_cb") {
        return cacheBankPort;
    }

    return ClockedObject::getPort(if_name, idx);
}

void
Scheduler::handleFunctional(PacketPtr pkt)
{
    (void)pkt;
}

bool
Scheduler::handleRequest(PacketPtr pkt)
{
    if (instQueue.size() >= maxInstQueueSize) {
        instPort.markRequestRetry();
        return false;
    }

    DPRINTF(Scheduler, "Got request for addr %#llx\n",
            static_cast<unsigned long long>(pkt->getAddr()));

    instQueue.push_back(pkt);
    scheduleDecodeIfNeeded();
    return true;
}

bool
Scheduler::handleResponse(DownstreamPortID portID, PacketPtr pkt)
{
    DPRINTF(Scheduler, "Got downstream response for addr %#llx\n",
            static_cast<unsigned long long>(pkt->getAddr()));

    switch (portID) {
      case DownstreamPortID::CC:
        return switchController.handleResponse(pkt);

      case DownstreamPortID::P2SL:
      case DownstreamPortID::P2SR:
      case DownstreamPortID::P2SRT:
        panic_if(
            !pkt->isResponse(),
            "Scheduler expected P2S completion response");

        DPRINTFS(
            Scheduler,
            this,
            "P2S COMPLETION RESPONSE port=%u\n",
            static_cast<unsigned>(portID));

        delete pkt;
        taskScheduler.completeCurrentTask();

        // The launching SET_PARAM write has been held unanswered on
        // instPort since it was dispatched -- ack it now that the P2S
        // engine has actually drained its output through cb_port. This is
        // what lets a hung arbiter show up as "never exits" instead of a
        // clean (but meaningless) exit right after dispatch.
        panic_if(
            pendingCommandPkt == nullptr,
            "Scheduler got a P2S completion with no pending command");

        pendingCommandPkt->makeResponse();
        pendingCommandPkt->setLE<uint64_t>(0);
        instPort.sendPacket(pendingCommandPkt);
        pendingCommandPkt = nullptr;

        scheduleDecodeIfNeeded();
        return true;

      case DownstreamPortID::CB:
        delete pkt;
        return true;
    }

    return false;
}

AddrRangeList
Scheduler::getAddrRanges() const
{
    return {AddrRange(pic::MmioBase, pic::MmioBase + pic::MmioWindowSize)};
}

void
Scheduler::sendRangeChange()
{
    instPort.sendRangeChange();
}

void
Scheduler::scheduleDecodeIfNeeded()
{
    // do not decode another instruction while
    // TaskScheduler says hardware is busy
    if (!instQueue.empty() && taskScheduler.idle() &&
        !instPort.responseBlocked() && !decodeEvent.scheduled()) {
        schedule(decodeEvent, clockEdge(Cycles(1)));
    }
}

void
Scheduler::processDecodeEvent()
{
    if (!taskScheduler.idle() || instQueue.empty() ||
        instPort.responseBlocked()) {
        return;
    }

    PacketPtr pkt = instQueue.front();
    instQueue.pop_front();

    const Addr pktAddr = pkt->getAddr();
    const uint64_t regOffset = pktAddr - pic::MmioBase;

    // The current public protocol is one 64-bit write per SET register.
    if (!pkt->isWrite() || pkt->getSize() != pic::MmioAccessSize ||
        pktAddr < pic::MmioBase ||
        pktAddr >= pic::MmioBase + pic::MmioWindowSize) {
        warn("%s received malformed PIC MMIO request addr=%#llx size=%u\n",
             name(), static_cast<unsigned long long>(pktAddr),
             pkt->getSize());

        pkt->makeResponse();
        pkt->setLE<uint64_t>(pic::RetryResponse);
        instPort.sendPacket(pkt);

        instPort.trySendRequestRetry();
        scheduleDecodeIfNeeded();
        return;
    }

    const uint64_t dataPayload = pkt->getLE<uint64_t>();

    DPRINTF(Scheduler, "Decoding request addr=%#llx value=%#llx\n",
            static_cast<unsigned long long>(pktAddr),
            static_cast<unsigned long long>(dataPayload));

    switch (static_cast<pic::SetRegister>(regOffset)) {
      case pic::SetRegister::Src:
        src = dataPayload;
        DPRINTF(Scheduler, "SET_SRC = %#llx\n",
                static_cast<unsigned long long>(src));
        pkt->makeResponse();
        pkt->setLE<uint64_t>(0);
        instPort.sendPacket(pkt);
        scheduleDecodeIfNeeded();
        return;

      case pic::SetRegister::Dst:
        dst = dataPayload;
        DPRINTF(Scheduler, "SET_DST = %#llx\n",
                static_cast<unsigned long long>(dst));
        pkt->makeResponse();
        pkt->setLE<uint64_t>(0);
        instPort.sendPacket(pkt);
        scheduleDecodeIfNeeded();
        return;

      case pic::SetRegister::Size:
      {
        const pic::SizeFields size = pic::unpackSize(dataPayload);
        row = size.row;
        byte_per_row = size.bytesPerRow;
        offset = size.rowOffset;
        DPRINTF(Scheduler, "SET_SIZE = (row=%hu, byte_per_row=%hu, offset=%hu)\n",
                row, byte_per_row, offset);
        pkt->makeResponse();
        pkt->setLE<uint64_t>(0);
        instPort.sendPacket(pkt);
        scheduleDecodeIfNeeded();
        return;
      }

      case pic::SetRegister::Param:
        dispatchSetParam(pkt, dataPayload);
        return;

      default:
        panic("Scheduler decoded an out-of-range SET register offset %#llx",
              static_cast<unsigned long long>(regOffset));
    }
}

// mod: p2s_arbiter -- SET_PARAM is the only register write that launches
// downstream work; it's split out of processDecodeEvent() because, unlike
// SET_SRC/DST/SIZE (acked immediately), a P2S-launching SET_PARAM holds
// `pkt` unanswered on instPort until the targeted engine's write queue
// actually drains through cb_port (see handleResponse()). This is scoped
// to just the p2s -> arbiter test path: LOAD/STORE/IM2COL/ACC/EXE/SWITCH/
// QUERY are not implemented here.
void
Scheduler::dispatchSetParam(PacketPtr pkt, uint64_t dataPayload)
{
    const pic::ParamFields param = pic::unpackParam(dataPayload);

    auto rejectWithRetry = [&]() {
        pkt->makeResponse();
        pkt->setLE<uint64_t>(pic::RetryResponse);
        instPort.sendPacket(pkt);
        scheduleDecodeIfNeeded();
    };

    if (param.module == pic::ModuleId::P2SL) {
        const uint8_t precision =
            static_cast<uint8_t>(param.others & pic::mask(3));

        RequestPtr request = std::make_shared<Request>(
            0, sizeof(P2S_L_Payload), 0, requestorId);
        PacketPtr taskPkt = new Packet(request, MemCmd::WriteReq);
        taskPkt->allocate();

        const P2S_L_Payload payload{
            src,
            dst,
            static_cast<uint32_t>(byte_per_row),
            static_cast<uint8_t>(row),
            precision
        };
        taskPkt->setData(reinterpret_cast<const uint8_t *>(&payload));

        DPRINTF(Scheduler, "SET_PARAM -> P2S_L\n");

        if (!p2sLPort.sendPacket(taskPkt)) {
            delete taskPkt;
            rejectWithRetry();
            return;
        }

        pendingCommandPkt = pkt;
        taskScheduler.markBusy();
        return;
    }

    if (param.module == pic::ModuleId::P2SR) {
        const uint8_t bufNum =
            static_cast<uint8_t>(param.others & pic::mask(2));
        const uint8_t precision =
            static_cast<uint8_t>((param.others >> 2) & pic::mask(3));
        const bool transpose = ((param.others >> 5) & 0x1) != 0;

        RequestPtr request = std::make_shared<Request>(
            0, sizeof(P2S_R_Payload), 0, requestorId);
        PacketPtr taskPkt = new Packet(request, MemCmd::WriteReq);
        taskPkt->allocate();

        const P2S_R_Payload payload{
            src,
            dst,
            static_cast<uint32_t>(offset),
            static_cast<uint32_t>(row),
            static_cast<uint32_t>(byte_per_row),
            precision,
            bufNum
        };
        taskPkt->setData(reinterpret_cast<const uint8_t *>(&payload));

        MemSidePort &target = transpose ? p2sRTPort : p2sRPort;
        DPRINTF(Scheduler, "SET_PARAM -> %s\n",
                transpose ? "P2S_R_T" : "P2S_R");

        if (!target.sendPacket(taskPkt)) {
            delete taskPkt;
            rejectWithRetry();
            return;
        }

        pendingCommandPkt = pkt;
        taskScheduler.markBusy();
        return;
    }

    panic("Scheduler: SET_PARAM module %s not implemented "
          "(p2s/arbiter test scope only)",
          pic::moduleName(param.module));
}
Scheduler::CPUSidePort::CPUSidePort(const std::string& name, Scheduler *owner) :
    ResponsePort(name, owner),
    owner(owner)
{
}

AddrRangeList
Scheduler::CPUSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
Scheduler::CPUSidePort::recvFunctional(PacketPtr pkt)
{
    owner->handleFunctional(pkt);
}

bool
Scheduler::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

void
Scheduler::CPUSidePort::sendPacket(PacketPtr pkt)
{
    assert(blockedResponse == nullptr);

    if (!sendTimingResp(pkt)) {
        blockedResponse = pkt;
    }
}

void
Scheduler::CPUSidePort::recvRespRetry()
{
    assert(blockedResponse != nullptr);

    PacketPtr pkt = blockedResponse;
    if (sendTimingResp(pkt)) {
        blockedResponse = nullptr;
        owner->scheduleDecodeIfNeeded();
    }
}

void
Scheduler::CPUSidePort::trySendRequestRetry()
{
    if (requestRetryPending && owner->instQueue.size() < owner->maxInstQueueSize) {
        requestRetryPending = false;
        sendRetryReq();
    }
}

Scheduler::MemSidePort::MemSidePort(const std::string& name,
                                    Scheduler *owner,
                                    DownstreamPortID port_id)
    : RequestPort(name, owner),owner(owner),portID(port_id)
{
}

bool
Scheduler::MemSidePort::sendPacket(PacketPtr pkt)
{
    if (blockedRequest != nullptr) {
        return false;
    }

    if (!isConnected()) {
        warn("%s tried to send a downstream PIC packet on an unconnected port\n", name());
        return false;
    }

    if (!sendTimingReq(pkt)) {
        blockedRequest = pkt;
        return false;
    }

    return true;
}

bool
Scheduler::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(portID, pkt);
}

void
Scheduler::MemSidePort::recvReqRetry()
{
    assert(blockedRequest != nullptr);

    PacketPtr pkt = blockedRequest;
    if (sendTimingReq(pkt)) {
        blockedRequest = nullptr;
    }
}

void
Scheduler::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// mod: p2s_arbiter -- the old prepareTask()/triggerTS()/processP2SxEvent()
// machinery here was dead code (never matched scheduler.hh even before
// this pass) and duplicated what Scheduler::dispatchSetParam() now does
// directly and synchronously. Collapsed to just the busy/idle flag.
Scheduler::TaskScheduler::TaskScheduler(Scheduler *owner)
    : owner(owner),
      currState(TaskState::IDLE)
{
}

Scheduler::SwitchController::SwitchController(Scheduler *owner)
    : owner(owner),
      setID(0),
      wayID(0),
      wayStateValid(0),
      flushAddr(0),
      currSwitchType(SwitchType::PIC2Cache),
      switchEvent([this] { processSwitchEvent(); }, "switchEvent"),
      queryEvent([this] { processQueryEvent(); }, "queryEvent"),
      requestFlushEvent([this] { processRequestFlushEvent(); }, "flushEvent"),
      switch2PICEvent([this] { processSwitch2PICEvent(); }, "switch2PICEvent"),
      switch2CacheEvent([this] { processSwitch2CacheEvent(); }, "switch2CacheEvent")
{
}

bool
Scheduler::SwitchController::handleResponse(PacketPtr pkt)
{
    return true;
}

void
Scheduler::SwitchController::processSwitchEvent()
{
}

void
Scheduler::SwitchController::processNextSet()
{
}

void
Scheduler::SwitchController::processQueryEvent()
{
    // mod: p2s_arbiter -- SWITCH/QUERY dispatch to the cache controller is
    // out of scope for the p2s/arbiter test path (no CacheController is
    // even wired up in those configs); left unimplemented like the other
    // SwitchController handlers above rather than resynced.
}

void
Scheduler::SwitchController::processRequestFlushEvent()
{
}

void
Scheduler::SwitchController::processSwitch2PICEvent()
{
}

void
Scheduler::SwitchController::processSwitch2CacheEvent()
{
}

void
Scheduler::startup()
{
    // Publish the MMIO range
    // Decoding itself starts when handleRequest() accepts the first packet.
    sendRangeChange();
    scheduleDecodeIfNeeded();
}

} // namespace gem5
