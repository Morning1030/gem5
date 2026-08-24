#include "learning_gem5/PIC/scheduler.hh"
#include "mem/request.hh"
#include <algorithm>
#include <cassert>
#include "debug/Scheduler.hh"
#include "mem/packet_access.hh"


namespace gem5
{
Scheduler::Scheduler(const SchedulerParams &params)
    : ClockedObject(params),
      instPort(name() + ".inst_port", this),
      cacheControllerPort(name() + ".mem_side_cc", this, DownstreamPortID::CC),
      p2sLPort(name() + ".mem_side_p2sl", this, DownstreamPortID::P2SL),
      p2sRPort(name() + ".mem_side_p2sr",this,DownstreamPortID::P2SR),
      p2sRTPort(name() + ".mem_side_p2srt",this,DownstreamPortID::P2SRT),
      cacheBankPort(name() + ".mem_side_cb",this,DownstreamPortID::CB),
      taskScheduler(this),
      switchController(this),
      decodeEvent([this] { processDecodeEvent(); }, name() + ".decodeEvent")
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
    const uint64_t offset = pktAddr - pic::MmioBase;

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
    uint64_t responseData = 0;

    DPRINTF(Scheduler, "Decoding request addr=%#llx value=%#llx\n",
            static_cast<unsigned long long>(pktAddr),
            static_cast<unsigned long long>(dataPayload));

    switch (offset) {
      case static_cast<uint64_t>(pic::SetRegister::Src):
        src = dataPayload;
        DPRINTF(Scheduler, "SET_SRC = %#llx\n", static_cast<unsigned long long>(src));
        break;

      case static_cast<uint64_t>(pic::SetRegister::Dst):
        dst = dataPayload;
        DPRINTF(Scheduler, "SET_DST = %#llx\n", static_cast<unsigned long long>(dst));
        break;

      case static_cast<uint64_t>(pic::SetRegister::Size): {
        size = dataPayload;
        const pic::SizeFields f = pic::unpackSize(size);
        DPRINTF(Scheduler,
                "SET_SIZE row=%u bytesPerRow=%u rowOffset=%u raw=%#llx\n",
                static_cast<unsigned>(f.row),
                static_cast<unsigned>(f.bytesPerRow),
                static_cast<unsigned>(f.rowOffset),
                static_cast<unsigned long long>(size));
        break;
      }

      case static_cast<uint64_t>(pic::SetRegister::Param): {
        param = dataPayload;
        const pic::ParamFields f = pic::unpackParam(param);

        DPRINTF(Scheduler,
                "SET_PARAM module=%s cmdID=%u others=%#llx raw=%#llx\n",
                pic::moduleName(f.module),
                static_cast<unsigned>(f.commandId),
                static_cast<unsigned long long>(f.others),
                static_cast<unsigned long long>(param));

        taskScheduler.prepareTask(pkt, src, dst, size);

        if (f.module == pic::ModuleId::Query) {
            const bool busy = !taskScheduler.idle();
            responseData = pic::packQueryResponse(
                {true, busy, true, 0, 0});
        }
        break;
      }

      default:
        warn("%s received unknown PIC MMIO offset %#llx\n",
            name(), static_cast<unsigned long long>(offset));
        responseData = pic::RetryResponse;
        break;
    }

    pkt->makeResponse();
    pkt->setLE<uint64_t>(responseData);
    instPort.sendPacket(pkt);

    instPort.trySendRequestRetry();

    scheduleDecodeIfNeeded();
}

Scheduler::CPUSidePort::CPUSidePort(const std::string& name,Scheduler *owner)
    : ResponsePort(name, owner), owner(owner)
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

Scheduler::TaskScheduler::TaskScheduler(Scheduler *owner)
    : owner(owner),
      currState(TaskState::IDLE),
      nextImmTask{TaskOp::SWITCH, nullptr},
      p2sEvent([this] { processP2SEvent(); }, "p2sEvent")
{
}

void
Scheduler::TaskScheduler::prepareTask(
    PacketPtr paramPkt, Addr currentSrc, Addr currentDst,
    uint64_t currentSize)
{
    const uint64_t dataPayload = paramPkt->getLE<uint64_t>();
    const pic::ParamFields fields = pic::unpackParam(dataPayload);
    const pic::SizeFields sizeFields = pic::unpackSize(currentSize);

    DPRINTFS(Scheduler, owner,
            "prepareTask: module=%s cmdID=%u src=%#llx dst=%#llx "
            "row=%u bytesPerRow=%u rowOffset=%u\n",
            pic::moduleName(fields.module),
            static_cast<unsigned>(fields.commandId),
            static_cast<unsigned long long>(currentSrc),
            static_cast<unsigned long long>(currentDst),
            static_cast<unsigned>(sizeFields.row),
            static_cast<unsigned>(sizeFields.bytesPerRow),
            static_cast<unsigned>(sizeFields.rowOffset));

    switch (fields.module) {
      case pic::ModuleId::P2SL: {
        P2S_L_Payload payload{
            currentSrc,
            currentDst,
            static_cast<uint32_t>(sizeFields.bytesPerRow),
            static_cast<uint8_t>(sizeFields.row),
            static_cast<uint8_t>(fields.others & 0x7)
        };

        RequestPtr request = std::make_shared<Request>(
            0,
            sizeof(P2S_L_Payload),
            Request::Flags(),
            Request::invldRequestorId);

        PacketPtr p2sPkt = Packet::createWrite(request);
        p2sPkt->allocate();
        p2sPkt->setData(reinterpret_cast<const uint8_t *>(&payload));

        Task t{TaskOp::P2S, p2sPkt};
        nextTask.push_back(t);

        DPRINTFS(
            Scheduler, owner,
            "SET_PARAM P2S_L packed: dram=%#llx pic=%#llx "
            "rows=%u stride=%u precision=%u\n",
            static_cast<unsigned long long>(payload.base_dramAddr_to_load),
            static_cast<unsigned long long>(payload.base_picAddr_to_store),
            static_cast<unsigned>(payload._L_block_row),
            static_cast<unsigned>(payload.next_row_offset_elem),
            static_cast<unsigned>(payload.precision));

        triggerTS(t);
        break;
      }
      case pic::ModuleId::P2SR: {
        const bool ifT = ((fields.others >> 5) & 0x1) != 0;

        const P2S_R_Payload payload{
            static_cast<uint64_t>(currentSrc),
            static_cast<uint64_t>(currentDst),
            static_cast<uint32_t>(sizeFields.rowOffset),
            static_cast<uint32_t>(sizeFields.row),
            static_cast<uint32_t>(sizeFields.bytesPerRow),
            static_cast<uint8_t>(
                (fields.others >> 2) & 0x7),
            static_cast<uint8_t>(
                fields.others & 0x3)
        };

        RequestPtr request =
            std::make_shared<Request>(
                0,
                sizeof(P2S_R_Payload),
                Request::Flags(),
                Request::invldRequestorId);

        PacketPtr p2sPkt = Packet::createWrite(request);

        p2sPkt->allocate();

        p2sPkt->setData(
            reinterpret_cast<const uint8_t *>(&payload));

        Task t{
            TaskOp::P2S,
            p2sPkt,
            ifT ? P2SRoute::RT : P2SRoute::R
        };

        nextTask.push_back(t);

        DPRINTFS(
            Scheduler,
            owner,
            "P2SR PACK: "
            "dram=%#llx baseArray=%#llx "
            "rows=%u cols=%u stride=%u "
            "precision=%u bufNum=%u if_T=%u route=%s\n",
            static_cast<unsigned long long>(
                payload.dramAddr),
            static_cast<unsigned long long>(
                payload.base_arrayID_to_store),
            static_cast<unsigned>(
                payload.nRows),
            static_cast<unsigned>(
                payload.nCols),
            static_cast<unsigned>(
                payload.next_row_offset_bytes),
            static_cast<unsigned>(
                payload.precision),
            static_cast<unsigned>(
                payload.bufNum),
            static_cast<unsigned>(ifT),
            ifT ? "RT" : "R");

        triggerTS(t);
        break;
      }

      case pic::ModuleId::P2SRTInternal: {
        DPRINTFS(
            Scheduler,
            owner,
            "P2SRT_INTERNAL is not a public command; "
            "use P2SR if_T=1\n");
        break;
      }



      case pic::ModuleId::Exe: {
        Task t{TaskOp::CAL, nullptr};
        nextTask.push_back(t);
        DPRINTFS(Scheduler, owner, "SET_PARAM CAL/EXE queued\n");
        triggerTS(t);
        break;
      }

      case pic::ModuleId::Acc: {
        Task t{TaskOp::ACC, nullptr};
        nextTask.push_back(t);
        DPRINTFS(Scheduler, owner, "SET_PARAM ACC queued\n");
        triggerTS(t);
        break;
      }

      case pic::ModuleId::Switch: {
        Task t{TaskOp::SWITCH, nullptr};
        nextImmTask = t;
        DPRINTFS(Scheduler, owner,
                "SET_PARAM SWITCH stored as immediate task; "
                "downstream switch packet remains un-packed\n");
        triggerTS(t);
        break;
      }

      case pic::ModuleId::Load:
      case pic::ModuleId::Im2Col:
      case pic::ModuleId::Store:
      case pic::ModuleId::Query:

      default:
        warn("Unknown PIC module ID %u in Scheduler::prepareTask\n",
             static_cast<unsigned>(fields.module));
        break;
    }
}

void
Scheduler::TaskScheduler::triggerTS(Task t)
{
    if (currState != TaskState::IDLE) {
        return;
    }

    if (t.taskOp == TaskOp::SWITCH) {
        DPRINTFS(
            Scheduler,
            owner,
            "SWITCH task waiting for Scheduler<->CacheController "
            "packet/parameter contract\n");
        return;
    }

    if (t.taskOp == TaskOp::P2S) {
        if (t.pkt == nullptr) {
            DPRINTFS(
                Scheduler,
                owner,
                "P2S task has no packed packet\n");
            return;
        }

        Scheduler::MemSidePort *port = nullptr;
        const char *route = "UNKNOWN";

        switch (t.p2sRoute) {
          case P2SRoute::L:
            port = &owner->p2sLPort;
            route = "L";
            break;

          case P2SRoute::R:
            port = &owner->p2sRPort;
            route = "R";
            break;

          case P2SRoute::RT:
            port = &owner->p2sRTPort;
            route = "RT";
            break;
        }

        if (port == nullptr || !port->isConnected()) {
            DPRINTFS(
                Scheduler,
                owner,
                "P2S route=%s waiting for connected port\n",
                route);
            return;
        }

        DPRINTFS(
            Scheduler,
            owner,
            "P2S FSM IDLE -> P2SING route=%s\n",
            route);

        currState = TaskState::P2SING;

        owner->schedule(
            p2sEvent,
            owner->clockEdge(Cycles(1)));

        return;
    }
}

void
Scheduler::TaskScheduler::processP2SEvent()
{
    if (nextTask.empty()) {
        currState = TaskState::IDLE;
        return;
    }

    Task &t = nextTask.front();

    if (t.taskOp != TaskOp::P2S ||
        t.pkt == nullptr) {

        currState = TaskState::IDLE;
        return;
    }

    Scheduler::MemSidePort *port = nullptr;
    const char *route = "UNKNOWN";

    switch (t.p2sRoute) {
      case P2SRoute::L:
        port = &owner->p2sLPort;
        route = "L";
        break;

      case P2SRoute::R:
        port = &owner->p2sRPort;
        route = "R";
        break;

      case P2SRoute::RT:
        port = &owner->p2sRTPort;
        route = "RT";
        break;
    }

    panic_if(
        port == nullptr,
        "Scheduler P2S task has invalid route");

    DPRINTFS(
        Scheduler,
        owner,
        "P2S DISPATCH route=%s payloadSize=%u\n",
        route,
        t.pkt->getSize());

    if (port->sendPacket(t.pkt)) {
        DPRINTFS(
            Scheduler,
            owner,
            "P2S DISPATCH ACCEPTED route=%s\n",
            route);

        nextTask.pop_front();

        // Stay P2SING.
        // Actual R / RT completion is Stage 4D, not hidden here.
    }
    else {
        DPRINTFS(
            Scheduler,
            owner,
            "P2S DISPATCH DEFERRED route=%s\n",
            route);
    }
}

void
Scheduler::TaskScheduler::completeCurrentTask()
{
    panic_if(
        currState != TaskState::P2SING,
        "Scheduler received P2S completion outside P2SING");

    DPRINTFS(
        Scheduler,
        owner,
        "P2S FSM P2SING -> IDLE: downstream completion\n");

    currState = TaskState::IDLE;
    owner->scheduleDecodeIfNeeded();
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
