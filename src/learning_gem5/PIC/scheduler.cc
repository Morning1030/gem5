#include "learning_gem5/PIC/scheduler.hh"
#include "mem/request.hh"
#include <algorithm>
#include <cassert>
#include "debug/Scheduler.hh"
#include "mem/packet_access.hh"


namespace gem5
{
Scheduler::Scheduler(SchedulerParams *params) :
    ClockedObject(params),
    instPort(params->name + ".inst_port", this),
    cacheControllerPort(params->name + ".cc_port", this),
    DPMPort(params->name + ".dpm_port", this),
    cacheBankPort(params->name + ".cb_port", this),
    taskScheduler(this),
    switchController(this),
    decodeEvent([this]{this->processDecodeEvent();}, "decodeEvent"),
    p2sLEvent([this]{this->processP2SLEvent();}, "p2sLEvent"),
    p2sREvent([this]{this->processP2SREvent();}, "p2sREvent"),
    p2sRTEvent([this]{this->processP2SRTEvent();}, "p2sRTEvent")
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

            // SET_DST
            case 0x02:
                dst = dataPayload;
                DPRINTF(Scheduler, "SET_DST to %#x\n", dst);
                break;

            // SET_SIZE
            case 0x04:
                row = dataPayload & 0x7FF;                      // 11 bit LSB
                byte_per_row = (dataPayload >> 11) & 0x7FF;     // 11 bit
                offset = (dataPayload >> 22)& 0x3FFF            // 15 bit
                DPRINTF(Scheduler, "SET_SIZE to (%hu, %hu, %hu)\n", row, byte_per_row, offset);
                break;

            // SET_PARAM
            case 0x06:
                taskScheduler->prepareTask(pkt, src, dst, row, byte_per_row, offset);
                break;
            default:
    }
    // TODO make pkt into response / delete it
    // waiting for instructions, self looping at each cycle
    if (taskScheduler.currState == IDLE && !instQueue.empty()) {
        schedule(decodeEvent, curTick() + cycles(1))    // temporarily set to 1
    }

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

Scheduler::TaskScheduler::TaskScheduler(Scheduler *owner)
    : owner(owner),
      currState(TaskState::IDLE),
      nextImmTask{TaskOp::SWITCH, nullptr},
      p2sEvent([this] { processP2SEvent(); }, "p2sEvent")
{
}


void
Scheduler::TaskScheduler::prepareTask(PacketPtr paramPkt, uint64_t src, uint64_t dst, uint16_t row, uint16_t byte_per_row, uint16_t offset)
{
    uint64_t dataPayload = paramPkt->getLE<uint64_t>();
    Task nextEnqTask;
    FUncID funcID = (dataPayload >> 60)& 0xF;                      // funcID is bit 60 ~ bit 63
    switch(funcID):
        case LOAD:
            nextEnqTask.funcID = LOAD;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM LOAD\n");
        case STORE:
            nextEnqTask.funcID = STORE;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM STORE\n");
        case P2S_L:
            // decode the params from SET_PARAM
            uint8_t precision = dataPayload & 0x8;    // TODO: but precision is 3 bit

            RequestorID requestorId = system.getRequestorId(this, "Scheduler");
            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(P2S_L_Payload),
                0,                   // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            P2S_L_Payload *p2s_L_Payload = new P2S_L_Payload{
                src,
                dst,
                byte_per_row,
                row,
                offset
                precision   // precision is int8
            };
            pkt->dataDynamic(reinterpret_cast<uint8_t*>(p2s_L_Payload));

            nextEnqTask.funcID = P2S_L;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        case P2S_R:
            uint8_t precision = dataPayload & 0x8;    // TODO: but precision is 3 bit
            uint8_t bufNum = (dataPayload >> 4) &0x3;

            RequestorID requestorId = system.getRequestorId(this, "Scheduler");

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                0,          // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            P2S_R_Payload *p2s_R_Payload = new P2S_R_Payload{
                src,
                dst,
                byte_per_row,
                row,
                offset,
                precision,
                bufNum
            };
            pkt->dataDynamic(reinterpret_cast<uint8_t*>(p2s_R_Payload));

            nextEnqTask.funcID = P2S_R;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        case P2S_R_T:
            uint8_t precision = dataPayload & 0x8;    // TODO: but precision is 3 bit
            uint8_t bufNum = (dataPayload >> 4) &0x3;

            RequestorID requestorId = system.getRequestorId(this, "Scheduler");

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                    0,                  // TODO
                    requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            P2S_R_Payload *p2s_R_T_Payload = new P2S_R_Payload{
                src,
                dst,
                byte_per_row,
                row,
                offset,
                precision,
                bufNum
            };
            pkt->dataDynamic(reinterpret_cast<uint8_t*>(p2s_R_T_Payload));

            nextEnqTask.funcID = P2S_R_T;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        case ACC:
            nextEnqTask.funcID = ACC;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM ACC\n");
            break;
        case CAL:
            nextEnqTask.funcID = CAL;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM CAL\n");
            break;
        case SWITCH:

            // decode datapayload and set wayID and switch type
            // wayID = static_cast<uint32_t>(dataPayload & 0xFFFFFFFF);
            bool st = static_cast<uint32_t>(dataPayload & 0x1);   // switch type
            if (!st) switchController->switchType = Cache2PIC;    // ALLOC
            else switchController->switchType = PIC2Cache;        // FREE

            nextEnqTask.funcID = SWITCH;
            nextEnqTask.pkt = pkt;

            taskScheduler->nextImmTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM SWITCH\n");
            break;
    //      case QUERY
    // TODO
    // call triggerTS
    triggerTS();
}
void
Scheduler::TaskScheduler::triggerTS()
{
    // TODO
    // schedule the task requests
    // for every cycle/trigger
    // check if there's immTask if yes then check hardware condition(IDLE/BUSY)
    // check if there's task in the queue, if yes then check hardware condition(IDLE/BUSY)


    // need to wait
    if (currState != IDLE) {
        return;
    }
    // IDLE right now
    else {
        // check if there's immtask
        if (!nextImmTask.empty()) {
            currState = SWITCHING;
            // Task t = nextTask.front();
            schedule(owner->switchController->switchEvent, curTick() + cycles(1));
        }

        else {
            if (!nextTask.empty()) {
                Task t = nextTask.front();
                switch(t.funcID) {
                    case P2S_L:
                        currState = P2SING;
                        schedule(p2sLEvent, curTick() + cycles(1));
                    case P2S_R:
                        currState = P2SING;
                        schedule(p2sREvent, curTick() + cycles(1));
                    case P2S_R_T:
                        currState = P2SING;
                        schedule(p2sRTEvent, curTick() + cycles(1));
                }
            }

        }
    }
}

void
Scheduler::TaskScheduler::processP2SLEvent() {
    // send the nextTask packet to DPM
    bool success = owner->P2SLPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
    }
}
void
Scheduler::TaskScheduler::processP2SREvent() {
    // send the nextTask packet to DPM
    bool success = owner->P2SRPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
    }
}
void
Scheduler::TaskScheduler::processP2SRTEvent() {
    // send the nextTask packet to DPM
    bool success = owner->P2SRTPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
    }
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
    size_t pktSize = std::max(sizeof(QueryPayload), sizeof(RespPayload));
    RequestorID requestorId = system.getRequestorId(this, "Scheduler");

    RequestPtr request = std::make_shared<Request>(
        pioAddr + offset                    // the target MMIO address of cache controller
        pktSize,                            // setID, wayID
        Request::QueryWay,                  // flag
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::QueryReq);
    QueryPayload *queryPayload = new QueryPayload{setID, wayID};
    pkt->dataDynamic(reinterpret_cast<uint8_t*>(queryPayload));

    bool success = owner->cacheControllerPort.sendTimingReq(pkt);
    // if (!success) {
    // owner->retryQueue.push_back(pkt);
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
