#include "learning_gem5/PIC/scheduler.hh"
#include "mem/request.hh"
#include <algorithm>
#include <cassert>
#include "debug/Scheduler.hh"
#include "mem/packet_access.hh"

#define MMIOBase 0x10028000ULL
namespace gem5
{
Scheduler::Scheduler(const SchedulerParams *params) :
    ClockedObject(params),
    instPort(params->name + ".inst_port", this),
    cacheControllerPort(params->name + ".cc_port", this),
    p2sLPort(params->name + ".p2sl_port", this),
    p2sRPort(params->name + ".p2sr_port", this),
    p2sRTPort(params->name + ".p2srt_port", this),
    cacheBankPort(params->name + ".cb_port", this),
    accPort(params->name + ".acc_port", this),
    switchControllerPort(params->name + ".sc_port", this),
    taskScheduler(this),
    requestorId(system.getRequestorId(this, "Scheduler")),
    decodeEvent([this]{this->processDecodeEvent();}, "decodeEvent"),
{}

Port&
Scheduler::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    else if (if_name == "cc_port") {
        return cacheControllerPort;
    }
    else if (if_name == "p2sl_port") {
        return p2sLPort;
    }
    else if (if_name == "p2sr_port") {
        return p2sRPort;
    }
    else if (if_name == "p2srt_port") {
        return p2sRTPort;
    }
    else if (if_name == "cb_port") {
        return cacheBankPort;
    }
    else if (if_name == "acc_port") {
        return accPort;
    }
    else if (if_name == "sc_port") {
        return switchControllerPort;
    }

    return ClockedObject::getPort(if_name, idx);
}
Scheduler::CPUSidePort::CPUSidePort(
    const std::string& name,
    Scheduler *owner) :
    ResponsePort(name, owner),
    owner(owner),
    blockedPacket(nullptr)
{}
bool
Scheduler::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}
void
Scheduler::CPUSidePort::sendPacket(PacketPtr pkt)
{
    // send p2s done to scheduler
    assert(blockedResponse == nullptr);

    if (sendTimingResp(pkt)) {
        blockedPacket = nullptr;
        DPRINTF(Scheduler, "Scheduler function complete: send back to CPU\n");
    }
    else blockedPacket = pkt;
}
void
Scheduler::CPUSidePort::recvRespRetry()
{
    // retry to send resp to scheduler
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
}
Scheduler::MemSidePort::MemSidePort(
    const std::string& name,
    Scheduler *owner,
    PICPortID port_id) : 
    RequestPort(name, owner),
    owner(owner),
    portID(picPortID),
    blockedPacket(nullptr)
{}
bool
Scheduler::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(this->portID, pkt);
}

void
Schdduler::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}

void
Scheduler::MemSidePort::recvReqRetry()
{
    // TODO retry to send req to downstream modules
    assert(blockedRequest != nullptr);

    PacketPtr pkt = blockedRequest;
    if (sendTimingReq(pkt)) {
        blockedRequest = nullptr;
    }
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

    // TODO prevent from decoding every cycle
    if (!instQueue.empty() && taskScheduler.idle() &&
        !instPort.responseBlocked() && !decodeEvent.scheduled()) { 
        schedule(decodeEvent, clockEdge(Cycles(1)));
    }
    return true;
}

bool
Scheduler::handleResponse(PICPortID portID, PacketPtr pkt)
{
    DPRINTF(Scheduler, "Got downstream response for addr %#llx\n",
            static_cast<unsigned long long>(pkt->getAddr()));

    switch (portID) {
        case PICPortID::CC:
            panic_if(!pkt->isResponse(),"Scheduler expected CC completion response");
            DPRINTFS(Scheduler, this, "P2SCC COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(portID));
            break;
        case PICPortID::P2SL:
            panic_if(!pkt->isResponse(),"Scheduler expected P2SL completion response");
            DPRINTFS(Scheduler, this, "P2SL COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(portID));
            break;
        case PICPortID::P2SR:
            panic_if(!pkt->isResponse(),"Scheduler expected P2SR completion response");
            DPRINTFS(Scheduler, this, "P2SR COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(portID));
            break;
        case PICPortID::P2SRT:
            panic_if(!pkt->isResponse(),"Scheduler expected P2SRT completion response");
            DPRINTFS(Scheduler, this, "P2SRT COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(portID));
            break;
        case PICPortID::CB:
            panic_if(!pkt->isResponse(),"Scheduler expected CB completion response");
            DPRINTFS(Scheduler, this, "CB COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(portID));
            break;
        default:
            DPRINTF(Scheduler, this, "receive response but not from any known port.\n");
    }
    delete pkt;
    currState = TaskState::IDLE;
    return true;
}

void
Scheduler::processDecodeEvent()
{
    // check at the entrance
    if (taskScheduler.currState != IDLE) return;    // || instQueue.empty() ??

    PacketPtr pkt = instQueue.front();
    instQueue.pop_front();

    const Addr pktAddr = pkt->getAddr();
    const uint64_t offset = pktAddr - pic::MMIOBase;

    // The current public protocol is one 64-bit write per SET register.
    if (!pkt->isWrite() || pkt->getSize() != pic::MmioAccessSize ||
        pktAddr < MMIOBase ||
        pktAddr >= MMIOBase + pic::MmioWindowSize) {
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
            // SET_SRC
            case 0x00:
                src = dataPayload;
                DPRINTF(Scheduler, "SET_SRC to %#x\n", src);
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
        schedule(decodeEvent, clockEdge(Cycles(1)))    // temporarily set to 1
    }

}

Scheduler::TaskScheduler::TaskScheduler(Scheduler *owner)
    : owner(owner),
    currState(TaskState::IDLE),
    loadEvent([this]{this->processLoadEvent();}, "LoadEvent"),
    storeEvent([this]{this->processStoreEvent();}, "StoreEvent"),
    p2sLEvent([this]{this->processP2SLEvent();}, "p2sLEvent"),
    p2sREvent([this]{this->processP2SREvent();}, "p2sREvent"),
    p2sRTEvent([this]{this->processP2SRTEvent();}, "p2sRTEvent"),
    calEvent([this]{this->processCalEvent();}, "calEvent"),
    accEvent([this]{this->processAccEvent();}, "accEvent"),
    switchEvent([this]{this->processSwitchEvent();}, "switchEvent")
{}

void
Scheduler::TaskScheduler::prepareTask(PacketPtr paramPkt, uint64_t src, uint64_t dst, uint16_t row, uint16_t byte_per_row, uint16_t offset)
{
    uint64_t dataPayload = paramPkt->getLE<uint64_t>();
    Task nextEnqTask;
    FUncID funcID = (dataPayload >> 60)& 0xF;                      // funcID is bit 60 ~ bit 63
    switch(funcID):
        case LOAD:
            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(LoadPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            LoadPayload loadPayload{src, dst, row, byte_per_row, offset};
            pkt->setData(reinterpret_cast<uint8_t*>(&loadPayload));

            nextEnqTask.funcID = LOAD;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM LOAD\n");

        case STORE:
            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(StorePayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            StorePayload storePayload{src, dst, row, byte_per_row, offset};
            pkt->setData(reinterpret_cast<uint8_t*>(&storePayload));

            nextEnqTask.funcID = STORE;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM STORE\n");

        case P2S_L:
            // decode the params from SET_PARAM
            uint8_t precision = dataPayload & 0x7;    // precision is 3 bit

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of p2sL
                sizeof(P2S_L_Payload),
                0,                   // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_L_Payload p2s_L_Payload{src, dst, byte_per_row, row, offset, precision};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_L_Payload));

            nextEnqTask.funcID = P2S_L;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;

        case P2S_R:
            uint8_t precision = dataPayload & 0x7;    // TODO: but precision is 3 bit
            uint8_t bufNum = (dataPayload >> 3) &0x3;

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                0,          // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_R_Payload p2s_R_Payload{src, dst, byte_per_row, row, offset, precision, bufNum};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_R_Payload));

            nextEnqTask.funcID = P2S_R;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;

        case P2S_R_T:
            uint8_t precision = dataPayload & 0x7;
            uint8_t bufNum = (dataPayload >> 3) &0x3;

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                    0,                  // TODO
                    requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_R_Payload p2s_R_T_Payload{src, dst, byte_per_row, row, offset, precision, bufNum};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_R_T_Payload));

            nextEnqTask.funcID = P2S_R_T;
            nextEnqTask.pkt = pkt;
            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;

        case CAL:
            uint32_t R_Valid_nRols = (dataPayload >> 30) &0x3FF;
            uint8_t nBufPerMat = (dataPayload >> 28) &0x3;
            uint8_t nCalPerMat = (dataPayload >> 26) &0x3;
            uint8_t Base_R_Bit = (dataPayload >> 23) &0x7;
            uint8_t L_Precision = (dataPayload >> 20) &0x7;
            uint8_t L_Block_Row = (dataPayload >> 12) &0xFF;
            bool SignL = (dataPayload >> 11) &0x1;
            bool SignR_bitLast = (dataPayload >> 10) &0x1;
            bool accWidth = (dataPayload >> 9) &0x1;

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(CalPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            CalPayload calPayload{src, dst, R_Valid_nRols, nBufPerMat, nCalPerMat,
                Base_R_Bit, L_Precision, L_Block_Row, SignL, SignR_bitLast, accWidth};
            pkt->setData(reinterpret_cast<uint8_t*>(&calPayload));

            nextEnqTask.funcID = CAL;
            nextEnqTask.pkt = pkt;

            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM CAL\n");
            break;

        case ACC:
            uint8_t bitWidth = (dataPayload >> 8) &0x7;  // 3 bit
            uint8_t accRowNum = (dataPayload >> 11) &0x7FF;      // 11 bit
            uint32_t srcNum = (dataPayload >> 22) &0x7;          // 4 bit
            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(AccPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            AccPayload accPayload{src, dst, accRowNum, srcNum, bitWidth};
            pkt->setData(reinterpret_cast<uint8_t*>(&accPayload));

            nextEnqTask.funcID = ACC;
            nextEnqTask.pkt = pkt;

            taskScheduler->nextTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM ACC\n");
            break;
        case SWITCH:
            // decode datapayload and set wayID and switch type
            // wayID = static_cast<uint32_t>(dataPayload & 0xFFFFFFFF);
            bool op = dataPayload & 0x1;   // switch type, 0 = ALLOC, 1 = FREE
            uint8_t nLevels = (dataPayload >> 1) & 0xF;

            RequestPtr request = std::make_shared<Request>(
                pioAddr + offset,    // the target MMIO address of dpm
                sizeof(SwitchPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            SwitchPayload switchPayload{op, nLevels};
            pkt->setData(reinterpret_cast<uint8_t*>(&switchPayload));

            nextEnqTask.funcID = SWITCH;
            nextEnqTask.pkt = pkt;

            taskScheduler->nextImmTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM SWITCH\n");
            break;
    //      case QUERY
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
            schedule(switchEvent, clockEdge(Cycles(1)));
        }

        else {
            if (!nextTask.empty()) {
                Task t = nextTask.front();
                switch(t.funcID) {
                    case LOAD:
                        currState = LOADING;
                        schedule(loadEvent, clockEdge(Cycles(1)));
                    case STORE:
                        currState = STORING;
                        schedule(storeEvent, clockEdge(Cycles(1)));
                    case P2S_L:
                        currState = P2SING;
                        schedule(p2sLEvent, clockEdge(Cycles(1)));
                    case P2S_R:
                        currState = P2SING;
                        schedule(p2sREvent, clockEdge(Cycles(1)));
                    case P2S_R_T:
                        currState = P2SING;
                        schedule(p2sRTEvent, clockEdge(Cycles(1)));
                    case CAL:
                        currState = CALING;
                        schedule(calEvent, clockEdge(Cycles(1)));
                    case ACC:
                        currState = ACCING;
                        schedule(accEvent, clockEdge(Cycles(1)));
                    case SWITCH:
                        currState = SWITCHING;
                        schedule(switchEvent, clockEdge(Cycles(1)));
                }
            }

        }
    }
}

void
Scheduler::TaskScheduler::processP2SLEvent() {
    // send the nextTask packet to DPM
    bool success = owner->p2sLPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SL busy, stalling p2s request.\n");
    }
}
void
Scheduler::TaskScheduler::processP2SREvent() {
    // send the nextTask packet to DPM
    bool success = owner->p2sRPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SR busy, stalling p2s request.\n");
    }
}
void
Scheduler::TaskScheduler::processP2SRTEvent() {
    // send the nextTask packet to DPM
    bool success = owner->p2sRTPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SRT busy, stalling p2s request.\n");
    }
}
void
Scheduler::TaskScheduler::processCalEvent() {
    // send the nextTask packet to DPM
    bool success = owner->cacheBankPortPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "CacheBank busy, stalling p2s request.\n");
    }
}
void
Scheduler::TaskScheduler::processAccEvent() {
    // send the nextTask packet to DPM
    bool success = owner->accPort.sendTimingReq(nextTask.front());
    if (success) {
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "Acc busy, stalling p2s request.\n");
    }
}

void
Scheduler::startup()
{
    // Publish the MMIO range
    // Decoding itself starts when handleRequest() accepts the first packet.
    sendRangeChange();
    schedule(decodeEvent, clockEdge(Cycles(1)));
}

} // namespace gem5
