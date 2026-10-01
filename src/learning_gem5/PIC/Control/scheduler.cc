#include "learning_gem5/PIC/Control/scheduler.hh"
#include "mem/request.hh"
#include <algorithm>
#include <cassert>
#include "debug/Scheduler.hh"
#include "mem/packet_access.hh"

namespace gem5
{
Scheduler::Scheduler(const SchedulerParams &params) :
    ClockedObject(params),
    instPort(params.name + ".inst_port", this),
    cacheControllerPort(params.name + ".cc_port", this, PICPortID::CC),
    loadPort(params.name + "ld_port", this, PICPortID::LD),
    storePort(params.name + "st_port", this, PICPortID::ST),
    p2sLPort(params.name + ".p2sl_port", this, PICPortID::P2SL),
    p2sRPort(params.name + ".p2sr_port", this, PICPortID::P2SR),
    p2sRTPort(params.name + ".p2srt_port", this, PICPortID::P2SRT),
    cacheBankPort(params.name + ".cb_port", this, PICPortID::CB),
    accPort(params.name + ".acc_port", this, PICPortID::ACC),
    switchControllerPort(params.name + ".sc_port", this, PICPortID::SC),
    cmdStateHelperPort(params.name + ".csh_port", this, PICPortID::CSH),
    requestorId(params.system->getRequestorId(this, "Scheduler")),
    currState(TaskState::IDLE),
    decodeEvent([this]{this->processDecodeEvent();}, "decodeEvent"),
    prepareTaskEvent([this]{this->processPrepareTaskEvent();}, "prepareTaskEvent"),
    enqueEvent([this]{this->processEnqueEvent();}, "enqueEvent"),
    setCmdEvent([this]{this->processSetCmdEvent();}, "setCmdEvent"),
    triggerEvent([this]{this->processTriggerEvent();}, "triggerEvent"),
    loadEvent([this]{this->processLoadEvent();}, "LoadEvent"),
    storeEvent([this]{this->processStoreEvent();}, "StoreEvent"),
    p2sLEvent([this]{this->processP2SLEvent();}, "p2sLEvent"),
    p2sREvent([this]{this->processP2SREvent();}, "p2sREvent"),
    p2sRTEvent([this]{this->processP2SRTEvent();}, "p2sRTEvent"),
    calEvent([this]{this->processCalEvent();}, "calEvent"),
    accEvent([this]{this->processAccEvent();}, "accEvent"),
    switchEvent([this]{this->processSwitchEvent();}, "switchEvent")
{}

Port&
Scheduler::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        return instPort;
    }
    else if (if_name == "ld_port") {
        return loadPort;
    }
    else if (if_name == "st_port") {
        return storePort;
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
    else if (if_name == "csh_port") {
        return cmdStateHelperPort;
    }
    return ClockedObject::getPort(if_name, idx);
}
Scheduler::CPUSidePort::CPUSidePort(
    const std::string& name,
    Scheduler *owner) :
    ResponsePort(name),
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
    assert(blockedPacket == nullptr);

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
    PICPortID picPortID) : 
    RequestPort(name),
    owner(owner),
    picPortID(picPortID),
    blockedPacket(nullptr)
{}
bool
Scheduler::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleResponse(this->picPortID, pkt);
}

void
Scheduler::MemSidePort::sendPacket(PacketPtr pkt)
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
    assert(blockedPacket != nullptr);

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

bool
Scheduler::handleRequest(PacketPtr pkt)
{
    if (instQueue.size() >= maxInstQueueSize) {
        // instPort.markRequestRetry();
        return false;
    }

    DPRINTF(Scheduler, "Got request for addr %#llx\n",
            static_cast<unsigned long long>(pkt->getAddr()));

    instQueue.push_back(pkt);

    // TODO prevent from decoding every cycle
    // if (!instQueue.empty() && taskScheduler.idle() &&
    //     !instPort.responseBlocked() && !decodeEvent.scheduled()) { 
    //     schedule(decodeEvent, clockEdge(Cycles(1)));
    // }
    if (!instQueue.empty() && currState == TaskState::IDLE &&
        !decodeEvent.scheduled()) { 
        schedule(decodeEvent, clockEdge(Cycles(1)));
    }
    return true;
}

bool
Scheduler::handleResponse(PICPortID picPortID, PacketPtr pkt)
{
    DPRINTF(Scheduler, "Got downstream response for addr %#llx\n",
            static_cast<unsigned long long>(pkt->getAddr()));

    switch (picPortID) {
        case PICPortID::CC:
            panic_if(!pkt->isResponse(),"Scheduler expected CC completion response");
            DPRINTF(Scheduler, "P2SCC COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::P2SL:
            isEnqCmd = true;
            panic_if(!pkt->isResponse(),"Scheduler expected P2SL completion response");
            DPRINTF(Scheduler, "P2SL COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::P2SR:
            isEnqCmd = true;
            panic_if(!pkt->isResponse(),"Scheduler expected P2SR completion response");
            DPRINTF(Scheduler, "P2SR COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::P2SRT:
            isEnqCmd = true;
            panic_if(!pkt->isResponse(),"Scheduler expected P2SRT completion response");
            DPRINTF(Scheduler, "P2SRT COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::CB:
            panic_if(!pkt->isResponse(),"Scheduler expected CB completion response");
            DPRINTF(Scheduler, "CB COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::ACC:
            isEnqCmd = true;
            panic_if(!pkt->isResponse(),"Scheduler expected CB completion response");
            DPRINTF(Scheduler, "CB COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::SC:
            // TODO: fill the resp from switchController to the registers
            panic_if(!pkt->isResponse(),"Scheduler expected CB completion response");
            DPRINTF(Scheduler, "CB COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        case PICPortID::CSH:
            isQueryCmd = true;
            // TODO: depends on what kind of query it is
            panic_if(!pkt->isResponse(),"Scheduler expected CSH completion response");
            DPRINTF(Scheduler, "CSH COMPLETION RESPONSE port=%u\n",static_cast<unsigned>(picPortID));
            break;
        default:
            DPRINTF(Scheduler, "receive response but not from any known port.\n");
    }
    // only enq cmd has something to do with currState and resp
    if (isEnqCmd) {
        delete pkt;
        currState = TaskState::IDLE;
    }

    return true;
}

void
Scheduler::processDecodeEvent()
{
    // check at the entrance
    if (currState != TaskState::IDLE) return;    // || instQueue.empty() ??

    PacketPtr pkt = instQueue.front();
    instQueue.pop_front();

    const Addr pktAddr = pkt->getAddr();
    const uint64_t MMIOOffset = pktAddr - pic::MmioBase;

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

        // instPort.trySendRequestRetry();
        // scheduleDecodeIfNeeded();
        return;
    }

    const uint64_t dataPayload = pkt->getLE<uint64_t>();

    DPRINTF(Scheduler, "Decoding request addr=%#llx value=%#llx\n",
            static_cast<unsigned long long>(pktAddr),
            static_cast<unsigned long long>(dataPayload));

    switch (MMIOOffset) {
        // SET_SRC
        case static_cast<uint64_t>(pic::SetRegister::Src): {
            src = dataPayload;
            DPRINTF(Scheduler, "SET_SRC to %#x\n", src);
            delete pkt;
            break;
        }
        // SET_DST
        case static_cast<uint64_t>(pic::SetRegister::Dst): {
            dst = dataPayload;
            DPRINTF(Scheduler, "SET_DST to %#x\n", dst);
            delete pkt;
            break;
        }
        // SET_SIZE
        case static_cast<uint64_t>(pic::SetRegister::Size): {
            row = static_cast<uint16_t>(dataPayload & 0x7FF);                      // 11 bit LSB
            byte_per_row = static_cast<uint16_t>((dataPayload >> 11) & 0x7FF);     // 11 bit
            offset = static_cast<uint16_t>((dataPayload >> 22)& 0x3FFF);            // 15 bit
            DPRINTF(Scheduler, "SET_SIZE to (%hu, %hu, %hu)\n", row, byte_per_row, offset);
            delete pkt;
            break;
        }

        // SET_PARAM
        case static_cast<uint64_t>(pic::SetRegister::Param): {
            paramPkt = pkt;
            schedule(prepareTaskEvent, clockEdge(Cycles(1)));
            break;
        }
        default:
            DPRINTF(Scheduler, "Not identified instuction");

    }

    // waiting for instructions, self looping at each cycle
    if (currState == TaskState::IDLE && !instQueue.empty()) {
        schedule(decodeEvent, clockEdge(Cycles(1)));    // temporarily set to 1
    }

}

// prepare task and enqueue the task
// :=set_d_resp
void
Scheduler::processPrepareTaskEvent()
{
    uint64_t dataPayload = paramPkt->getLE<uint64_t>();
    delete paramPkt;                                               // maybe like this?

    // Task nextEnqTask;
    pic::ModuleID moduleID = static_cast<pic::ModuleID>((dataPayload >> 60)& 0xF);                      // moduleID is bit 60 ~ bit 63
    uint8_t cmdID = static_cast<uint8_t>(dataPayload & 0xFF);                            // cmdID is bit 0 ~ bit 7
    DPRINTF(Scheduler, "raw datapayload: %#llx\n", dataPayload);

    switch(moduleID) {
        case pic::ModuleID::LOAD: {
            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(LSPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            LSPayload loadPayload{src, dst, row, byte_per_row, offset};
            pkt->setData(reinterpret_cast<uint8_t*>(&loadPayload));

            nextEnqTask.moduleID = pic::ModuleID::LOAD;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            // nextEnqTask.clientID = 

            DPRINTF(Scheduler, "SET_PARAM LOAD\n");
            break;
        }
        case pic::ModuleID::STORE: {
            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(LSPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            LSPayload storePayload{src, dst, row, byte_per_row, offset};
            pkt->setData(reinterpret_cast<uint8_t*>(&storePayload));

            nextEnqTask.moduleID = pic::ModuleID::STORE;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            // nextEnqTask.clientID = 

            DPRINTF(Scheduler, "SET_PARAM STORE\n");
            break;
        }
        case pic::ModuleID::P2S_L: {
            // decode the params from SET_PARAM
            uint8_t precision = static_cast<uint8_t>(dataPayload & 0x7);    // precision is 3 bit

            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of p2sL
                sizeof(P2S_L_Payload),
                0,                   // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_L_Payload p2s_L_Payload{src, dst, byte_per_row, row, precision};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_L_Payload));

            nextEnqTask.moduleID = pic::ModuleID::P2S_L;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            // nextEnqTask.clientID = 

            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        }
        case pic::ModuleID::P2S_R: {
            uint8_t precision = static_cast<uint8_t>(dataPayload & 0x7);
            uint8_t bufNum = static_cast<uint8_t>((dataPayload >> 3) &0x3);

            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                0,          // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_R_Payload p2s_R_Payload{src, dst, byte_per_row, row, offset, precision, bufNum};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_R_Payload));

            nextEnqTask.moduleID = pic::ModuleID::P2S_R;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            // nextEnqTask.clientID = 

            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        }
        case pic::ModuleID::P2S_R_T: {
            uint8_t precision = static_cast<uint8_t>(dataPayload & 0x7);
            uint8_t bufNum = static_cast<uint8_t>((dataPayload >> 3) &0x3);

            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(P2S_R_Payload),
                    0,                  // TODO
                    requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            P2S_R_Payload p2s_R_T_Payload{src, dst, byte_per_row, row, offset, precision, bufNum};
            pkt->setData(reinterpret_cast<uint8_t*>(&p2s_R_T_Payload));

            nextEnqTask.moduleID = pic::ModuleID::P2S_R_T;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            // nextEnqTask.clientID = 

            DPRINTF(Scheduler, "SET_PARAM P2S\n");
            break;
        }
        case pic::ModuleID::CAL: {
            uint32_t R_Valid_nRols = static_cast<uint32_t>((dataPayload >> 30) &0x3FF);
            uint8_t nBufPerMat = static_cast<uint8_t>((dataPayload >> 28) &0x3);
            uint8_t nCalPerMat = static_cast<uint8_t>((dataPayload >> 26) &0x3);
            uint8_t Base_R_Bit = static_cast<uint8_t>((dataPayload >> 23) &0x7);
            uint8_t L_Precision = static_cast<uint8_t>((dataPayload >> 20) &0x7);
            uint8_t L_Block_Row = static_cast<uint8_t>((dataPayload >> 12) &0xFF);
            bool SignL = static_cast<bool>((dataPayload >> 11) &0x1);
            bool SignR_bitLast = static_cast<bool>((dataPayload >> 10) &0x1);
            bool accWidth = static_cast<bool>((dataPayload >> 9) &0x1);

            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(CalPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            CalPayload calPayload{src, dst, R_Valid_nRols, nBufPerMat, nCalPerMat,
                Base_R_Bit, L_Precision, L_Block_Row, SignL, SignR_bitLast, accWidth};
            pkt->setData(reinterpret_cast<uint8_t*>(&calPayload));

            nextEnqTask.moduleID = pic::ModuleID::CAL;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            nextEnqTask.clientID = QryTabClient::EXE;

            DPRINTF(Scheduler, "SET_PARAM CAL\n");
            break;
        }
        case pic::ModuleID::ACC: {
            uint8_t bitWidth = static_cast<uint8_t>((dataPayload >> 8) &0x7);  // 3 bit
            uint32_t accRowNum = static_cast<uint32_t>((dataPayload >> 11) &0x7FF);      // 11 bit
            uint8_t srcNum = static_cast<uint8_t>((dataPayload >> 22) &0x7);          // 3 bit
            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(AccPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            AccPayload accPayload{src, dst, accRowNum, srcNum, bitWidth};
            pkt->setData(reinterpret_cast<uint8_t*>(&accPayload));

            nextEnqTask.moduleID = pic::ModuleID::ACC;
            nextEnqTask.pkt = pkt;
            nextEnqTask.cmdID = cmdID;
            nextEnqTask.clientID = QryTabClient::ACC;

            DPRINTF(Scheduler, "SET_PARAM ACC\n");
            break;
        }
        case pic::ModuleID::SWITCH: {
            // decode datapayload and set wayID and switch type
            // wayID = static_cast<uint32_t>(dataPayload & 0xFFFFFFFF);
            bool op = dataPayload & 0x1;   // switch type, 0 = ALLOC, 1 = FREE
            uint8_t nLevels = (dataPayload >> 1) & 0xF;

            RequestPtr request = std::make_shared<Request>(
                0,    // the target MMIO address of dpm
                sizeof(SwitchPayload),
                0,                  // TODO
                requestorId
            );

            PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
            pkt->allocate();

            SwitchPayload switchPayload{op, nLevels};
            pkt->setData(reinterpret_cast<uint8_t*>(&switchPayload));

            nextEnqTask.moduleID = pic::ModuleID::SWITCH;
            nextEnqTask.pkt = pkt;

            nextImmTask.push_back(nextEnqTask);
            DPRINTF(Scheduler, "SET_PARAM SWITCH\n");
            break;
        }
        case pic::ModuleID::QUERY: {
            bool immQuery = static_cast<bool>((dataPayload >> 8) &0x1);
            if (immQuery) {
                // TODO go gather the response result from the switch
                // and combine together with query result
            }
            else {
                // request whether cmdID is done by sending the packet immediately
                RequestPtr request = std::make_shared<Request>(
                    0,    // TODO
                    sizeof(QueryPayload),
                    0,                  // TODO
                    requestorId
                );

                PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
                pkt->allocate();
                // check if_finish, actually don't care
                QueryPayload queryPayload{QryTabClient::READER, cmdID, false};
                pkt->setData(reinterpret_cast<uint8_t*>(&queryPayload));
                cmdStateHelperPort.sendPacket(pkt);
                return;         // maybe like this? TBD
            }
            break;
        }
    }
    // for most case, proceed to enqueue state
    // TODO find a way to checkt whether nextEnqTask is valid
    if (nextEnqTask.pkt != nullptr && !enqueEvent.scheduled()) {
        schedule(enqueEvent, clockEdge(Cycles(1)));
    }
}
void
Scheduler::processEnqueEvent() {
    nextTask.push_back(nextEnqTask);
    schedule(setCmdEvent, clockEdge(Cycles(1)));
}
void
Scheduler::processSetCmdEvent(){
    // request whether cmdID is done by sending the packet immediately
    RequestPtr request = std::make_shared<Request>(
        0,    // TODO
        sizeof(QueryPayload),
        0,                  // TODO
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    QueryPayload queryPayload{nextEnqTask.clientID, nextEnqTask.cmdID, false};    // to init the cmd_state
    pkt->setData(reinterpret_cast<uint8_t*>(&queryPayload));
    cmdStateHelperPort.sendPacket(pkt);

    if (currState == TaskState::IDLE && !triggerEvent.scheduled()) {
        schedule(triggerEvent, clockEdge(Cycles(1)));
    }
}

// dequeue from the queue and execute
void
Scheduler::processTriggerEvent()
{
    // TODO
    // schedule the task requests
    // for every cycle/trigger
    // check if there's immTask if yes then check hardware condition(IDLE/BUSY)
    // check if there's task in the queue, if yes then check hardware condition(IDLE/BUSY)

    // need to wait
    if (currState != TaskState::IDLE) {
        return;
    }
    // IDLE right now
    else {
        // check if there's immtask
        if (!nextImmTask.empty()) {
            currState = TaskState::SWITCHING;
            schedule(switchEvent, clockEdge(Cycles(1)));
        }

        else {
            if (!nextTask.empty()) {
                Task t = nextTask.front();
                switch(t.moduleID) {
                    case pic::ModuleID::LOAD:
                        currState = TaskState::LOADING;
                        schedule(loadEvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::STORE:
                        currState = TaskState::STORING;
                        schedule(storeEvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::P2S_L:
                        currState = TaskState::P2SING;
                        schedule(p2sLEvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::P2S_R:
                        currState = TaskState::P2SING;
                        schedule(p2sREvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::P2S_R_T:
                        currState = TaskState::P2SING;
                        schedule(p2sRTEvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::CAL:
                        currState = TaskState::CALING;
                        schedule(calEvent, clockEdge(Cycles(1)));
                        break;
                    case pic::ModuleID::ACC:
                        currState = TaskState::ACCING;
                        schedule(accEvent, clockEdge(Cycles(1)));
                        break;
                }
            }

        }
    }
}
void
Scheduler::processLoadEvent() {
    // send the nextTask packet to DPM
    bool success = loadPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send load request to load ctrl\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "Load Ctrl busy, stalling load request.\n");
    }
}
void
Scheduler::processStoreEvent() {
    // send the nextTask packet to DPM
    bool success = storePort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send store request to store ctrl\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "Store Ctrl busy, stalling store request.\n");
    }
}
void
Scheduler::processP2SLEvent() {
    // send the nextTask packet to DPM
    bool success = p2sLPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send p2sL request to p2sL\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SL busy, stalling p2s request.\n");
    }
}
void
Scheduler::processP2SREvent() {
    // send the nextTask packet to DPM
    bool success = p2sRPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send p2sR request to p2sR\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SR busy, stalling p2s request.\n");
    }
}
void
Scheduler::processP2SRTEvent() {
    // send the nextTask packet to DPM
    bool success = p2sRTPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send p2sRT request to p2sRT\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "P2SRT busy, stalling p2s request.\n");
    }
}
void
Scheduler::processCalEvent() {
    // send the nextTask packet to DPM
    bool success = cacheBankPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send cal request to cache bank\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "CacheBank busy, stalling cal request.\n");
    }
}
void
Scheduler::processAccEvent() {
    // send the nextTask packet to DPM
    bool success = accPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send Acc request to accumulator\n");
        nextTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "Accumulator busy, stalling acc request.\n");
    }
}
void
Scheduler::processSwitchEvent() {
    // send the nextTask packet to DPM
    bool success = switchControllerPort.sendTimingReq(nextTask.front().pkt);
    if (success) {
        DPRINTF(Scheduler, "Send Switch request to switchCtrl\n");
        nextImmTask.pop_front();
    }
    else {
        // need to store and retry
        DPRINTF(Scheduler, "switch busy, stalling switch request.\n");
    }
}
// void
// Scheduler::startup()
// {
//     // Publish the MMIO range
//     // Decoding itself starts when handleRequest() accepts the first packet.
//     sendRangeChange();
//     schedule(decodeEvent, clockEdge(Cycles(1)));
// }

} // namespace gem5
