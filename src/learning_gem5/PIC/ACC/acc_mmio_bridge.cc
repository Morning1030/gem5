#include "learning_gem5/PIC/ACC/acc_mmio_bridge.hh"
#include "learning_gem5/PIC/pic_protocol.hh"

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "debug/AccMmioBridge.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{
namespace pic
{

AccMmioBridge::AccMmioBridge(const AccMmioBridgeParams &params): 
    ClockedObject(params),
    mmioPort(params.name + ".mmio_port", this),
    accPort(params.name + ".acc_port", this),
    cmdStateHelperPort(params.name + ".csh_port", this),
    requestorId(params.system->getRequestorId(this, "AccMmioBridge")),
    mmioResponseEvent([this] {this->processMmioResponse();},".mmio_response_event"),
    decodeEvent([this]{this->processDecodeEvent();}, "decodeEvent"),
    prepareTaskEvent([this]{this->processPrepareTaskEvent();}, "prepareTaskEvent"),
    enqueEvent([this]{this->processEnqueEvent();}, "enqueEvent"),
    setCmdEvent([this]{this->processSetCmdEvent();}, "setCmdEvent")
    {
        panic_if(
            params.system == nullptr,
            "%s requires a System",
            name());
    }

Port &
AccMmioBridge::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mmio_port") {
        return mmioPort;
    }

    else if (if_name == "acc_port") {
        return accPort;
    }

    else if (if_name == "csh_port") {
        return cmdStateHelperPort;
    }
    return ClockedObject::getPort(if_name, idx);
}
AccMmioBridge::CPUSidePort::CPUSidePort(
    const std::string &name,
    AccMmioBridge *owner) :
    ResponsePort(name),
    owner(owner)
{}
bool
AccMmioBridge::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleRequest(pkt);
}

void
AccMmioBridge::CPUSidePort::recvRespRetry()
{
    assert(blockedResponse != nullptr);

    PacketPtr pkt = blockedResponse;

    if (sendTimingResp(pkt)) {
        blockedResponse = nullptr;
    }
}

void
AccMmioBridge::CPUSidePort::sendResponse(PacketPtr pkt)
{
    panic_if(
        blockedResponse != nullptr,
        "%s attempted to queue two blocked MMIO responses",
        name());

    if (!sendTimingResp(pkt)) {
        blockedResponse = pkt;
    }
}


AccMmioBridge::MemSidePort::MemSidePort(
    const std::string &name,
    AccMmioBridge *owner)
    : RequestPort(name),
      owner(owner)
{}

void
AccMmioBridge::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");
    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
    }
}
bool
AccMmioBridge::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleAccResponse(pkt);
}

void
AccMmioBridge::MemSidePort::recvReqRetry()
{
    owner->retryAccRequest();
}

void
AccMmioBridge::sendMmioSuccess(PacketPtr pkt)
{
    panic_if(
        pendingMmioResponsePkt != nullptr,
        "%s already has a pending MMIO response",
        name());

    if (!pkt->isResponse()) {
        pkt->makeResponse();
    }

    pkt->setLE<uint64_t>(0);

    pendingMmioResponsePkt = pkt;

    if (!mmioResponseEvent.scheduled()) {
        schedule(
            mmioResponseEvent,
            clockEdge(Cycles(1)));
    }
}

void
AccMmioBridge::processMmioResponse()
{
    if (pendingMmioResponsePkt == nullptr) {
        return;
    }

    PacketPtr pkt =
        pendingMmioResponsePkt;

    pendingMmioResponsePkt = nullptr;

    mmioPort.sendResponse(pkt);
}

/*
bool
AccMmioBridge::handleRequest(PacketPtr pkt)
{
    panic_if(!pkt->isWrite(), "%s expected a PIC MMIO WriteReq", name());
    panic_if(pkt->getSize() != MmioAccessSize, "%s expected %u-byte PIC MMIO request, got %u", name(), MmioAccessSize, pkt->getSize());
    panic_if(pkt->getAddr() < MmioBase || pkt->getAddr() >= MmioBase + MmioWindowSize, "%s received out-of-range PIC MMIO addr=%#llx", name(), 
        static_cast<unsigned long long>(pkt->getAddr()));

    const uint64_t offset = pkt->getAddr() - MmioBase;
    const uint64_t value = pkt->getLE<uint64_t>();

    switch (offset) {

      case static_cast<uint64_t>(SetRegister::Src):

        src = value;
        srcValid = true;

        inform(
            "%s ACC test SET_SRC=%#llx",
            name(),
            static_cast<unsigned long long>(
                src));

        sendMmioSuccess(pkt);
        return true;

      case static_cast<uint64_t>(SetRegister::Dst):

        dst = value;
        dstValid = true;

        inform(
            "%s ACC test SET_DST=%#llx",
            name(),
            static_cast<unsigned long long>(
                dst));

        sendMmioSuccess(pkt);
        return true;


      case static_cast<uint64_t>(SetRegister::Size):

        sendMmioSuccess(pkt);
        return true;

      case static_cast<uint64_t>(SetRegister::Param):
      {
        panic_if(
            blockedAccPkt != nullptr ||
            pendingMmioParamPkt != nullptr,
            "%s received a second ACC launch while one launch is blocked",
            name());

        panic_if(
            accInFlight,
            "%s received a second ACC command while ACC is still busy",
            name());

        panic_if(
            !srcValid || !dstValid,
            "%s ACC SET_PARAM arrived before SET_SRC/SET_DST",
            name());

        const ParamFields fields =
            unpackParam(value);

        panic_if(
            fields.module != ModuleID::Acc,
            "%s test bridge only accepts ModuleID::Acc, got %s",
            name(),
            moduleName(fields.module));

        launchAcc(
            pkt,
            fields);

        return true;
      }

      default:

        panic(
            "%s received unknown PIC MMIO offset %#llx",
            name(),
            static_cast<unsigned long long>(
                offset));
    }
}
*/
bool
AccMmioBridge::handleRequest(PacketPtr pkt)
{
    if (instQueue.size() >= maxInstQueueSize) {
        // instPort.markRequestRetry();
        return false;
    }

    // DPRINTF(AccMmioBridge, "Got request from addr %#llx\n",
    //        static_cast<unsigned long long>(pkt->getAddr()));

    instQueue.push_back(pkt);

    // if (!instQueue.empty() && taskScheduler.idle() &&
    //     !decodeEvent.scheduled()) { 
    //     schedule(decodeEvent, clockEdge(Cycles(1)));
    // }
    
    if (!instQueue.empty() && !decodeEvent.scheduled()) {
        DPRINTF(AccMmioBridge, "Decode from handle request"); 
        schedule(decodeEvent, clockEdge(Cycles(1)));
    }
    return true;
}

void
AccMmioBridge::processDecodeEvent()
{
    // check at the entrance
    // if (taskScheduler.currState != IDLE) return;    // || instQueue.empty() ??

    PacketPtr pkt = instQueue.front();
    instQueue.pop_front();

    const Addr pktAddr = pkt->getAddr();
    const uint64_t MMIOOffset = pktAddr - pic::MmioBase;

    // The current public protocol is one 64-bit write per SET register.
    // if (!pkt->isWrite() || pkt->getSize() != pic::MmioAccessSize ||
    //     pktAddr < pic::MmioBase ||
    //     pktAddr >= pic::MmioBase + pic::MmioWindowSize) {
    //     warn("%s received malformed PIC MMIO request addr=%#llx size=%u\n",
    //          name(), static_cast<unsigned long long>(pktAddr),
    //          pkt->getSize());

    //     pkt->makeResponse();
    //     pkt->setLE<uint64_t>(pic::RetryResponse);
    //     instPort.sendPacket(pkt);

    //     instPort.trySendRequestRetry();
    //     scheduleDecodeIfNeeded();
    //     return;
    // }

    const uint64_t dataPayload = pkt->getLE<uint64_t>();

    DPRINTF(AccMmioBridge, "Decoding request addr=%#llx value=%#llx\n",
            static_cast<unsigned long long>(pktAddr),
            static_cast<unsigned long long>(dataPayload));

    switch (MMIOOffset) {
            // SET_SRC
            case static_cast<uint64_t>(pic::SetRegister::Src):
                src = dataPayload;
                DPRINTF(AccMmioBridge, "SET_SRC to %#x\n", src);
                delete pkt;
                break;
            // SET_DST
            case static_cast<uint64_t>(pic::SetRegister::Dst):
                dst = dataPayload;
                DPRINTF(AccMmioBridge, "SET_DST to %#x\n", dst);
                delete pkt;
                break;

            // SET_SIZE
            case static_cast<uint64_t>(pic::SetRegister::Size):
                row = static_cast<uint16_t>(dataPayload & 0x7FF);                      // 11 bit LSB
                byte_per_row = static_cast<uint16_t>((dataPayload >> 11) & 0x7FF);     // 11 bit
                offset = static_cast<uint16_t>((dataPayload >> 22)& 0x3FFF);            // 15 bit
                DPRINTF(AccMmioBridge, "SET_SIZE to (%hu, %hu, %hu)\n", row, byte_per_row, offset);
                delete pkt;
                break;

            // SET_PARAM
            case static_cast<uint64_t>(pic::SetRegister::Param):
                paramPkt = pkt;
                DPRINTF(AccMmioBridge, "SET_PARAM from decoding\n");
                schedule(prepareTaskEvent, clockEdge(Cycles(1)));
                // taskScheduler->prepareTask(pkt, src, dst, row, byte_per_row, offset);
                break;
            default:
                DPRINTF(AccMmioBridge, "Not identified instuction");
    }

    // waiting for instructions, self looping at each cycle
    // if (taskScheduler.currState == IDLE && !instQueue.empty()) {
    //     schedule(decodeEvent, clockEdge(Cycles(1)))    // temporarily set to 1
    // }
    if (!instQueue.empty()) {
        DPRINTF(AccMmioBridge, "Decode self looping\n");
        schedule(decodeEvent, clockEdge(Cycles(1)));    // temporarily set to 1
    }

}
void
AccMmioBridge::processPrepareTaskEvent()
{
    uint64_t dataPayload = paramPkt->getLE<uint64_t>();
    DPRINTF(AccMmioBridge, "dataPayload from SET_PARAM: %llx", dataPayload);
    delete paramPkt;                                               // maybe like this?


    // Task nextEnqTask;
    ModuleID moduleID = static_cast<pic::ModuleID>((dataPayload >> 60)& 0xF);                      // moduleID is bit 60 ~ bit 63
    uint8_t cmdID = static_cast<uint8_t>(dataPayload & 0xFF);                            // cmdID is bit 0 ~ bit 7

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

            DPRINTF(AccMmioBridge, "SET_PARAM LOAD\n");
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

            DPRINTF(AccMmioBridge, "SET_PARAM STORE\n");
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

            DPRINTF(AccMmioBridge, "SET_PARAM P2S\n");
            break;
        }
        case pic::ModuleID::P2S_R: {
            uint8_t precision = static_cast<uint8_t>(dataPayload & 0x7);    // TODO: but precision is 3 bit
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

            DPRINTF(AccMmioBridge, "SET_PARAM P2S\n");
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

            DPRINTF(AccMmioBridge, "SET_PARAM P2S\n");
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

            DPRINTF(AccMmioBridge, "SET_PARAM CAL\n");
            break;
        }
        case pic::ModuleID::ACC: {
            uint8_t bitWidth = static_cast<uint8_t>((dataPayload >> 8) &0x7);  // 3 bit
            uint32_t accRowNum = static_cast<uint32_t>((dataPayload >> 11) &0x7FF);      // 11 bit
            uint8_t srcNum = static_cast<uint8_t>((dataPayload >> 22) &0x7);          // 4 bit
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

            DPRINTF(AccMmioBridge, "prepare Task ACC\n");
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

            // taskScheduler->nextImmTask.push_back(nextEnqTask);
            DPRINTF(AccMmioBridge, "SET_PARAM SWITCH\n");
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
                // cmdStateHelperPort.sendPacket(pkt);
                return;         // maybe like this? TBD
            }
            break;
        }
    }
    // for most case, proceed to enqueue state
    // if (nextEnqTask != NULL && !enqueEvent.scheduled()) {
    //     schedule(enqueEvent, clockEdge(Cycles(4)));
    // }
    if (!enqueEvent.scheduled()) {
        schedule(enqueEvent, clockEdge(Cycles(4)));
    }
}
void
AccMmioBridge::processEnqueEvent() {
    nextTask.push_back(nextEnqTask);
    DPRINTF(AccMmioBridge, "Successfully enqueue nextTask to queue\n");
    // if () {
    //     schedule(setCmdEvent, clockEdge(Cycles(1)));
    // }
    schedule(setCmdEvent, clockEdge(Cycles(5)));
}
void
AccMmioBridge::processSetCmdEvent(){
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
    DPRINTF(AccMmioBridge, "Successfully send cmd state query to cmdStateHelper");
}


void
AccMmioBridge::launchAcc(
    PacketPtr mmioPkt,
    ParamFields fields)
{
    const uint8_t sourceCount =
        static_cast<uint8_t>(
            fields.others &
            mask(4));

    const uint16_t rowCount =
        static_cast<uint16_t>(
            (fields.others >> 4) &
            mask(11));

    const uint8_t bitWidthCode =
        static_cast<uint8_t>(
            (fields.others >> 15) &
            mask(3));

    panic_if(
        rowCount == 0,
        "%s ACC rowCount must be non-zero",
        name());

    panic_if(
        bitWidthCode > 1,
        "%s ACC bitWidthCode=%u is invalid; "
        "official ACC uses 0=16b, 1=32b",
        name(),
        static_cast<unsigned>(
            bitWidthCode));


    RequestPtr request =
        std::make_shared<Request>(
            0,
            sizeof(AccRequestPayload),
            Request::Flags(),
            requestorId);

    PacketPtr accPkt =
        Packet::createWrite(request);

    accPkt->allocate();


    const AccRequestPayload payload{
        src,
        dst,
        rowCount,
        sourceCount,
        static_cast<uint8_t>(
            bitWidthCode == 1 ? 1 : 0),
        {0, 0}
    };

    accPkt->setData(
        reinterpret_cast<const uint8_t *>(
            &payload));


    inform(
        "%s launching ACC through real FunctionBuilder/MMIO path: "
        "src=%#llx dst=%#llx sources=%u rows=%u width=%s",
        name(),
        static_cast<unsigned long long>(
            src),
        static_cast<unsigned long long>(
            dst),
        static_cast<unsigned>(
            sourceCount),
        static_cast<unsigned>(
            rowCount),
        bitWidthCode ? "32" : "16");

    pendingMmioParamPkt =
        mmioPkt;


    if (!accPort.sendTimingReq(accPkt)) {
        blockedAccPkt =
            accPkt;

        return;
    }


    accInFlight = true;

    accRequestAccepted();
}


void
AccMmioBridge::accRequestAccepted()
{
    panic_if(
        pendingMmioParamPkt == nullptr,
        "%s accepted ACC request without pending SET_PARAM",
        name());


    PacketPtr mmioPkt =
        pendingMmioParamPkt;

    pendingMmioParamPkt =
        nullptr;

    sendMmioSuccess(mmioPkt);
}


void
AccMmioBridge::retryAccRequest()
{
    panic_if(
        blockedAccPkt == nullptr,
        "%s received ACC retry without blocked ACC request",
        name());


    PacketPtr pkt =
        blockedAccPkt;


    if (!accPort.sendTimingReq(pkt)) {
        return;
    }


    blockedAccPkt =
        nullptr;

    accInFlight = true;

    accRequestAccepted();
}


bool
AccMmioBridge::handleAccResponse(PacketPtr pkt)
{
    panic_if(
        !accInFlight,
        "%s received ACC completion without an in-flight ACC",
        name());

    panic_if(
        !pkt->isResponse(),
        "%s expected ACC completion response",
        name());


    delete pkt;

    accInFlight = false;


    inform(
        "%s ACC CONTROL COMPLETION observed",
        name());


    exitSimLoop(
        name() +
        " ACC E2E FULL PASS");

    return true;
}

} // namespace pic
} // namespace gem5
