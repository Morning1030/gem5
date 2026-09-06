#include "learning_gem5/PIC/ACC/acc_mmio_bridge.hh"

#include <cassert>
#include <cstdint>

#include "base/logging.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{
namespace pic
{

AccMmioBridge::MmioPort::MmioPort(
    const std::string &name,
    AccMmioBridge *owner)
    : ResponsePort(name, owner),
      owner(owner)
{
}

Tick
AccMmioBridge::MmioPort::recvAtomic(PacketPtr pkt)
{
    panic("%s atomic access unsupported", name());
}

void
AccMmioBridge::MmioPort::recvFunctional(PacketPtr pkt)
{
    panic("%s functional access unsupported", name());
}

bool
AccMmioBridge::MmioPort::recvTimingReq(PacketPtr pkt)
{
    return owner->handleMmioRequest(pkt);
}

void
AccMmioBridge::MmioPort::recvRespRetry()
{
    assert(blockedResponse != nullptr);

    PacketPtr pkt = blockedResponse;

    if (sendTimingResp(pkt)) {
        blockedResponse = nullptr;
    }
}

AddrRangeList
AccMmioBridge::MmioPort::getAddrRanges() const
{
    return {
        AddrRange(
            MmioBase,
            MmioBase + MmioWindowSize)
    };
}

void
AccMmioBridge::MmioPort::sendResponse(PacketPtr pkt)
{
    panic_if(
        blockedResponse != nullptr,
        "%s attempted to queue two blocked MMIO responses",
        name());

    if (!sendTimingResp(pkt)) {
        blockedResponse = pkt;
    }
}


AccMmioBridge::AccPort::AccPort(
    const std::string &name,
    AccMmioBridge *owner)
    : RequestPort(name, owner),
      owner(owner)
{
}

bool
AccMmioBridge::AccPort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleAccResponse(pkt);
}

void
AccMmioBridge::AccPort::recvReqRetry()
{
    owner->retryAccRequest();
}

void
AccMmioBridge::AccPort::recvRangeChange()
{
}


AccMmioBridge::AccMmioBridge(
    const AccMmioBridgeParams &params)
    : ClockedObject(params),
      mmioPort(
          name() + ".mmio_port",
          this),
      accPort(
          name() + ".acc_port",
          this),
      requestorId(
          Request::invldRequestorId),
      mmioResponseEvent(
          [this] {
              processMmioResponse();
          },
          name() + ".mmio_response_event")
{
    panic_if(
        params.system == nullptr,
        "%s requires a System",
        name());

    requestorId =
        params.system->getRequestorId(
            this,
            "AccMmioBridge");
}

Port &
AccMmioBridge::getPort(
    const std::string &if_name,
    PortID idx)
{
    if (if_name == "mmio_port") {
        return mmioPort;
    }

    if (if_name == "acc_port") {
        return accPort;
    }

    return ClockedObject::getPort(
        if_name,
        idx);
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


bool
AccMmioBridge::handleMmioRequest(PacketPtr pkt)
{
    panic_if(
        !pkt->isWrite(),
        "%s expected a PIC MMIO WriteReq",
        name());

    panic_if(
        pkt->getSize() != MmioAccessSize,
        "%s expected %u-byte PIC MMIO request, got %u",
        name(),
        MmioAccessSize,
        pkt->getSize());

    panic_if(
        pkt->getAddr() < MmioBase ||
        pkt->getAddr() >= MmioBase + MmioWindowSize,
        "%s received out-of-range PIC MMIO addr=%#llx",
        name(),
        static_cast<unsigned long long>(
            pkt->getAddr()));

    const uint64_t offset =
        pkt->getAddr() - MmioBase;

    const uint64_t value =
        pkt->getLE<uint64_t>();

    switch (offset) {

      case static_cast<uint64_t>(
          SetRegister::Src):

        src = value;
        srcValid = true;

        inform(
            "%s ACC test SET_SRC=%#llx",
            name(),
            static_cast<unsigned long long>(
                src));

        sendMmioSuccess(pkt);
        return true;


      case static_cast<uint64_t>(
          SetRegister::Dst):

        dst = value;
        dstValid = true;

        inform(
            "%s ACC test SET_DST=%#llx",
            name(),
            static_cast<unsigned long long>(
                dst));

        sendMmioSuccess(pkt);
        return true;


      case static_cast<uint64_t>(
          SetRegister::Size):

        sendMmioSuccess(pkt);
        return true;


      case static_cast<uint64_t>(
          SetRegister::Param):
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
            fields.module != ModuleId::Acc,
            "%s test bridge only accepts ModuleId::Acc, got %s",
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
