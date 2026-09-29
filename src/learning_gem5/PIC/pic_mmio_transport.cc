#include "learning_gem5/PIC/pic_mmio_transport.hh"
#include <memory>
#include <utility>
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PicMmioTransport.hh"
#include "mem/request.hh"
#include "mem/packet_access.hh"


namespace gem5
{
namespace pic
{
PicMmioTransport::PicMmioTransport(const PicMmioTransportParams &params)
    : ClockedObject(params),
      mmioPort(params.name + ".mmio_port", this),
      system(params.system),
      requestorId(params.system->getRequestorId(this, "PIC_MMIO_Transport")),
      requestGap(params.request_gap),
      protocolRetryDelay(params.protocol_retry_delay),
      sendEvent([this] { this->processSendEvent(); }, "sendEvent")
{}

// PicMmioTransport::~PicMmioTransport()
// {
//     delete blockedPacket;
//     delete inFlightPacket;
// }

Port &
PicMmioTransport::getPort(const std::string &ifName, PortID idx)
{
    if (ifName == "mmio_port") {
        return mmioPort;
    }
    return ClockedObject::getPort(ifName, idx);
}
PicMmioTransport::MemSidePort::MemSidePort(
    const std::string &name,
    PicMmioTransport *owner) :
    RequestPort(name),
    owner(owner)
{}

bool
PicMmioTransport::MemSidePort::recvTimingResp(PacketPtr pkt) {
    return owner->handleResponse(pkt);
}

void
PicMmioTransport::MemSidePort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr,
             "%s already owns a backpressured packet", name());
    // panic_if(owner->inFlightPacket != nullptr || owner->waitingForResponse,
    //           "%s already has a request in flight", name());

    if (!sendTimingReq(pkt)) {
        blockedPacket = pkt;
        DPRINTF(PicMmioTransport,
                "Request backpressured at addr=%#x; waiting for retry\n",
                pkt->getAddr());
    }

    // owner->inFlightPacket = pkt;
    // owner->waitingForResponse = true;
}
void
PicMmioTransport::MemSidePort::recvReqRetry()
{
    panic_if(blockedPacket == nullptr,
             "%s received recvReqRetry() without a blocked request", name());
    // panic_if(owner->waitingForResponse || owner->inFlightPacket != nullptr,
    //          "%s got request retry while another request is in flight", name());

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    sendPacket(pkt);
}

void
PicMmioTransport::submit(PicSetRequest request, Completion completion)
{
    // TBD: std::move的使用蠻酷的
    pending.push_back({std::move(request), std::move(completion)});
    scheduleSend(Cycles(1));
}

void
PicMmioTransport::scheduleSend(Cycles delay)
{
    // if (!pending.empty() && mmioPort.blockedPacket == nullptr &&
    //     inFlightPacket == nullptr && !waitingForResponse &&
    //     !sendEvent.scheduled()) {
    //     schedule(sendEvent, clockEdge(delay));
    // }

    if (!pending.empty() && mmioPort.blockedPacket == nullptr &&
        !sendEvent.scheduled()) {
        schedule(sendEvent, clockEdge(delay));
    }
    // if (!pending.empty() && mmioPort.blockedPacket == nullptr) schedule(sendEvent, clockEdge(delay));
}

void
PicMmioTransport::processSendEvent()
{
    panic_if(pending.empty(), "%s send event has no pending request", name());
    panic_if(mmioPort.blockedPacket != nullptr, "%s send event fired while another request is active", name());
    // panic_if(inFlightPacket != nullptr || waitingForResponse, "%s send event fired while another request is active", name());

    // DPRINTF(PicMmioTransport, "pending elements: %d", pending.size());
    const PicSetRequest requestInfo = pending.front().request;
    pending.pop_front();
    const uint64_t address = registerAddress(requestInfo.reg);

    RequestPtr request = std::make_shared<Request>(
        address,
        MmioAccessSize,
        Request::Flags(),
        requestorId
    );

    PacketPtr pkt = new Packet(request, MemCmd::WriteReq);
    pkt->allocate();

    pkt->setLE<uint64_t>(requestInfo.value);

    DPRINTF(PicMmioTransport,
            "Sending %-24s addr=%#x value=%#x\n",
            requestInfo.label, address, requestInfo.value);
    
    mmioPort.sendPacket(pkt);
    scheduleSend(Cycles(1));
}

bool
PicMmioTransport::handleResponse(PacketPtr pkt)
{
    // panic_if(!waitingForResponse || inFlightPacket == nullptr ||
    //              pending.empty(),
    //          "%s received an unexpected response", name());
    // panic_if(pkt != inFlightPacket,
    //          "%s did not receive the same Packet pointer it sent", name());
    panic_if(!pkt->isResponse(), "%s received a non-response Packet", name());

    const PicSetRequest requestInfo = pending.front().request;
    const uint64_t responseData = pkt->getLE<uint64_t>();
    const bool rejected = requestInfo.reg == SetRegister::Param &&
                          responseData == RetryResponse;

    delete pkt;
    // inFlightPacket = nullptr;
    // waitingForResponse = false;

    if (rejected) {
        ++protocolRetries;
        DPRINTF(PicMmioTransport,
                "PIC rejected %s with all-ones; scheduling reissue\n",
                requestInfo.label);
        scheduleSend(protocolRetryDelay);
        return true;
    }

    PendingRequest completed = std::move(pending.front());
    pending.pop_front();
    ++completedRequests;

    if (completed.completion) {
        completed.completion({responseData});
    }

    scheduleSend(requestGap);
    return true;
}

// bool
// PicMmioTransport::idle() const
// {
//     return pending.empty() && blockedPacket == nullptr &&
//            inFlightPacket == nullptr && !waitingForResponse &&
//            !sendEvent.scheduled();
// }

// std::size_t
// PicMmioTransport::queuedRequests() const
// {
//     return pending.size();
// }

} // namespace pic
} // namespace gem5
