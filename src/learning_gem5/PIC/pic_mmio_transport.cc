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

PicMmioTransport::CommandPort::CommandPort(
    const std::string &name, PicMmioTransport &owner)
    : RequestPort(name, &owner), owner(owner)
{
}

bool
PicMmioTransport::CommandPort::recvTimingResp(PacketPtr pkt)
{
    return owner.handleTimingResp(pkt);
}

void
PicMmioTransport::CommandPort::recvReqRetry()
{
    owner.handleReqRetry();
}

void
PicMmioTransport::CommandPort::recvRangeChange()
{
}

PicMmioTransport::PicMmioTransport(const PicMmioTransportParams &params)
    : ClockedObject(params),
      commandPort(name() + ".command_port", *this),
      system(params.system),
      requestorId(Request::invldRequestorId),
      requestGap(params.request_gap),
      protocolRetryDelay(params.protocol_retry_delay),
      sendEvent([this] { sendNext(); }, name() + ".send_event")
{
    panic_if(system == nullptr, "%s requires a System", name());
    requestorId = system->getRequestorId(this);
}

PicMmioTransport::~PicMmioTransport()
{
    delete blockedPacket;
    delete inFlightPacket;
}

Port &
PicMmioTransport::getPort(const std::string &ifName, PortID idx)
{
    if (ifName == "command_port") {
        return commandPort;
    }
    return ClockedObject::getPort(ifName, idx);
}

void
PicMmioTransport::submit(PicSetRequest request, Completion completion)
{
    pending.push_back({std::move(request), std::move(completion)});
    scheduleSend(requestGap);
}

bool
PicMmioTransport::idle() const
{
    return pending.empty() && blockedPacket == nullptr &&
           inFlightPacket == nullptr && !waitingForResponse &&
           !sendEvent.scheduled();
}

std::size_t
PicMmioTransport::queuedRequests() const
{
    return pending.size();
}

void
PicMmioTransport::scheduleSend(Cycles delay)
{
    if (!pending.empty() && blockedPacket == nullptr &&
        inFlightPacket == nullptr && !waitingForResponse &&
        !sendEvent.scheduled()) {
        schedule(sendEvent, clockEdge(delay));
    }
}

bool
PicMmioTransport::trySend(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr,
             "%s already owns a backpressured packet", name());
    panic_if(inFlightPacket != nullptr || waitingForResponse,
             "%s already has a request in flight", name());

    if (!commandPort.sendTimingReq(pkt)) {
        blockedPacket = pkt;
        DPRINTF(PicMmioTransport,
                "Request backpressured at addr=%#x; waiting for retry\n",
                pkt->getAddr());
        return false;
    }

    inFlightPacket = pkt;
    waitingForResponse = true;
    return true;
}

void
PicMmioTransport::sendNext()
{
    panic_if(pending.empty(), "%s send event has no pending request", name());
    panic_if(blockedPacket != nullptr || inFlightPacket != nullptr ||
                 waitingForResponse,
             "%s send event fired while another request is active", name());

    const PicSetRequest &requestInfo = pending.front().request;
    const uint64_t address = registerAddress(requestInfo.reg);
    RequestPtr request = std::make_shared<Request>(
        address, MmioAccessSize, Request::Flags(), requestorId);
    PacketPtr pkt = Packet::createWrite(request);
    pkt->allocate();
    pkt->setLE<uint64_t>(requestInfo.value);

    DPRINTF(PicMmioTransport,
            "Sending %-24s addr=%#x value=%#x\n",
            requestInfo.label, address, requestInfo.value);
    trySend(pkt);
}

bool
PicMmioTransport::handleTimingResp(PacketPtr pkt)
{
    panic_if(!waitingForResponse || inFlightPacket == nullptr ||
                 pending.empty(),
             "%s received an unexpected response", name());
    panic_if(pkt != inFlightPacket,
             "%s did not receive the same Packet pointer it sent", name());
    panic_if(!pkt->isResponse(), "%s received a non-response Packet", name());

    const PicSetRequest &requestInfo = pending.front().request;
    const uint64_t responseData = pkt->getLE<uint64_t>();
    const bool rejected = requestInfo.reg == SetRegister::Param &&
                          responseData == RetryResponse;

    delete pkt;
    inFlightPacket = nullptr;
    waitingForResponse = false;

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

void
PicMmioTransport::handleReqRetry()
{
    panic_if(blockedPacket == nullptr,
             "%s received recvReqRetry() without a blocked request", name());
    panic_if(waitingForResponse || inFlightPacket != nullptr,
             "%s got request retry while another request is in flight", name());

    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;
    trySend(pkt);
}

} // namespace pic
} // namespace gem5
