#ifndef __LEARNING_GEM5_PIC_PIC_MMIO_TRANSPORT_HH__
#define __LEARNING_GEM5_PIC_PIC_MMIO_TRANSPORT_HH__

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "learning_gem5/PIC/pic_protocol.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/PicMmioTransport.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/system.hh"

namespace gem5
{
namespace pic
{
using Completion = std::function<void(const PicSetResponse &)>;

class PicMmioTransport : public ClockedObject
{
  private:
    class MemSidePort : public RequestPort
    {
      private:
        PicMmioTransport *owner;

      public:
        MemSidePort(const std::string &name, PicMmioTransport *owner);
        PacketPtr blockedPacket = nullptr;
        void sendPacket(PacketPtr);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override {};
    };

    struct PendingRequest
    {
        PicSetRequest request;
        Completion completion;
    };

    MemSidePort mmioPort;
    System *const system;
    RequestorID requestorId;
    const Cycles requestGap;
    const Cycles protocolRetryDelay;
    EventFunctionWrapper sendEvent;

    std::deque<PendingRequest> pending;

    void scheduleSend(Cycles delay);
    void processSendEvent();
    bool trySend(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);

  public:
    PicMmioTransport(const PicMmioTransportParams &params);
    // ~PicMmioTransport() override;

    Port &getPort(const std::string &ifName,
                  PortID idx = InvalidPortID) override;

    void submit(PicSetRequest request, Completion completion);

    bool idle() const;
    std::size_t queuedRequests() const;
    PacketPtr inFlightPacket = nullptr;
    bool waitingForResponse = false;
    uint64_t completedRequests = 0;
    uint64_t protocolRetries = 0;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_PIC_MMIO_TRANSPORT_HH__
