#ifndef __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__
#define __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__

#include <cstdint>
#include <string>

#include "learning_gem5/PIC/ACC/accumulator.hh"
#include "learning_gem5/PIC/pic_protocol.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/AccMmioBridge.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{
namespace pic
{

class AccMmioBridge : public ClockedObject
{
  private:
    class MmioPort : public ResponsePort
    {
      private:
        AccMmioBridge *owner;
        PacketPtr blockedResponse = nullptr;

      public:
        MmioPort(const std::string &name, AccMmioBridge *owner);

        void sendResponse(PacketPtr pkt);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;
    };

    class AccPort : public RequestPort
    {
      private:
        AccMmioBridge *owner;

      public:
        AccPort(const std::string &name, AccMmioBridge *owner);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    MmioPort mmioPort;
    AccPort accPort;

    RequestorID requestorId;

    uint64_t src = 0;
    uint64_t dst = 0;

    bool srcValid = false;
    bool dstValid = false;

    PacketPtr blockedAccPkt = nullptr;
    PacketPtr pendingMmioParamPkt = nullptr;

    bool accInFlight = false;

    PacketPtr pendingMmioResponsePkt = nullptr;

    EventFunctionWrapper mmioResponseEvent;

    bool handleMmioRequest(PacketPtr pkt);
    bool handleAccResponse(PacketPtr pkt);

    void retryAccRequest();

    void sendMmioSuccess(PacketPtr pkt);
    void processMmioResponse();

    void launchAcc(PacketPtr mmioPkt, ParamFields fields);
    void accRequestAccepted();

  public:
    AccMmioBridge(const AccMmioBridgeParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__
