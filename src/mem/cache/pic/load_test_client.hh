#ifndef __MEM_CACHE_PIC_LOAD_TEST_CLIENT_HH__
#define __MEM_CACHE_PIC_LOAD_TEST_CLIENT_HH__

#include <vector>

#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/LoadTestClient.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Test stand-in for the Scheduler's ld_port (the RTL CMD_resolver's
 * load_P_req). Sends each LOAD in `loads` to Load_ctl.cmd_side as an
 * LSPayload, waits for Load_ctl's response (the load is done, RTL
 * busy_load falling), then sends the next one.
 */
class LoadTestClient : public ClockedObject
{
  public:
    typedef LoadTestClientParams Params;

  private:
    class ClientPort : public RequestPort
    {
      private:
        LoadTestClient *owner;

      public:
        ClientPort(const std::string &name, LoadTestClient *owner) :
            RequestPort(name), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleResp(pkt); }
        void recvReqRetry() override { owner->retry(); }
        void recvRangeChange() override { }
    };

    struct Load
    {
        uint64_t src;
        uint64_t dst;
        uint16_t row;
        uint16_t bytePerRow;
        uint16_t offset;
    };

  public:
    explicit LoadTestClient(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void startup() override;

  private:
    void sendNext();
    void retry();
    bool handleResp(PacketPtr pkt);

    ClientPort port;
    EventFunctionWrapper sendEvent;
    RequestorID requestorId;
    const bool exitWhenDone;
    const Cycles startDelay;

    std::vector<Load> loads;
    size_t next = 0;
    Tick sentAt = 0;
    PacketPtr held = nullptr;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_LOAD_TEST_CLIENT_HH__
