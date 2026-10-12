#ifndef __MEM_CACHE_PIC_ARB_TEST_CLIENT_HH__
#define __MEM_CACHE_PIC_ARB_TEST_CLIENT_HH__

#include <cstdint>
#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/ArbTestClient.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Test client for AccessBankArb. Runs a script, one op at a time, waiting
 * for each response before sending the next:
 *   W <addr> <data>   write (kind 'p2s': 16-byte P2SWritePayload)
 *   R <addr> <expect> read  (kind 'autoload': 8-byte read); the data in
 *                     the response must equal <expect>, else panic.
 *                     <expect> = '*' only prints the value.
 * <addr> is the 17-bit array address the arbiter decodes.
 */
class ArbTestClient : public ClockedObject
{
  public:
    typedef ArbTestClientParams Params;

  private:
    class ClientPort : public RequestPort
    {
      private:
        ArbTestClient *owner;

      public:
        ClientPort(const std::string &name, ArbTestClient *owner) :
            RequestPort(name), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleResp(pkt); }
        void recvReqRetry() override { owner->retry(); }
        void recvRangeChange() override { }
    };

    struct Op
    {
        bool write;
        uint64_t addr;
        uint64_t data;
        bool anyValue = false;   //!< read: print the value, do not check
    };

  public:
    explicit ArbTestClient(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void startup() override;

  private:
    void sendNext();
    bool handleResp(PacketPtr pkt);
    void retry();

    ClientPort port;
    EventFunctionWrapper sendEvent;
    RequestorID requestorId;

    const bool isP2S;
    const bool exitWhenDone;
    const Cycles startDelay;

    std::vector<Op> ops;
    size_t next = 0;
    unsigned readsChecked = 0;
    PacketPtr held = nullptr;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_ARB_TEST_CLIENT_HH__
