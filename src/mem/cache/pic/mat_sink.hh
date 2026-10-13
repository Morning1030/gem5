#ifndef __MEM_CACHE_PIC_MAT_SINK_HH__
#define __MEM_CACHE_PIC_MAT_SINK_HH__

#include <deque>
#include <unordered_map>
#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/MatSink.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Test stand-in for the Mats. Takes 8-byte reads and writes on any port,
 * keeps the data per (port, address), and answers after `latency` cycles.
 * A read of an address that was never written returns read_default.
 *
 * With refuse_first set, each port turns away its first request and sends
 * a retry retry_delay cycles later, so the sender must hold and resend.
 */
class MatSink : public ClockedObject
{
  public:
    typedef MatSinkParams Params;

  private:
    class ArrPort : public ResponsePort
    {
      private:
        int id;
        MatSink *owner;

      public:
        ArrPort(const std::string &name, int id, MatSink *owner) :
            ResponsePort(name), id(id), owner(owner)
        { }

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("MatSink: recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override
        { panic("MatSink: recvFunctional unimpl."); }
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handleRequest(pkt, id); }
        void recvRespRetry() override { owner->respRetry(); }
    };

    struct Entry
    {
        PacketPtr pkt = nullptr;
        int port = 0;
        Tick when = 0;
    };

  public:
    explicit MatSink(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    bool handleRequest(PacketPtr pkt, int port);
    void sendReady();
    void respRetry();
    void sendRetries();

    std::vector<ArrPort> ports;
    EventFunctionWrapper respEvent;
    const Cycles latency;
    const bool refuseFirst;
    const uint64_t readDefault;
    const Cycles retryDelay;

    EventFunctionWrapper retryEvent;
    std::vector<bool> refusedOnce;
    std::vector<bool> owesRetry;

    std::deque<Entry> queue;
    Entry blocked;
    bool waitingRetry = false;

    /** (port << 32 | addr) -> last written value. */
    std::unordered_map<uint64_t, uint64_t> mem;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_MAT_SINK_HH__
