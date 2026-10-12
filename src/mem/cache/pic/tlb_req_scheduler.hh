#ifndef __MEM_CACHE_PIC_TLB_REQ_SCHEDULER_HH__
#define __MEM_CACHE_PIC_TLB_REQ_SCHEDULER_HH__

#include <vector>

#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/TLBReqScheduler.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Port of TLB_req_schedular. Takes vaddr queries from the read and write
 * DMA page-split modules, serves one at a time (round-robin), translates,
 * and returns the paddr to the side that asked.
 *
 * cpu_side[0/1]: ReadReq, addr = vaddr, size 8. Response carries paddr (LE).
 *
 * Translation is a plain function call (translate()), not a port to a PTW:
 * there is no tile to ask in gem5. The RTL paddr_valid/paddr_received
 * handshake becomes the response being accepted by the requester.
 */
class TLBReqScheduler : public ClockedObject
{
  public:
    typedef TLBReqSchedulerParams Params;

  private:
    class CPUSidePort : public ResponsePort
    {
      private:
        int id;
        TLBReqScheduler *owner;

      public:
        PacketPtr blockedResp = nullptr;  //!< response refused by requester
        bool needRetry = false;           //!< gem5 port protocol: owe a retry

        CPUSidePort(const std::string &name, int id,
                    TLBReqScheduler *owner) :
            ResponsePort(name), id(id), owner(owner)
        { }

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("TLBReqScheduler: recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override
        { panic("TLBReqScheduler: recvFunctional unimpl."); }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    enum class State { Idle, Translating, ReturnPaddr };

  public:
    explicit TLBReqScheduler(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    bool handleRequest(PacketPtr pkt, int side);

    /** One FSM step. */
    void step();
    void arbitrate();
    /** VA -> PA. Identity for now; panics if it cannot translate. */
    Addr translate(Addr vaddr);
    void sendResp();
    /** Response accepted: go idle and send owed retries. */
    void finishQuery();
    void respRetry(int side);

    void scheduleStep();

    std::vector<CPUSidePort> cpuPorts;
    EventFunctionWrapper stepEvent;

    const bool virtualization;
    const Cycles translateLatency;
    const Cycles stateLatency;

    State state = State::Idle;
    PacketPtr pending[2] = {nullptr, nullptr};  //!< per-side waiting query
    PacketPtr active = nullptr;                 //!< query being served
    int activeSide = -1;
    int lastGrant = 1;                          //!< side 0 wins first
    Addr paddr = 0;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_TLB_REQ_SCHEDULER_HH__
