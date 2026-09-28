#ifndef __LEARNING_GEM5_AUTOLOAD_L_HH__
#define __LEARNING_GEM5_AUTOLOAD_L_HH__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mem/port.hh"
#include "params/AutoLoadL.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

struct AutoLoadLPayload
{
    uint64_t source;  // request: L row index in the cache bank
    uint64_t data;    // response: the 64-bit L row
};

/**
 * Cache-wide L-vector fetcher, shared by every PIC Mat (one port each).
 *
 * The arbitration is a round-robin arbiter, not a queue: a refused Mat
 * keeps its own request (like request_vec.valid held in send_L_req); this
 * object only remembers WHICH Mats are waiting, and when idle grants the
 * next one after the last-granted pointer (sendRetryReq -> the Mat
 * resends -> accepted). Only one fetch is in flight at a time:
 *   Idle -> ReadMem -> RevVec -> WriteL -> ReportFinish -> Idle
 */
class AutoLoadL : public ClockedObject
{
  private:
    enum class State { Idle, ReadMem, RevVec, WriteL, ReportFinish };

    class CPUSidePort : public ResponsePort
    {
      private:
        AutoLoadL *owner;
        PortID idx;

      protected:
        Tick recvAtomic(PacketPtr) override
        { panic("recvAtomic unimplemented."); }
        void recvFunctional(PacketPtr) override
        { panic("recvFunctional unimplemented."); }
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handleRequest(pkt, idx); }
        void recvRespRetry() override { owner->handleRespRetry(); }
        AddrRangeList getAddrRanges() const override { return {}; }

      public:
        CPUSidePort(const std::string &name, AutoLoadL *owner, PortID idx)
            : ResponsePort(name, owner, idx), owner(owner), idx(idx) {}
    };

    class MemSidePort : public RequestPort
    {
      private:
        AutoLoadL *owner;

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleResponse(pkt); }
        void recvReqRetry() override { owner->handleReqRetry(); }

      public:
        MemSidePort(const std::string &name, AutoLoadL *owner)
            : RequestPort(name, owner), owner(owner) {}
    };

    std::vector<std::unique_ptr<CPUSidePort>> instPorts;
    MemSidePort cacheBankPort;
    RequestorID requestorId;

    State state = State::Idle;
    std::vector<bool> pending;       // Mats refused, waiting for a grant
    PortID reservedPort = InvalidPortID;  // granted, has not resent yet
    PortID lastGranted;              // round-robin pointer
    PortID curPort = InvalidPortID;  // Mat being served
    PacketPtr curPkt = nullptr;      // its request
    PacketPtr blockedReadReq = nullptr;
    PacketPtr blockedResp = nullptr;
    uint64_t dataReadFromBank = 0;

    EventFunctionWrapper arbitrateEvent;
    EventFunctionWrapper readMemEvent;
    EventFunctionWrapper writeLEvent;
    EventFunctionWrapper reportEvent;

    bool handleRequest(PacketPtr pkt, PortID idx);
    bool handleResponse(PacketPtr pkt);
    void handleReqRetry();
    void handleRespRetry();

    void arbitrate();
    void processReadMem();
    void processWriteL();
    void processReport();
    void finishFetch();

  public:
    AutoLoadL(const AutoLoadLParams &params);
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_AUTOLOAD_L_HH__
