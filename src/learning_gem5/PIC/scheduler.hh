#ifndef __LEARNING_GEM5_SCHEDULER_HH__
#define __LEARNING_GEM5_SCHEDULER_HH__

#include <cstdint>
#include <deque>
#include <string>

#include "learning_gem5/PIC/pic_protocol.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/Scheduler.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

// datapayload for switch
struct QueryPayload
{
    uint32_t setID;
    uint32_t wayID;
};

struct RespPayload
{
    uint64_t addr;
    uint32_t state;
};

// datapayload for p2s

struct P2S_R_Payload
{
    uint64_t dramAddr;
    uint64_t base_arrayID_to_store;
    uint32_t next_row_offset_bytes;
    uint32_t nRows;
    uint32_t nCols;
    uint8_t precision;
    uint8_t bufNum;
};

struct P2S_L_Payload
{
    uint64_t base_dramAddr_to_load;
    uint64_t base_picAddr_to_store;
    uint32_t next_row_offset_elem;
    uint8_t _L_block_row;
    uint8_t precision;
};

class Scheduler : public ClockedObject
{
  private:
    enum class TaskOp {P2S, CAL, ACC, SWITCH};
    enum class P2SRoute{L,R,RT};

    struct Task
    {
        TaskOp taskOp;
        PacketPtr pkt;
        P2SRoute p2sRoute = P2SRoute::L;
    };
    enum class DownstreamPortID{CC,P2SL,P2SR,P2SRT,CB};

    // to interact with MMIO request
    class CPUSidePort : public ResponsePort
    {
      private:
        Scheduler *owner;
        PacketPtr blockedResponse = nullptr;

        bool requestRetryPending = false;

      public:
        CPUSidePort(const std::string& name, Scheduler *owner);
        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;

        bool responseBlocked() const { return blockedResponse != nullptr; }
        void markRequestRetry() { requestRetryPending = true; }
        void trySendRequestRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        {
            panic("Scheduler::CPUSidePort recvAtomic unimplemented.");
        }
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    // to interact with Cache controller / DPM / cache bank
    class MemSidePort : public RequestPort
    {
      private:
        Scheduler *owner;
        DownstreamPortID portID;
        PacketPtr blockedRequest = nullptr;

      public:
        MemSidePort(const std::string& name, Scheduler *owner,DownstreamPortID port_id);
        bool sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    class TaskScheduler
    {
      private:
        enum class TaskState
        {
            IDLE,
            SWITCHING,
            P2SING,
            CALING,
            ACCING
        };

        Scheduler *owner;
        std::deque<Task> nextTask;
        TaskState currState;
        Task nextImmTask;
        EventFunctionWrapper p2sEvent;

        void triggerTS(Task t);
        void processP2SEvent();

      public:
        TaskScheduler(Scheduler *owner);

        void prepareTask(PacketPtr paramPkt, Addr currentSrc, Addr currentDst, uint64_t currentSize);

        bool idle() const { return currState == TaskState::IDLE; }

        // use when a real downstream reports completion
        void completeCurrentTask();
    };

    class SwitchController
    {
      private:
        enum class SwitchType {PIC2Cache, Cache2PIC};

        Scheduler *owner;
        uint32_t setID;
        uint32_t wayID;
        uint32_t wayStateValid;
        Addr flushAddr;
        SwitchType currSwitchType;

        EventFunctionWrapper switchEvent;
        EventFunctionWrapper queryEvent;
        EventFunctionWrapper requestFlushEvent;
        EventFunctionWrapper switch2PICEvent;
        EventFunctionWrapper switch2CacheEvent;

        void processSwitchEvent();
        void processQueryEvent();
        void processRequestFlushEvent();
        void processSwitch2PICEvent();
        void processSwitch2CacheEvent();
        void processNextSet();

      public:
        SwitchController(Scheduler *owner);
        bool handleResponse(PacketPtr pkt);
    };

    CPUSidePort instPort;
    MemSidePort cacheControllerPort; // cache controller direct port
    MemSidePort p2sLPort;          // direct P2S_L command port
    MemSidePort p2sRPort;          // direct P2S_R command port
    MemSidePort p2sRTPort;         // direct P2S_R_T command port
    MemSidePort cacheBankPort;       // cache bank direct port
    TaskScheduler taskScheduler;
    SwitchController switchController;

    std::deque<PacketPtr> instQueue;

    uint64_t src = 0;   // SET_SRC
    uint64_t dst = 0;   // SET_DST
    uint64_t size = 0;  // SET_SIZE
    uint64_t param = 0; // SET_PARAM

    size_t maxInstQueueSize = 1000; // temporarily set to 1000
    EventFunctionWrapper decodeEvent;

    void processDecodeEvent();
    void scheduleDecodeIfNeeded();

  public:
    Scheduler(const SchedulerParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    AddrRangeList getAddrRanges() const;
    void sendRangeChange();
    void handleFunctional(PacketPtr pkt);
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(DownstreamPortID portID, PacketPtr pkt);

    void startup() override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_SCHEDULER_HH__
