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
struct P2S_L_Payload
{
    uint64_t base_dramAddr_to_load;
    uint64_t base_picAddr_to_store;
    uint32_t next_row_offset_elem;  // the low 15 bits
    uint8_t _L_block_row;           // 8 bits
    uint8_t precision;              // 3 bits
};
// datapayload for p2s
struct P2S_R_Payload
{
    uint64_t dramAddr;
    uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
    uint32_t next_row_offset_bytes;                                 // 15bits
    uint32_t nRows                          ;                        // Read how many rows
    uint32_t nCols;                                                 // Number of columns to read, max 1024
    uint8_t precision;
    uint8_t bufNum;                                                 // 2 bits
};
// datapayload for exe/cal
struct CAL_Payload
{
    uint64_t _L_vec_fetch_addr;     // SET_SRC
    uint64_t set_up_addr;             // SET_DST
    uint32_t _R_block_row;          // SET_PARAM
    uint8_t nBuf;
    uint8_t nCal;
    uint8_t _R_base_bit;
    uint8_t L_precision;
    uint8_t _L_block_row;
    bool signed_L;
    bool signed_R_last_exist;
    bool accWidth;
};

/*
    uint32_t RValidNRows;
    uint8_t nBufPerMat;
    uint8_t nCalPerMat;
    uint8_t baseRBit;
    uint8_t L_Precision;
    uint8_t L_Block_Row;
    bool signL;
    bool signRBitLast;
    bool accWidth;
    bool needL;
*/
class Scheduler : public ClockedObject
{
  private:
    /*
    val LOAD_ID=0
        val P2SL_ID=1
        val P2SR_ID=2
        val P2SRT_ID=3
        val IM2COL_ID=4
        val ACC_ID=5
        val EXE_ID=6
        val STORE_ID=7
        val SWITCH_ID=8
        val QUERY_ID=9

        val totalModule=10
    */
    enum class FuncID {LOAD, STORE, P2S_L, P2S_R, P2S_R_T, ACC, CAL, SWITCH, QUERY};

    struct Task
    {
        FuncID funcID;
        PacketPtr pkt;
    };
    // mod: p2s_arbiter -- was "PortID", colliding with the global
    // gem5::PortID that Scheduler::getPort()'s own signature needs
    // (unqualified PortID inside this class resolved to this enum
    // instead, breaking the getPort() override below). scheduler.cc
    // already expects this name -- see its DownstreamPortID:: usages --
    // this rename just brings the header in line with it.
    enum class DownstreamPortID{CC, P2SL, P2SR, P2SRT, CB};

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
                std::deque<Task> nextImmTask;   // switch is immTask
                TaskState currState;

                EventFunctionWrapper p2sLEvent;
                EventFunctionWrapper p2sREvent;
                EventFunctionWrapper p2sRTEvent;


                void prepareTask(PacketPtr paramPkt, uint64_t src, uint64_t dst, uint16_t row, uint16_t byte_per_row, uint16_t offset);
        void triggerTS(Task t);
        void processP2SLEvent();
                void processP2SREvent();
                void processP2SRTEvent();

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
        // register file data to store the params
        uint64_t src;                   // SET_SRC
        uint64_t dst;                   // SET_DST
        uint16_t row;                   // SET_SIZE
        uint16_t byte_per_row;          // SET_SIZE
        uint16_t offset;                // SET_SIZE

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
