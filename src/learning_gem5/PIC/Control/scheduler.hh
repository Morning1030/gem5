#ifndef __LEARNING_GEM5_SCHEDULER_HH__
#define __LEARNING_GEM5_SCHEDULER_HH__

#include <cstdint>
#include <deque>
#include <string>

#include "learning_gem5/PIC/Control/cmd_state_helper.hh"
#include "learning_gem5/PIC/pic_protocol.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/Scheduler.hh"
#include "sim/clocked_object.hh"
#include "sim/system.hh"

namespace gem5
{

struct LSPayload
{   uint64_t src;
    uint64_t dst;
    uint16_t row;                   
    uint16_t byte_per_row;
    uint16_t offset;
};

struct P2S_L_Payload
{
    uint64_t base_dramAddr_to_load;
    uint64_t base_picAddr_to_store;
    uint16_t next_row_offset_elem;  // the low 15 bits
    uint16_t _L_block_row;           // 8 bits
    uint8_t precision;              // 3 bits
};

struct P2S_R_Payload
{
    uint64_t dramAddr;
    uint64_t base_arrayID_to_store; // Which subarray to put the first selected bit map
    uint16_t next_row_offset_bytes;                                 // 15bits
    uint16_t nRows                          ;                        // Read how many rows
    uint16_t nCols;                                                 // Number of columns to read, max 1024
    uint8_t precision;
    uint8_t bufNum;                                                 // 2 bits
};

struct CalPayload
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

struct AccPayload
{
    uint64_t base_src_picAddr;
    uint64_t dest_picAddr;
    uint32_t row_num;
    uint8_t src_arrayNum;
    uint8_t bitWidth;
};

struct SwitchPayload
{
  bool opType;
  uint8_t nLevels;
};

struct QueryPayload
{
  // added clientID to identify which client does the query come from to avoid redundant ports
  QryTabClient clientID;  
  uint8_t cmdID;
  bool is_finish;
};

class Scheduler : public ClockedObject
{
  public:
    enum class PICPortID{CC, LD, ST, P2SL, P2SR, P2SRT, CB, ACC, SC, CSH};
    enum class TaskState
    {
      IDLE,
      LOADING,
      STORING,
      P2SING,
      CALING,
      ACCING,
      SWITCHING
    };

  private:

    struct Task
    {
        QryTabClient clientID;
        uint8_t cmdID;
        pic::ModuleID moduleID;
        PacketPtr pkt;
    };
    

    // to interact with MMIO request
    class CPUSidePort : public ResponsePort
    {
      public:
        CPUSidePort(const std::string& name, Scheduler *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
        void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override {return {AddrRange(pic::MmioBase, pic::MmioBase + pic::MmioWindowSize)};}

      private:
        Scheduler *owner;
        PacketPtr blockedPacket;
    };

    // to interact with Cache controller / DPM / cache bank
    class MemSidePort : public RequestPort
    {
      private:
        Scheduler *owner;
        PICPortID picPortID;
        PacketPtr blockedPacket;

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
      
      public:
        MemSidePort(const std::string& name, Scheduler *owner, PICPortID picPortID);
        void sendPacket(PacketPtr pkt);
    };


    CPUSidePort instPort;
    MemSidePort cacheControllerPort; // cache controller direct port
    MemSidePort loadPort;          // direct load ctrl port
    MemSidePort storePort;         // direct store ctrl port
    MemSidePort p2sLPort;          // direct P2S_L command port
    MemSidePort p2sRPort;          // direct P2S_R command port
    MemSidePort p2sRTPort;         // direct P2S_R_T command port
    MemSidePort cacheBankPort;       // cache bank direct port
    MemSidePort accPort;
    MemSidePort switchControllerPort; // switch controller direct port
    MemSidePort cmdStateHelperPort;

    RequestorID requestorId;

    std::deque<PacketPtr> instQueue;
    std::deque<Task> nextTask;
    std::deque<Task> nextImmTask;   // switch is immTask

    // register file data to store the params
    uint64_t src;                   // SET_SRC
    uint64_t dst;                   // SET_DST
    uint16_t row;                   // SET_SIZE
    uint16_t byte_per_row;          // SET_SIZE
    uint16_t offset;                // SET_SIZE

    PacketPtr paramPkt;
    Task nextEnqTask;
    bool isImmeCmd, isEnqCmd, isQueryCmd;

    size_t maxInstQueueSize = 1000; // temporarily set to 1000

    TaskState currState;
    
    // bool idle() const { return currState == TaskState::IDLE; }
    // use when a real downstream reports completion
    // void completeCurrentTask();

    EventFunctionWrapper decodeEvent;
    EventFunctionWrapper prepareTaskEvent;
    EventFunctionWrapper enqueEvent;
    EventFunctionWrapper setCmdEvent;
    
    EventFunctionWrapper triggerEvent;
    EventFunctionWrapper loadEvent;
    EventFunctionWrapper storeEvent;
    EventFunctionWrapper p2sLEvent;
    EventFunctionWrapper p2sREvent;
    EventFunctionWrapper p2sRTEvent;
    EventFunctionWrapper calEvent;
    EventFunctionWrapper accEvent;
    EventFunctionWrapper switchEvent;              

    void processDecodeEvent();
    void processPrepareTaskEvent();
    void processEnqueEvent();
    void processSetCmdEvent();
    void processTriggerEvent();
    void processLoadEvent();
    void processStoreEvent();
    void processP2SLEvent();
    void processP2SREvent();
    void processP2SRTEvent();
    void processCalEvent();
    void processAccEvent();
    void processSwitchEvent();  
    // void scheduleDecodeIfNeeded();

  public:
    Scheduler(const SchedulerParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void handleFunctional(PacketPtr pkt);
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PICPortID picPortID, PacketPtr pkt);
    // void startup() override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_SCHEDULER_HH__
