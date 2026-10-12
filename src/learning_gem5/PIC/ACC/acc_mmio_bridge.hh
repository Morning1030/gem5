#ifndef __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__
#define __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__

#include <cstdint>
#include <deque>
#include <string>

#include "learning_gem5/PIC/ACC/accumulator.hh"
#include "learning_gem5/PIC/pic_protocol.hh"
#include "learning_gem5/PIC/Control/cmd_state_helper.hh"
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

class AccMmioBridge : public ClockedObject
{
  private:
    struct Task
    {
        QryTabClient clientID;
        uint8_t cmdID;
        
        ModuleID moduleID;
        PacketPtr pkt;
    };
    class CPUSidePort : public ResponsePort
    {
      private:
        AccMmioBridge *owner;
        PacketPtr blockedResponse = nullptr;

      public:
        CPUSidePort(const std::string &name, AccMmioBridge *owner);

        void sendResponse(PacketPtr pkt);

      protected:
        Tick recvAtomic(PacketPtr pkt) override {panic("%s atomic access unsupported", name());}
        void recvFunctional(PacketPtr pkt) override {panic("%s atomic access unsupported", name());}
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override {return {AddrRange(MmioBase, MmioBase + MmioWindowSize)};}
    };

    class MemSidePort : public RequestPort
    {
      private:
        AccMmioBridge *owner;
        PacketPtr blockedPacket = nullptr;

      public:
        MemSidePort(const std::string &name, AccMmioBridge *owner);
        void sendPacket(PacketPtr);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override {}
    };

    enum TaskState
        {
          IDLE,
          LOADING,
          STORING,
          P2SING,
          CALING,
          ACCING,
          SWITCHING
    };

    CPUSidePort mmioPort;
    MemSidePort accPort;
    MemSidePort cmdStateHelperPort;

    RequestorID requestorId;

    uint64_t src;                   // SET_SRC
    uint64_t dst;                   // SET_DST
    uint16_t row;                   // SET_SIZE
    uint16_t byte_per_row;          // SET_SIZE
    uint16_t offset;                // SET_SIZE
    PacketPtr paramPkt;
    Task nextEnqTask;

    bool srcValid = false;
    bool dstValid = false;

    PacketPtr blockedAccPkt = nullptr;
    PacketPtr pendingMmioParamPkt = nullptr;

    bool accInFlight = false;

    PacketPtr pendingMmioResponsePkt = nullptr;

    std::deque<PacketPtr> instQueue;
    size_t maxInstQueueSize = 1000;
    std::deque<Task> nextTask;
    std::deque<Task> nextImmTask;
    TaskState currState;

    EventFunctionWrapper mmioResponseEvent;
    EventFunctionWrapper decodeEvent;
    EventFunctionWrapper prepareTaskEvent;
    EventFunctionWrapper enqueEvent;
    EventFunctionWrapper setCmdEvent;
    EventFunctionWrapper triggerEvent;
    EventFunctionWrapper accEvent;

    bool handleRequest(PacketPtr pkt);
    bool handleAccResponse(PacketPtr pkt);

    void retryAccRequest();

    void sendMmioSuccess(PacketPtr pkt);
    void processMmioResponse();

    void launchAcc(PacketPtr mmioPkt, ParamFields fields);
    void accRequestAccepted();
    void processDecodeEvent();
    void processPrepareTaskEvent();
    void processEnqueEvent();
    void processSetCmdEvent();
    void processTriggerEvent();
    void processAccEvent();

    void triggerTS();

  public:
    AccMmioBridge(const AccMmioBridgeParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace pic
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_ACC_MMIO_BRIDGE_HH__
