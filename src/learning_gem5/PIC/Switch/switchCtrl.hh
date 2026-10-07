#ifndef __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__
#define __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__

#include <cstdint>
#include "base/types.hh"
#include "sim/eventq.hh"
#include "params/SwitchController.hh"
#include "sim/clocked_object.hh"

#include "learning_gem5/PIC/Switch/pic_payloads.hh"
#include "learning_gem5/PIC/Switch/Flushctrl.hh"

namespace gem5
{

class SwitchController : public ClockedObject
{
  private:
    class InstPort : public ResponsePort        // 接 Scheduler 的 sc_port
    {
      private:
        SwitchController *owner;
        PacketPtr blockedResponse = nullptr;
      public:
        InstPort(const std::string &name, SwitchController *owner);
        void sendResponse(PacketPtr pkt);
      protected:
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        Tick recvAtomic(PacketPtr) override;
        void recvFunctional(PacketPtr) override;
        AddrRangeList getAddrRanges() const override;
    };

    class CacheCtrlPort : public RequestPort    // 接 CacheController 的 pic_ctrl_port
    {
      private:
        SwitchController *owner;
        PacketPtr blockedPkt = nullptr;
      public:
        CacheCtrlPort(const std::string &name, SwitchController *owner);
        void sendPacket(PacketPtr pkt);
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override {}
    };

    class FlushPort : public RequestPort
    {
      private:
        SwitchController *owner;
        PacketPtr blockedPkt = nullptr;
      public:
        FlushPort(const std::string &name, SwitchController *owner);
        void sendPacket(PacketPtr pkt);
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override {}
    };

    InstPort instPort;
    CacheCtrlPort cacheCtrlPort;
    FlushPort flushPort;
    RequestorID requestorId;

    const uint32_t numSets;
    const uint32_t numWays;
    const uint32_t nWayPerLevel;
    const uint32_t picAvailLevels;
    enum class SwitchType {PIC2Cache, Cache2PIC};

    enum class State {
        Idle,
        WaitCacheIdle,    // RTL: activate_pre_check
        WaitWayMode,      // wait PicWayModeResp
        WaitDirResult,    // RTL: activate_dirResp
        CheckFinish,      // RTL: check_finish
        WaitFlush         // RTL: resp_op_res
    };
    State switchState;
    uint32_t cacheModeEndWay;  // RTL: runtime_cache_Mode_endWayID
    uint32_t picLevels;        // RTL: runtime_PIC_Mode_levels        
    uint32_t setID;
    uint32_t beginWay;
    uint32_t endWay;
    uint8_t reqLevels;
    bool opSuccess;
    SwitchType currSwitchType;

    FlushController flushCtl;
    friend class FlushController;

    EventFunctionWrapper drainQueryEvent;     // send drain query
    EventFunctionWrapper queryEvent;          // send directory query
    EventFunctionWrapper switch2PICEvent;
    EventFunctionWrapper switch2CacheEvent;

    void processDrainQueryEvent();
    void processQueryEvent();
    void processSwitch2PICEvent();
    void processSwitch2CacheEvent();
    void processNextSet();
    void sendWayMode(bool picMode);

  public:
    SwitchController(const SwitchControllerParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;void processSwitchEvent(bool allocate, uint8_t nLevels);
    bool handleDirResponse(PacketPtr pkt);  
    bool handleFlushResponse(PacketPtr pkt);
    bool handleControlRequest(PacketPtr pkt);
    bool isIdle() const { return switchState == State::Idle; }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__
