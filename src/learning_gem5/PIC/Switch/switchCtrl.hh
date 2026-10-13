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

class CacheController;

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
    CacheController *cache;

    const uint32_t numSets;
    const uint32_t numWays;
    const uint32_t nWayPerLevel;
    const uint32_t picAvailLevels;
    const uint32_t totalMatNum;
    const uint32_t totalLevels;     // num_ways / n_way_per_level
    const uint32_t nMatPerLevel;    // total_mat_num / totalLevels

    enum class State {                 // RTL SwitchCtl_pic switch_state
        Idle, ActPreCheck, ActQueryDir, ActDirResp, CheckFinish,
        DeactPreCheck, RespOpRes
    };
    enum class DirState { Idle, SaveRegout, Resp };   // RTL Directory sw_state

    State    switchState = State::Idle;
    DirState dirState    = DirState::Idle;

    uint32_t cacheModeEndWay;  // RTL: runtime_cache_Mode_endWayID
    uint32_t picLevels;        // RTL: runtime_PIC_Mode_levels（pre_check 就生效，給 Directory）
    uint32_t picLevelsForAssert = 0;  // RTL: runtime_PIC_Mode_levels_for_assert（switch_resp 才生效，給 Bank）
    uint32_t respMatIdBegin = 0;      // RTL: respReg.avail_MatID_begin（失敗時保留舊值）
    uint32_t beginWay = 0;
    uint32_t endWay = 0;
    uint8_t  reqLevels = 0;
    bool     opSuccess = false;
    bool     isAlloc = false;

    uint32_t querySetPtr = 0;
    uint32_t dirWayPtr   = 0;
    CacheWayQueryRespPayload snap{};   // regout_for_switch_query
    bool     snapValid = false;
    uint32_t snapIdx   = 0;
    bool     needUnblock = false;
    PacketPtr pendingInstPkt = nullptr;   // switch_resp 要到 resp_op_res 才回

    FlushController flushCtl;
    friend class FlushController;

    EventFunctionWrapper tickEvent;
    void tick();
    void sendWayQuery();
    void respondSwitch();

  public:
    SwitchController(const SwitchControllerParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    bool handleDirResponse(PacketPtr pkt);  
    bool handleFlushResponse(PacketPtr pkt);
    bool handleControlRequest(PacketPtr pkt);
    bool isIdle() const { return switchState == State::Idle; }

    // RTL: io.picActivated / io.cacheLevelEnd → BankedStore（PIC 端存取合法性）
    bool picActivated() const { return picLevelsForAssert > 0; }
    uint32_t cacheLevelEnd() const
    { return totalLevels - 1 - picLevelsForAssert; }
    // RTL BankSellPIC::ifValidPIC_Req（matID 為全域編號）
    bool isPicMatAccessible(uint32_t matID) const
    { return picActivated() && matID / nMatPerLevel > cacheLevelEnd(); }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__
