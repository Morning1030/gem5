#ifndef __LEARNING_GEM5_PIC_SWITCH_PIC_PAYLOADS_HH__
#define __LEARNING_GEM5_PIC_SWITCH_PIC_PAYLOADS_HH__

#include <cstdint>
#include "base/types.hh"

namespace gem5
{

constexpr uint32_t PIC_MAX_WAYS = 16;   
constexpr uint64_t PicCtrlBase = 0x10029000ULL;
constexpr uint64_t PicCtrlWindowSize = 0x100ULL;

enum class PicCtrlReg : uint64_t
{
    DrainQuery = 0x00,
    WayQuery   = 0x08,
    Flush      = 0x10,
    WayMode    = 0x18,
    SwitchBusy = 0x20, 
};

constexpr uint64_t
picCtrlAddr(PicCtrlReg reg)
{
    return PicCtrlBase + static_cast<uint64_t>(reg);
}

// SwitchCtrl -> CacheController
struct CacheWayQueryPayload
{
    uint32_t setID;
    uint32_t beginWay;
    uint32_t endWay;
};

// CacheController -> SwitchCtrl
struct CacheWayQueryRespPayload
{
    uint32_t numValid;
    uint32_t wayID[PIC_MAX_WAYS];
    Addr     tag[PIC_MAX_WAYS];
};

// FlushCtrl -> CacheController 
struct CacheFlushPayload
{
    uint32_t setID;
    Addr     tag;
};

// Scheduler -> SwitchCtrl（switch_req）
// RTL: SwitchInfo{op, nLevels}；op = true 為 ALLOC（PIC_Switch.ALLOC = true.B）
// 版面必須和 Control/scheduler.hh 的 SwitchPayload 相同（這裡不 include 它，
// 因為 Scheduler 目前沒有編進去，params/Scheduler.hh 不存在）
struct SwitchReqPayload
{
    bool    opType;
    uint8_t nLevels;
};

// SwitchCtrl -> Scheduler（switch_resp，寫回原本的 switch_req packet）
// RTL: SwitchResult{op_success, avail_MatID_begin}
struct SwitchRespPayload
{
    uint8_t opSuccess;
    uint8_t availMatIdBegin;
};

// SwitchCtrl -> CacheController
struct DrainQueryPayload
{
    uint32_t wayID;
};

// CacheController -> SwitchCtrl
struct DrainQueryRespPayload
{
    uint32_t isDrained;   // 1 = MSHR idle, 0 = busy
};

//SwitchCtrl -> CacheController (PicWayModeReq)
struct WayModePayload
{
    uint32_t beginWay;
    uint32_t endWay;
    uint32_t picMode;    // 1 = PIC（不可以被 findVictim 選）, 0 = cache
};

// SwitchCtrl -> CacheController
// switchCtl.io.switchIdle → Scheduler.scala 的 sinkA.io.req.ready
struct SwitchBusyPayload
{
    uint32_t busy;  
};
static_assert(sizeof(SwitchBusyPayload) == 4, "...");

constexpr bool
isDirReg(PicCtrlReg r)
{
    return r == PicCtrlReg::DrainQuery ||
           r == PicCtrlReg::WayQuery ||
           r == PicCtrlReg::WayMode ||
           r == PicCtrlReg::SwitchBusy;
}


static_assert(sizeof(CacheWayQueryPayload) == 12,
              "CacheWayQueryPayload layout changed unexpectedly");
static_assert(sizeof(CacheWayQueryRespPayload) == 200,
              "CacheWayQueryRespPayload layout changed unexpectedly");
static_assert(sizeof(CacheFlushPayload) == 16,
              "CacheFlushPayload layout changed unexpectedly");
static_assert(sizeof(DrainQueryPayload) == 4, "...");
static_assert(sizeof(DrainQueryRespPayload) == 4, "...");
static_assert(sizeof(WayModePayload) == 12, "...");
static_assert(sizeof(SwitchReqPayload) == 2,
              "SwitchReqPayload must match Scheduler's SwitchPayload");
static_assert(sizeof(SwitchRespPayload) == 2,
              "SwitchRespPayload must fit in the 2-byte SwitchPayload packet");

} 

#endif 
