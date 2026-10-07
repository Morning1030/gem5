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

constexpr bool
isDirReg(PicCtrlReg r)
{
    return r == PicCtrlReg::DrainQuery ||
           r == PicCtrlReg::WayQuery   ||
           r == PicCtrlReg::WayMode;
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

} 

#endif 
