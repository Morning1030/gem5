#ifndef __LEARNING_GEM5_PIC_SWITCH_PIC_PAYLOADS_HH__
#define __LEARNING_GEM5_PIC_SWITCH_PIC_PAYLOADS_HH__

#include <cstdint>
#include "base/types.hh"

namespace gem5
{

// SwitchCtrl -> CacheController
struct CacheWayQueryPayload
{
    uint32_t setID;
    uint32_t wayID;
};

// CacheController -> SwitchCtrl
struct CacheWayQueryRespPayload
{
    Addr     tag;      // tag of the block (meaningful only when state != 0)
    uint32_t state;    // 0 = INVALID, non-zero = VALID
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

} 

#endif 
