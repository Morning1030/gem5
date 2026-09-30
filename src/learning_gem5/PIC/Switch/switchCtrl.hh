#ifndef __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__
#define __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__

#include <cstdint>
#include "base/types.hh"
#include "sim/eventq.hh"

#include "learning_gem5/PIC/Switch/pic_payloads.hh"
#include "learning_gem5/PIC/Switch/Flushctrl.hh"

namespace gem5
{

class Scheduler;   // forward declare

class SwitchController {
      private:
        enum class SwitchType {PIC2Cache, Cache2PIC};

        enum class State {
            Idle,
            WaitCacheIdle,    // RTL: activate_pre_check
            QueryDirectory,   // RTL: activate_queryDir
            WaitDirResult,    // RTL: activate_dirResp
            CheckFinish,      // RTL: check_finish
            WaitFlush         // RTL: resp_op_res
        };
        Scheduler *owner;
        State switchState;
        uint32_t setID;
        uint32_t wayID;
        uint32_t numSets;
        SwitchType currSwitchType;

        FlushController flushCtl;

        EventFunctionWrapper drainQueryEvent;     // send drain query
        EventFunctionWrapper queryEvent;          // send directory query
        EventFunctionWrapper switch2PICEvent;
        EventFunctionWrapper switch2CacheEvent;

        void processDrainQueryEvent();
        void processQueryEvent();
        void processSwitch2PICEvent();
        void processSwitch2CacheEvent();
        void processNextSet();

      public:
        SwitchController(Scheduler *owner);
        void processSwitchEvent(bool allocate, uint32_t way, uint32_t sets);
        bool handleResponse(PacketPtr pkt);
        bool isIdle() const { return switchState == State::Idle; }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_SWITCH_CTRL_HH__
