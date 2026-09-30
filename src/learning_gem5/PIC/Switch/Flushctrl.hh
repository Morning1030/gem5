#ifndef __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__
#define __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__

#include <cstdint>
#include <queue>
#include "base/types.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"

#include "learning_gem5/PIC/Switch/pic_payloads.hh"

namespace gem5
{

class Scheduler;   // forward declare

// =====================================================================
//  FlushController — independent flush FSM (RTL: flush FSM in SwitchCtl_pic)
//
//  Maintains an internal queue of (setID, tag) flush requests.
//  SwitchController enqueues via enqueueFlush() and polls
//  isFlushQueueEmpty() at the end.
//
//  States:  DealIdle → SendFlushReq → WaitFlushDone → DealIdle
// =====================================================================

class FlushController {
  public:
    enum class State { DealIdle, SendFlushReq, WaitFlushDone };

  private:
    Scheduler *owner;
    State      flushState;

    // Internal flush queue (RTL: flush_queue, depth 16)
    struct FlushEntry { uint32_t setID; Addr tag; };
    std::queue<FlushEntry> flushQueue;

    // Current entry being flushed
    FlushEntry currentEntry;

    EventFunctionWrapper flushSendEvent;
    EventFunctionWrapper flushDoneEvent;   
                                         

    void processFlushSendEvent();

  public:
    FlushController(Scheduler *owner);

    /** SwitchCtl calls this to enqueue a dirty line for flush (direct call). */
    void enqueueFlush(uint32_t setID, Addr tag);

    /** Handle flush-done response from CacheController. */
    bool handleFlushResponse(PacketPtr pkt);

    /** True when queue is empty AND state is DealIdle. */
    bool isFlushQueueEmpty() const
    {
      return flushQueue.empty() && flushState == State::DealIdle;
    }

    State getState() const { return flushState; }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__
