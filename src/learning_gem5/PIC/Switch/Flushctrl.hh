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

class SwitchController;   // forward declare

// 同一 tick 的先後：port response（Default_Pri）→ SwitchController 的 event
// → FlushController 的 event。switch 讀的是「本拍開始時」的 flush 狀態
// （queue 滿不滿、flush 全部做完沒），所以 flush 的 event 排在後面。
constexpr Event::Priority SwitchEventPri = Event::CPU_Tick_Pri;
constexpr Event::Priority FlushEventPri  = Event::CPU_Tick_Pri + 1;

// =====================================================================
//  FlushController — flush FSM (RTL: flush_queue + flush FSM in SwitchCtl_pic)
//
//  不是 SimObject：SwitchController 底下的一般 class，沒有自己的 port，
//  event 透過 owner 排程，FlushReq 走 owner 的 cache_side[Flush]。
//  enqueueFlush() push 的項目下一拍才看得到（Chisel Queue 語意）。
//
//  States:  Idle (deqEvent) → SendFlushReq (sendEvent) → WaitFlushDone
//           → FlushDone 回來 (doneEvent) → Idle
// =====================================================================

class FlushController {
  public:
    enum class State { Idle, SendFlushReq, WaitFlushDone };
    static constexpr size_t kQueueDepth = 16;      // RTL flush_queue depth

  private:
    SwitchController *owner;
    State flushState = State::Idle;

    struct FlushEntry { uint32_t setID; Addr tag; Tick visibleAt; };
    std::queue<FlushEntry> flushQueue;
    FlushEntry currentEntry{0, 0, 0};

    EventFunctionWrapper deqEvent;     // flush_idle：deq
    EventFunctionWrapper sendEvent;    // send_flush_req：flushReq.valid
    EventFunctionWrapper doneEvent;    // wait_flushDone → flush_idle

    void processDeqEvent();
    void processSendEvent();
    void processDoneEvent();

  public:
    explicit FlushController(SwitchController *owner);

    /** RTL flush_queue.io.enq.ready */
    bool queueNotFull() const { return flushQueue.size() < kQueueDepth; }

    /** push，下一拍才看得到；flush idle 時排 deqEvent */
    void enqueueFlush(uint32_t setID, Addr tag);

    /** Handle flush-done response from CacheController. */
    bool handleFlushResponse(PacketPtr pkt);

    /** True when queue is empty AND state is Idle. */
    bool isFlushQueueEmpty() const
    {
        return flushQueue.empty() && flushState == State::Idle;
    }

    State getState() const { return flushState; }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__
