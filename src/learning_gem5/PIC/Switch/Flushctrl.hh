#ifndef __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__
#define __LEARNING_GEM5_PIC_SWITCH_FLUSHCTRL_HH__

#include <cstdint>
#include <queue>
#include "base/types.hh"
#include "mem/packet.hh"

#include "learning_gem5/PIC/Switch/pic_payloads.hh"

namespace gem5
{

class SwitchController;   // forward declare

// =====================================================================
//  FlushController — flush FSM (RTL: flush_queue + flush FSM in SwitchCtl_pic)
//
//  沒有自己的 event：每拍由 SwitchController::tick() 呼叫 tick()。
//  enqueueFlush() 只 push，下一拍的 tick() 才看得到（Chisel Queue 語意）。
//
//  States:  Idle (deq) → SendFlushReq (fire) → WaitFlushDone → Idle
// =====================================================================

class FlushController {
  public:
    enum class State { Idle, SendFlushReq, WaitFlushDone };
    static constexpr size_t kQueueDepth = 16;      // RTL flush_queue depth

  private:
    SwitchController *owner;
    State flushState = State::Idle;

    struct FlushEntry { uint32_t setID; Addr tag; };
    std::queue<FlushEntry> flushQueue;
    FlushEntry currentEntry{0, 0};
    bool doneSeen = false;                         // io.flushDone（組合訊號）

    void sendFlushReq();

  public:
    explicit FlushController(SwitchController *owner) : owner(owner) {}

    /** RTL flush_queue.io.enq.ready */
    bool queueNotFull() const { return flushQueue.size() < kQueueDepth; }

    /** 只 push，下一拍才看得到 */
    void enqueueFlush(uint32_t setID, Addr tag);

    /** 由 SwitchController::tick 每拍呼叫 */
    void tick();

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
