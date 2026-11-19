#include "learning_gem5/PIC/Switch/Flushctrl.hh"

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Switch/switchCtrl.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

// =================================================================
//  FlushController  —  event-driven flush FSM
// =================================================================
//  RTL equivalent: flush_queue + flush FSM in SwitchCtl_pic.scala
//    flush_idle (deq) → send_flush_req (fire) → wait_flushDone → flush_idle
//  只在狀態會變的那拍被叫醒：
//    deqEvent   queue 有東西、而且 flush idle
//    sendEvent  deq 的下一拍
//    doneEvent  FlushDone response 剛好在 edge 上回來時，本拍處理
//               （QueuedResponsePort 晚 1 tick 到的情況直接 finishFlush）
// =================================================================

FlushController::FlushController(SwitchController *owner)
    : owner(owner),
      deqEvent([this] { processDeqEvent(); },
               owner->name() + ".flushCtl.deqEvent", false, FlushEventPri),
      sendEvent([this] { processSendEvent(); },
                owner->name() + ".flushCtl.sendEvent", false, FlushEventPri),
      doneEvent([this] { processDoneEvent(); },
                owner->name() + ".flushCtl.doneEvent", false, FlushEventPri)
{
}

void
FlushController::enqueueFlush(uint32_t setID, Addr tag)
{
    panic_if(!queueNotFull(), "FlushCtl: enqueue while full");
    // 本拍 enqueue 的，下一拍的 flush_idle 才 deq 得到
    const Tick visibleAt = owner->clockEdge(Cycles(1));
    flushQueue.push({setID, tag, visibleAt});

    DPRINTF(SwitchCtrl, "FlushCtl: enqueue set=%u tag=%#x (queueSize=%u)\n",
            setID, tag, (unsigned)flushQueue.size());

    // flush 忙的時候不用排：doneEvent 回到 idle 時會排 deqEvent
    if (flushState == State::Idle && !deqEvent.scheduled())
        owner->schedule(deqEvent, visibleAt);
}

void
FlushController::processDeqEvent()                // RTL flush_idle：deq.ready = 1
{
    panic_if(flushState != State::Idle, "FlushCtl: deq while state=%d",
             (int)flushState);
    if (flushQueue.empty())
        return;
    if (flushQueue.front().visibleAt > curTick()) {
        owner->schedule(deqEvent, flushQueue.front().visibleAt);
        return;
    }

    currentEntry = flushQueue.front();
    flushQueue.pop();
    flushState = State::SendFlushReq;
    owner->schedule(sendEvent, owner->clockEdge(Cycles(1)));

    owner->flushDequeued();                       // queue 騰出位置
}

void
FlushController::processSendEvent()               // RTL send_flush_req：flushReq.valid
{
    // ready 的延遲由 CacheController 模擬
    RequestPtr req = std::make_shared<Request>(
        picCtrlAddr(PicCtrlReg::Flush), sizeof(CacheFlushPayload), 0,
        owner->requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    *pkt->getPtr<CacheFlushPayload>() =
        CacheFlushPayload{currentEntry.setID, currentEntry.tag};

    DPRINTF(SwitchCtrl, "FlushCtl: SendFlushReq set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);

    flushState = State::WaitFlushDone;
    owner->cacheSidePorts[SwitchController::Flush].sendPacket(pkt);
}

bool
FlushController::handleFlushResponse(PacketPtr pkt)
{
    if (flushState != State::WaitFlushDone || doneEvent.scheduled())
        return false;

    DPRINTF(SwitchCtrl, "FlushCtl: FlushDone set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);
    delete pkt;

    // RTL: switchFlushOK 是組合邏輯（FlushReqRouter is_switchCtlReq →
    // Scheduler.scala:426），SourceX 回應的那拍 T 就是 wait_flushDone 看到
    // flushDone 的那拍，T+1 回到 flush_idle。
    if (owner->clockEdge() == curTick()) {
        // 剛好在 edge 上收到：排在本拍 switch event 之後處理
        owner->schedule(doneEvent, curTick());
    } else {
        // CacheController 走 QueuedResponsePort，一律晚 1 tick 送到
        // （mem/packet_queue.cc: when = max(when, curTick() + 1)）。
        // 邏輯上是前一個 edge T 收到的；T 的 event 都跑完了，直接處理，
        // 不要再往上取整到下一個 edge（那樣會多算一拍）。
        finishFlush(owner->clockEdge() - owner->clockPeriod());
    }
    return true;
}

void
FlushController::processDoneEvent()               // RTL wait_flushDone → flush_idle
{
    finishFlush(curTick());
}

void
FlushController::finishFlush(Tick doneEdge)       // doneEdge = RTL 看到 flushDone 的那拍 T
{
    flushState = State::Idle;
    const Tick idleCycle = doneEdge + owner->clockPeriod();   // T+1：flush_idle
    if (!flushQueue.empty()) {
        // T+1 flush_idle deq（visibleAt 比 T+1 晚的，processDeqEvent 會再延後）
        if (!deqEvent.scheduled())
            owner->schedule(deqEvent, idleCycle);
    } else {
        owner->flushAllDone(idleCycle);           // resp_op_res 在 T+1 看得到
    }
}

} // namespace gem5
