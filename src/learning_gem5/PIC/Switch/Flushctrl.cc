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
//    doneEvent  FlushDone response 回來的那拍
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
    // 這拍（不在 edge 上就是下一個 edge）回到 idle
    owner->schedule(doneEvent, owner->clockEdge());
    return true;
}

void
FlushController::processDoneEvent()               // RTL wait_flushDone → flush_idle
{
    flushState = State::Idle;
    if (!flushQueue.empty()) {
        // 下一拍 flush_idle deq（包含這拍剛 enqueue 的）
        if (!deqEvent.scheduled())
            owner->schedule(deqEvent, owner->clockEdge(Cycles(1)));
    } else {
        owner->flushAllDone();                    // resp_op_res 下一拍看得到
    }
}

} // namespace gem5
