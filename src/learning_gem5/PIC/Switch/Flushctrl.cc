#include "learning_gem5/PIC/Switch/Flushctrl.hh"

#include <algorithm>

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

// =================================================================
//  FlushController  —  Independent flush FSM (pipeline)
// =================================================================
//  RTL equivalent: flush_queue + flush FSM in SwitchCtl_pic.scala
//    flush_idle → send_flush_req → wait_flushDone → flush_idle
// =================================================================

FlushController::FlushController(Scheduler *owner)
    : owner(owner),
      flushState(State::DealIdle),
      currentEntry{0, 0},
      flushSendEvent([this] { processFlushSendEvent(); }, "flushSendEvent"),
      flushDoneEvent([] {}, "flushDoneEvent")   // response-driven, not self-scheduled
{
}

// -----------------------------------------------------------------
//  enqueueFlush  —  Called by SwitchCtl from WaitDirResult
// -----------------------------------------------------------------
//  If DealIdle, immediately schedule a send. Otherwise the queue
//  will be consumed when current flush completes.
// -----------------------------------------------------------------

void
FlushController::enqueueFlush(uint32_t setID, Addr tag)
{
    flushQueue.push({setID, tag});

    DPRINTF(SwitchCtrl, "FlushCtl: enqueue set=%u tag=%#x (queueSize=%u)\n",
            setID, tag, (unsigned)flushQueue.size());

    // If idle, kick off processing
    if (flushState == State::DealIdle && !owner->isEventScheduled(flushSendEvent)) {
        owner->schedule(flushSendEvent, owner->clockEdge(Cycles(1)));
    }
}

// -----------------------------------------------------------------
//  processFlushSendEvent  —  Dequeue and send FlushReq packet
// -----------------------------------------------------------------
//  RTL: flush_idle → dequeue → send_flush_req → flushReq.fire
// -----------------------------------------------------------------

void
FlushController::processFlushSendEvent()
{
    assert(!flushQueue.empty());

    currentEntry = flushQueue.front();
    flushQueue.pop();

    // Build FlushReq packet with CacheFlushPayload
    const size_t pktSize = sizeof(CacheFlushPayload);
    RequestPtr req = std::make_shared<Request>(
        0, pktSize, 0, owner->requestorId
    );
    PacketPtr pkt = new Packet(req, MemCmd::FlushReq);
    pkt->allocate();

    CacheFlushPayload payload{currentEntry.setID, currentEntry.tag};
    pkt->setData(reinterpret_cast<const uint8_t*>(&payload));

    DPRINTF(SwitchCtrl, "FlushCtl: SendFlushReq set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);

    flushState = State::SendFlushReq;

    if (!owner->cacheControllerPort.sendTimingReq(pkt)) {
        // Port busy — retry next cycle
        // Push entry back to front
        DPRINTF(SwitchCtrl, "FlushCtl: port busy, retrying next cycle\n");
        flushState = State::DealIdle;
        delete pkt;
        // Put the entry back
        std::queue<FlushEntry> temp;
        temp.push(currentEntry);
        while (!flushQueue.empty()) {
            temp.push(flushQueue.front());
            flushQueue.pop();
        }
        flushQueue = temp;
        owner->schedule(flushSendEvent, owner->clockEdge(Cycles(1)));
        return;
    }

    // Packet sent — transition to WaitFlushDone
    flushState = State::WaitFlushDone;
}

// -----------------------------------------------------------------
//  handleFlushResponse  —  Flush completion from CacheController
// -----------------------------------------------------------------
//  RTL: wait_flushDone → flushDone → flush_idle
//  If queue is not empty, immediately schedule next flush send.
// -----------------------------------------------------------------

bool
FlushController::handleFlushResponse(PacketPtr pkt)
{
    if (flushState != State::WaitFlushDone) {
        return false;
    }

    DPRINTF(SwitchCtrl, "FlushCtl: FlushDone set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);
    delete pkt;

    flushState = State::DealIdle;

    // If there are more entries, schedule next flush
    if (!flushQueue.empty()) {
        owner->schedule(flushSendEvent, owner->clockEdge(Cycles(1)));
    }

    return true;
}

} // namespace gem5
