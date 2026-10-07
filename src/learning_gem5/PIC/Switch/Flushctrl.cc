#include "learning_gem5/PIC/Switch/Flushctrl.hh"

#include <algorithm>

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Switch/switchCtrl.hh"
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

FlushController::FlushController(SwitchController *owner)
    : owner(owner),
      flushState(State::DealIdle),
      currentEntry{0, 0},
      flushSendEvent([this] { processFlushSendEvent(); }, owner->name() + ".flushSendEvent")
      
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
    if (flushState == State::DealIdle && !flushSendEvent.scheduled()) {
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
        picCtrlAddr(PicCtrlReg::Flush), pktSize, 0, owner->requestorId
    );
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();

    *pkt->getPtr<CacheFlushPayload>() = CacheFlushPayload{currentEntry.setID, currentEntry.tag};

    DPRINTF(SwitchCtrl, "FlushCtl: SendFlushReq set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);

    flushState = State::WaitFlushDone;
    owner->flushPort.sendPacket(pkt);
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
