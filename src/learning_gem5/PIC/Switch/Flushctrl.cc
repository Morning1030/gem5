#include "learning_gem5/PIC/Switch/Flushctrl.hh"

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Switch/switchCtrl.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace gem5
{

// =================================================================
//  FlushController  —  per-cycle flush FSM
// =================================================================
//  RTL equivalent: flush_queue + flush FSM in SwitchCtl_pic.scala
//    flush_idle (deq) → send_flush_req (fire) → wait_flushDone → flush_idle
// =================================================================

void
FlushController::enqueueFlush(uint32_t setID, Addr tag)
{
    panic_if(!queueNotFull(), "FlushCtl: enqueue while full");
    flushQueue.push({setID, tag});

    DPRINTF(SwitchCtrl, "FlushCtl: enqueue set=%u tag=%#x (queueSize=%u)\n",
            setID, tag, (unsigned)flushQueue.size());
}

void
FlushController::tick()
{
    switch (flushState) {
      case State::Idle:                       // RTL flush_idle：deq.ready = 1
        if (!flushQueue.empty()) {
            currentEntry = flushQueue.front();
            flushQueue.pop();
            flushState = State::SendFlushReq;
        }
        break;

      case State::SendFlushReq:               // RTL send_flush_req：flushReq.valid
        sendFlushReq();                       // ready 的延遲由 CacheController 模擬
        flushState = State::WaitFlushDone;
        break;

      case State::WaitFlushDone:
        if (doneSeen) {
            doneSeen = false;
            flushState = State::Idle;
        }
        break;
    }
}

void
FlushController::sendFlushReq()
{
    RequestPtr req = std::make_shared<Request>(
        picCtrlAddr(PicCtrlReg::Flush), sizeof(CacheFlushPayload), 0,
        owner->requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    *pkt->getPtr<CacheFlushPayload>() =
        CacheFlushPayload{currentEntry.setID, currentEntry.tag};

    DPRINTF(SwitchCtrl, "FlushCtl: SendFlushReq set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);

    owner->flushPort.sendPacket(pkt);
}

bool
FlushController::handleFlushResponse(PacketPtr pkt)
{
    if (flushState != State::WaitFlushDone)
        return false;

    DPRINTF(SwitchCtrl, "FlushCtl: FlushDone set=%u tag=%#x\n",
            currentEntry.setID, currentEntry.tag);
    delete pkt;
    doneSeen = true;                          // 同一拍的 tick 會看到
    return true;
}

} // namespace gem5
