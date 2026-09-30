#include "learning_gem5/PIC/Switch/switchCtrl.hh"

#include <algorithm>

#include "base/trace.hh"
#include "debug/SwitchCtrl.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

Scheduler::SwitchController::SwitchController(Scheduler *owner)
: owner(owner),
    setID(0),
    wayID(0),
    currSwitchType(SwitchType::PIC2Cache),
    FlushController(owner),
    drainQueryEvent([this] { processDrainQueryEvent(); }, "switchEvent"),,
    requestFlushEvent([this] { processRequestFlushEvent(); }, "flushEvent"),
    switch2PICEvent([this] { processSwitch2PICEvent(); }, "switch2PICEvent"),
    switch2CacheEvent([this] { processSwitch2CacheEvent(); }, "switch2CacheEvent")
{
}

void
Scheduler::SwitchController::processSwitchEvent(bool allocate, uint32_t way, uint32_t sets)
{
    assert(isIdle() && "Cannot start switch while another is in progress");

    currSwitchType = allocate ? SwitchType::Cache2PIC : SwitchType::PIC2Cache;
    wayID   = way;
    numSets = sets;
    setID   = 0;

    DPRINTF(SwitchCtrl,
            "Switch started: type=%s wayID=%u numSets=%u\n",
            currSwitchType == SwitchType::Cache2PIC ? "Cache2PIC" : "PIC2Cache",
            wayID, numSets);

    if (currSwitchType == SwitchType::Cache2PIC) {
        // ---- C2P: must drain MSHR first ----
        //  RTL: activate_pre_check waits for isScheNoOtherWorks
        switchState = State::WaitCacheIdle;
        owner->schedule(drainQueryEvent, owner->clockEdge(Cycles(1)));
    } else {
        // ---- P2C: no drain, no query, no flush needed ----
        //  RTL: deactivate_pre_check → resp_op_res (one-shot)
        //  PIC mode way has no valid cache lines, just toggle mode.
        DPRINTF(SwitchCtrl,
                "PIC2Cache: fast path, scheduling switch2CacheEvent\n");
        owner->schedule(switch2CacheEvent, owner->clockEdge(Cycles(1)));
    }
}

void
Scheduler::SwitchController::processDrainQueryEvent()
{
    const size_t pktSize = std::max(sizeof(DrainQueryPayload),
                                    sizeof(DrainQueryRespPayload));

    RequestPtr req = std::make_shared<Request>(
        0, pktSize, 0, owner->requestorId
    );

    PacketPtr pkt = new Packet(req, MemCmd::DrainQueryReq);
    pkt->allocate();

    DrainQueryPayload payload{wayID};
    pkt->setData(reinterpret_cast<const uint8_t*>(&payload));

    DPRINTF(SwitchCtrl, "DrainQuery: wayID=%u\n", wayID);

    if (!owner->cacheControllerPort.sendTimingReq(pkt)) {
        delete pkt;
        owner->schedule(drainQueryEvent, owner->clockEdge(Cycles(1)));
        return;
    }
    // Stay in WaitCacheIdle, response handled in handleResponse
}

void
Scheduler::SwitchController::processQueryEvent()
{
    const size_t pktSize = std::max(sizeof(CacheWayQueryPayload),
                                    sizeof(CacheWayQueryRespPayload));

    RequestPtr req = std::make_shared<Request>(
        0, pktSize, 0, owner->requestorId
    );

    PacketPtr pkt = new Packet(req, MemCmd::QueryReq);
    pkt->allocate();

    CacheWayQueryPayload payload{setID, wayID};
    pkt->setData(reinterpret_cast<const uint8_t*>(&payload));

    DPRINTF(SwitchCtrl, "Query: set=%u way=%u\n", setID, wayID);

    switchState = State::QueryDirectory;

    if (!owner->cacheControllerPort.sendTimingReq(pkt)) {
        // Port busy — retry next cycle
        switchState = State::WaitCacheIdle;  // back to safe state
        delete pkt;
        owner->schedule(queryEvent, owner->clockEdge(Cycles(1)));
        return;
    }

    // Packet sent — wait for response
    switchState = State::WaitDirResult;
}

bool
Scheduler::SwitchController::handleResponse(PacketPtr pkt)
{
    // ---- Drain response ----
    if (switchState == State::WaitCacheIdle) {
        DrainQueryRespPayload resp{};
        pkt->writeData(reinterpret_cast<uint8_t*>(&resp));

        DPRINTF(SwitchCtrl, "DrainResp: isDrained=%u\n", resp.isDrained);
        delete pkt;

        if (resp.isDrained) {
            // MSHR is idle — start querying directory
            //  RTL: activate_pre_check → activate_queryDir
            DPRINTF(SwitchCtrl, "MSHR drained, starting directory query\n");
            owner->schedule(queryEvent, owner->clockEdge(Cycles(1)));
        } else {
            // Not yet — retry drain query (RTL: busy-wait in same state)
            DPRINTF(SwitchCtrl, "MSHR not drained, retrying\n");
            owner->schedule(drainQueryEvent, owner->clockEdge(Cycles(10)));
        }
        return true;
    }
    // ---- Query response ----
    if (switchState == State::WaitDirResult) {
        CacheWayQueryRespPayload resp{};
        pkt->writeData(reinterpret_cast<uint8_t*>(&resp));

        DPRINTF(SwitchCtrl,
                "QueryResp: set=%u way=%u valid=%u tag=%#x\n",
                setID, wayID, resp.state, resp.tag);

        delete pkt;

        if (resp.state != 0) {
            // ---- VALID line: enqueue flush (pipeline, don't wait) ----
            //  RTL: activate_dirResp → flush_queue.enq
            flushCtl.enqueueFlush(setID, resp.tag);
        }

        // ---- Advance to next set (RTL: check_finish) ----
        processNext();
        return true;
    }
 
    // ---- Flush responses go to FlushController ----
    if (flushCtl.getState() == FlushController::State::WaitFlushDone) {
        bool handled = flushCtl.handleFlushResponse(pkt);
        if (handled && switchState == State::WaitFlush) {
            // Check if all flushes are done now
            if (flushCtl.isFlushQueueEmpty()) {
                DPRINTF(SwitchCtrl, "All flushes complete, finalising C2P\n");
                owner->schedule(switch2PICEvent,
                                owner->clockEdge(Cycles(1)));
            }
            // else: still waiting, FlushCtl will keep processing
        }
        return handled;
    }

    return false;   // not ours
}



void
Scheduler::SwitchController::processNext()
{
    setID++;

    if (setID < numSets) {
        // More sets to scan — continue query loop
        //  RTL: check_finish → activate_queryDir (setPtr++)
        DPRINTF(SwitchCtrl, "NextSet: setID=%u / %u\n", setID, numSets);
        switchState = State::CheckFinish;   // transient, immediately schedule
        owner->schedule(queryEvent, owner->clockEdge(Cycles(1)));
    } else {
        // All sets scanned — wait for FlushCtl to finish
        //  RTL: check_finish → resp_op_res
        DPRINTF(SwitchCtrl,
                "All %u sets scanned. Waiting for flush queue to drain.\n",
                numSets);

        switchState = State::WaitFlush;

        // Check if flushes already done (e.g., no dirty lines at all)
        if (flushCtl.isFlushQueueEmpty()) {
            DPRINTF(SwitchCtrl, "Flush queue already empty, finalising\n");
            owner->schedule(switch2PICEvent,
                            owner->clockEdge(Cycles(1)));
        }
        // else: handleResponse will schedule switch2PICEvent when done
    }
}

void
Scheduler::SwitchController::processSwitch2PICEvent()
{
    DPRINTF(SwitchCtrl, "Switch2PIC: wayID=%u → PIC mode ON\n", wayID);

    // TODO: send Cache2PIC command to CacheController
    //       (setWayPICMode(wayID, true))
    //       then switchState = State::Idle, notify TaskScheduler

    switchState = State::Idle;
}

void
Scheduler::SwitchController::processSwitch2CacheEvent()
{
    DPRINTF(SwitchCtrl, "Switch2Cache: wayID=%u → PIC mode OFF\n", wayID);

    // TODO: send PIC2Cache command to CacheController
    //       (setWayPICMode(wayID, false))
    //       then switchState = State::Idle, notify TaskScheduler

    switchState = State::Idle;
}


} // namespace gem5