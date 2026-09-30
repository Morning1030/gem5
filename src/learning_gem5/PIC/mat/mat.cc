#include "learning_gem5/PIC/mat/mat.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/MatBank.hh"
#include "learning_gem5/PIC/AutoLoadL/AutoLoadL.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

Mat::Mat(const MatParams &params) :
    ClockedObject(params),
    requestorId(params.system->getRequestorId(this, "Mat")),
    tickEvent([this] { processTick(); }, name()),
    loadPort(params.name + ".load_l_port", this),
    cachePort(params.name + ".cache_port", this),
    maxCycles(params.max_cycles),
    startCycles(params.start_cycles),
    bankIndex(params.bank_index),
    way(params.way),
    tags(params.tags)
{
    setUp.exec = true;
    setUp.nBuf = params.n_buf;
    setUp.nCal = params.n_cal;
    setUp.accWidth = params.acc_width;
    setUp.R_base_bit = params.r_base_bit;
    setUp.R_block_row = params.r_block_row;
    setUp.L_block_row = params.l_block_row;
    setUp.L_precision = params.l_precision;
    setUp.L_vec_fetch_addr = params.l_vec_fetch_addr;
    setUp.signed_L = params.signed_l;
    setUp.signed_R_last_exist = params.signed_r_last_exist;

    fsm.setRequestVecFire([this] { return requestVec(); });
    fsm.setResponseVecFire([this] { return responseVec(); });
}

Port &
Mat::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "load_l_port")
        return loadPort;
    if (if_name == "cache_port")
        return cachePort;
    return ClockedObject::getPort(if_name, idx);
}

void
Mat::startup()
{
    schedule(tickEvent, clockEdge(startCycles));
}

// request_vec.fire: the request was granted (accepted).
bool
Mat::requestVec()
{
    if (reqFired) {
        reqFired = false;
        return true;
    }
    if (blockedReq != nullptr)
        return false;  // valid held, waiting for the grant (retry)

    RequestPtr req = std::make_shared<Request>(
        0, sizeof(AutoLoadLPayload), 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    AutoLoadLPayload payload{fsm.lVecAddr(), 0};
    pkt->setData(reinterpret_cast<const uint8_t *>(&payload));

    DPRINTF(MatBank, "bank %u way %u: request L row %llu\n", bankIndex, way,
            (unsigned long long)payload.source);
    if (loadPort.sendTimingReq(pkt))
        return true;
    blockedReq = pkt;
    return false;
}

// response_vec.fire: true once its data landed in vecBuf.
bool
Mat::responseVec()
{
    if (!respArrived)
        return false;
    respArrived = false;
    return true;
}

bool
Mat::handleLoadResp(PacketPtr pkt)
{
    const uint64_t data = pkt->getConstPtr<AutoLoadLPayload>()->data;

    // The Mat-level load interface, not the FSM, carries the payload.
    fsm.datapath().loadVec(true, data);
    respArrived = true;
    ++loads;
    DPRINTF(MatBank, "bank %u way %u: vecBuf <- %#llx\n", bankIndex, way,
            (unsigned long long)data);
    delete pkt;

    // Was idle waiting on this response -- resume stepping.
    if (!tickEvent.scheduled())
        schedule(tickEvent, nextCycle());
    return true;
}

// AutoLoadL granted this Mat: resend the held request.
void
Mat::handleLoadRetry()
{
    if (blockedReq == nullptr)
        return;
    PacketPtr pkt = blockedReq;
    blockedReq = nullptr;
    if (loadPort.sendTimingReq(pkt)) {
        reqFired = true;
        if (!tickEvent.scheduled())
            schedule(tickEvent, nextCycle());
    } else {
        blockedReq = pkt;
    }
}

// Not yet exercised -- see mat.hh's class doc comment.
bool
Mat::handleCacheResp(PacketPtr pkt)
{
    delete pkt;
    return true;
}

void
Mat::handleCacheRetry()
{
}

void
Mat::processTick()
{
    ++cycles;
    panic_if(cycles > maxCycles, "%s not finished after %u cycles",
             name(), maxCycles);

    const bool wasIdle = (fsm.state() == mat_fsm::MainState::MainIdle);
    fsm.step(setUp);
    const bool isIdleNow = (fsm.state() == mat_fsm::MainState::MainIdle);

    if (tags != nullptr && wasIdle != isIdleNow) {
        // Job started (wasIdle) or fully finished (isIdleNow) -- report
        // only; never drives setWayPICMode (see PICLLCTags).
        tags->setMatBusy(bankIndex, way, /*busy=*/!isIdleNow);
    }

    if (wasIdle && !isIdleNow) {
        setUp.exec = false;  // command consumed
    }

    if (!setUp.exec && isIdleNow) {
        done = true;
        inform("%s: bank %u way %u finished, %u L rows loaded, "
               "vecBuf=%#llx\n", name(), bankIndex, way, loads,
               (unsigned long long)fsm.datapath().vecBuf());
        exitSimLoop("Mat finished");
        return;
    }

    // Waiting on AutoLoadL (request refused or response not back yet):
    // don't self-schedule -- handleLoadResp()/handleLoadRetry() resume
    // this Mat once it has something to do. Every other cycle keeps
    // ticking on its own.
    if (blockedReq != nullptr) {
        return;
    }
    if (fsm.state() == mat_fsm::MainState::MainWaitL &&
        fsm.loadVecState() == mat_fsm::LoadVecState::WaitLResp &&
        !respArrived) {
        return;
    }
    schedule(tickEvent, nextCycle());
}

} // namespace gem5
