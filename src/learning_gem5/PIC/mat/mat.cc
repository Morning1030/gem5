#include "learning_gem5/PIC/mat/mat.hh"

#include <cstring>

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
    arrayPort(params.name + ".array_side", this),
    arrayRespEvent([this] { processArrayResp(); }, name() + ".arrayResp"),
    arrayLatency(params.array_latency),
    maxCycles(params.max_cycles),
    startCycles(params.start_cycles),
    pic(params.pic),
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
    if (if_name == "array_side")
        return arrayPort;
    return ClockedObject::getPort(if_name, idx);
}

void
Mat::startup()
{
    // A normal cache Mat has no controller, so it never runs a job.
    if (pic)
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

AddrRangeList
Mat::ArrayPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

// 8-byte read or write of one wordline of one sub-array.
// addr = arrayID_in_mat << 9 | wordline.
bool
Mat::handleArrayReq(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "%s: array access must be 8 bytes, got %s", name(),
             pkt->print());

    // RTL (BankSellPIC.scala): the PIC channel may only touch a way that is
    // in PIC mode.
    panic_if(pic && tags != nullptr && !tags->isWayInPICMode(way),
             "%s: array access while way %u is not in PIC mode", name(),
             way);

    constexpr unsigned wordlineBits = 9;
    static_assert((1u << wordlineBits) == mat_fsm::CArrayWordlineNums,
                  "wordline field must cover the whole sub-array");
    const Addr a = pkt->getAddr();
    const unsigned array = a >> wordlineBits;
    const unsigned row = a & (mat_fsm::CArrayWordlineNums - 1);
    panic_if(array >= mat_fsm::NumArrays,
             "%s: array access to sub-array %u, only %u exist", name(),
             array, mat_fsm::NumArrays);

    // RTL (Mat.scala): while a job runs the arrays take their enable, address
    // and data from the Mat's controller (arrayCacheMode is false), so the
    // external port is ignored. A write is lost; a read gets nothing useful.
    if (pic && fsm.state() != mat_fsm::MainState::MainIdle) {
        ++droppedArrayAccesses;
        DPRINTF(MatBank, "bank %u way %u: %s array %u row %u dropped, "
                "Mat is busy\n", bankIndex, way,
                pkt->isWrite() ? "write" : "read", array, row);
        if (!pkt->isWrite()) {
            const uint64_t zero = 0;
            pkt->setData(reinterpret_cast<const uint8_t *>(&zero));
        }
    } else if (pkt->isWrite()) {
        uint64_t word = 0;
        std::memcpy(&word, pkt->getConstPtr<uint8_t>(), sizeof(word));
        fsm.datapath().writeCArrayWord(array, row, word);
        DPRINTF(MatBank, "bank %u way %u: write array %u row %u = %#llx\n",
                bankIndex, way, array, row, (unsigned long long)word);
    } else {
        const uint64_t word = fsm.datapath().cArrayWord(array, row);
        pkt->setData(reinterpret_cast<const uint8_t *>(&word));
        DPRINTF(MatBank, "bank %u way %u: read array %u row %u = %#llx\n",
                bankIndex, way, array, row, (unsigned long long)word);
    }
    pkt->makeResponse();

    arrayRespQ.emplace_back(clockEdge(arrayLatency), pkt);
    if (!arrayRespBlocked && !arrayRespEvent.scheduled())
        schedule(arrayRespEvent, arrayRespQ.front().first);
    return true;
}

void
Mat::processArrayResp()
{
    while (!arrayRespBlocked && !arrayRespQ.empty() &&
           arrayRespQ.front().first <= curTick()) {
        if (!arrayPort.sendTimingResp(arrayRespQ.front().second)) {
            arrayRespBlocked = true;
            return;
        }
        arrayRespQ.pop_front();
    }
    if (!arrayRespBlocked && !arrayRespQ.empty() &&
        !arrayRespEvent.scheduled()) {
        schedule(arrayRespEvent, arrayRespQ.front().first);
    }
}

void
Mat::handleArrayRespRetry()
{
    panic_if(!arrayRespBlocked, "%s: unexpected array response retry",
             name());
    arrayRespBlocked = false;
    processArrayResp();
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
        uint64_t mSum = 0;
        for (unsigned r = 0; r < mat_fsm::MArrayWordlineNums; ++r)
            mSum += fsm.datapath().mArrayWord(r) * (r + 1);
        inform("%s: bank %u way %u finished, %u L rows loaded, "
               "vecBuf=%#llx, array checksum=%#llx, %u array accesses "
               "dropped while busy\n", name(), bankIndex, way, loads,
               (unsigned long long)fsm.datapath().vecBuf(),
               (unsigned long long)mSum, droppedArrayAccesses);
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
