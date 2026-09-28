#include "learning_gem5/PIC/mat/mat_bank.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/MatBank.hh"
#include "learning_gem5/PIC/AutoLoadL/AutoLoadL.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

MatBank::MatBank(const MatBankParams &params) :
    ClockedObject(params),
    requestorId(params.system->getRequestorId(this, "MatBank")),
    tickEvent([this] { processTick(); }, name()),
    maxCycles(params.max_cycles),
    startCycles(params.start_cycles),
    bankIndex(params.bank_index),
    tags(params.tags)
{
    fatal_if(params.first_pic_way > params.num_mats,
             "first_pic_way exceeds num_mats");
    const unsigned numPic = params.num_mats - params.first_pic_way;
    fatal_if(params.port_load_l_port_connection_count != numPic,
             "%s: %u load_l_port connections, expected %u PIC Mats",
             name(), params.port_load_l_port_connection_count, numPic);

    mats.resize(numPic);
    for (unsigned i = 0; i < numPic; ++i) {
        MatCtx &m = mats[i];
        m.way = params.first_pic_way + i;
        m.setUp.exec = true;
        m.setUp.nBuf = params.n_buf;
        m.setUp.nCal = params.n_cal;
        m.setUp.accWidth = params.acc_width;
        m.setUp.R_base_bit = params.r_base_bit;
        m.setUp.R_block_row = params.r_block_row;
        m.setUp.L_block_row = params.l_block_row;
        m.setUp.L_precision = params.l_precision;
        m.setUp.L_vec_fetch_addr =
            params.l_vec_fetch_addr + m.way * params.l_vec_mat_stride;
        m.setUp.signed_L = params.signed_l;
        m.setUp.signed_R_last_exist = params.signed_r_last_exist;

        m.fsm.setRequestVecFire([this, i] { return requestVec(i); });
        m.fsm.setResponseVecFire([this, i] { return responseVec(i); });

        loadPorts.emplace_back(std::make_unique<LoadPort>(
            params.name + ".load_l_port[" + std::to_string(i) + "]",
            this, i));
    }
}

Port &
MatBank::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "load_l_port") {
        panic_if(idx >= loadPorts.size(), "bad load_l_port index %d", idx);
        return *loadPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

void
MatBank::startup()
{
    schedule(tickEvent, clockEdge(startCycles));
}

// request_vec.fire for Mat i: the request was granted (accepted).
bool
MatBank::requestVec(unsigned i)
{
    MatCtx &m = mats[i];
    if (m.reqFired) {
        m.reqFired = false;
        return true;
    }
    if (m.blockedReq != nullptr)
        return false;  // valid held, waiting for the grant (retry)

    RequestPtr req = std::make_shared<Request>(
        0, sizeof(AutoLoadLPayload), 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    AutoLoadLPayload payload{m.fsm.lVecAddr(), 0};
    pkt->setData(reinterpret_cast<const uint8_t *>(&payload));

    DPRINTF(MatBank, "way %u: request L row %llu\n", m.way,
            (unsigned long long)payload.source);
    if (loadPorts[i]->sendTimingReq(pkt))
        return true;
    m.blockedReq = pkt;
    return false;
}

// response_vec.fire for Mat i: true once its data landed in vecBuf.
bool
MatBank::responseVec(unsigned i)
{
    MatCtx &m = mats[i];
    if (!m.respArrived)
        return false;
    m.respArrived = false;
    return true;
}

bool
MatBank::handleLoadResp(unsigned i, PacketPtr pkt)
{
    MatCtx &m = mats[i];
    const uint64_t data = pkt->getConstPtr<AutoLoadLPayload>()->data;

    // The Mat-level load interface, not the FSM, carries the payload.
    m.fsm.datapath().loadVec(true, data);
    m.respArrived = true;
    ++m.loads;
    DPRINTF(MatBank, "way %u: vecBuf <- %#llx\n", m.way,
            (unsigned long long)data);
    delete pkt;
    return true;
}

// AutoLoadL granted this Mat: resend the held request.
void
MatBank::handleLoadRetry(unsigned i)
{
    MatCtx &m = mats[i];
    if (m.blockedReq == nullptr)
        return;
    PacketPtr pkt = m.blockedReq;
    m.blockedReq = nullptr;
    if (loadPorts[i]->sendTimingReq(pkt))
        m.reqFired = true;
    else
        m.blockedReq = pkt;
}

void
MatBank::processTick()
{
    ++cycles;
    panic_if(cycles > maxCycles, "MatBank %s not finished after %u cycles",
             name(), maxCycles);

    bool allDone = true;
    for (unsigned i = 0; i < mats.size(); ++i) {
        MatCtx &m = mats[i];
        if (m.done)
            continue;

        const bool wasIdle = (m.fsm.state() == mat_fsm::MainState::MainIdle);
        m.fsm.step(m.setUp);
        const bool isIdleNow = (m.fsm.state() == mat_fsm::MainState::MainIdle);

        if (tags != nullptr && wasIdle != isIdleNow) {
            // Job started (wasIdle) or fully finished (isIdleNow) --
            // report only; never drives setWayPICMode (see PICLLCTags).
            tags->setMatBusy(bankIndex, m.way, /*busy=*/!isIdleNow);
        }

        if (wasIdle && !isIdleNow) {
            m.setUp.exec = false;  // command consumed
        }
        if (!m.setUp.exec &&
            m.fsm.state() == mat_fsm::MainState::MainIdle) {
            m.done = true;
            inform("%s: way %u finished, %u L rows loaded, vecBuf=%#llx\n",
                   name(), m.way, m.loads,
                   (unsigned long long)m.fsm.datapath().vecBuf());
        } else {
            allDone = false;
        }
    }

    if (allDone) {
        exitSimLoop("MatBank finished");
        return;
    }
    schedule(tickEvent, nextCycle());
}

} // namespace gem5
