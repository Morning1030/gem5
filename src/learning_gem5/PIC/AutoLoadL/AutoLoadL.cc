#include "learning_gem5/PIC/AutoLoadL/AutoLoadL.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/AutoLoadL.hh"
#include "sim/system.hh"

namespace gem5
{

AutoLoadL::AutoLoadL(const AutoLoadLParams &params) :
    ClockedObject(params),
    cacheBankPort(params.name + ".cb_port", this),
    requestorId(params.system->getRequestorId(this, "AutoLoadL")),
    pending(params.port_inst_port_connection_count, false),
    lastGranted(params.port_inst_port_connection_count - 1),
    // Lowest priority at its tick, so every Mat that requests in the same
    // cycle is already pending when the arbiter picks.
    arbitrateEvent([this] { arbitrate(); }, name() + ".arbitrate", false,
                   Event::Progress_Event_Pri),
    readMemEvent([this] { processReadMem(); }, name() + ".readMem"),
    writeLEvent([this] { processWriteL(); }, name() + ".writeL"),
    reportEvent([this] { processReport(); }, name() + ".report")
{
    for (PortID i = 0; i < params.port_inst_port_connection_count; ++i) {
        instPorts.emplace_back(std::make_unique<CPUSidePort>(
            params.name + ".inst_port[" + std::to_string(i) + "]", this, i));
    }
}

Port &
AutoLoadL::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "inst_port") {
        panic_if(idx >= instPorts.size(), "bad inst_port index %d", idx);
        return *instPorts[idx];
    }
    if (if_name == "cb_port")
        return cacheBankPort;
    return ClockedObject::getPort(if_name, idx);
}

// A Mat's request_vec.valid. Accepted only when this Mat has been granted;
// otherwise the Mat keeps its request and we just remember it is waiting.
bool
AutoLoadL::handleRequest(PacketPtr pkt, PortID idx)
{
    if (state == State::Idle && reservedPort == idx) {
        reservedPort = InvalidPortID;
        lastGranted = idx;
        curPort = idx;
        curPkt = pkt;
        state = State::ReadMem;
        DPRINTF(AutoLoadL, "idle -> read_mem: granted mat port %d\n", idx);
        schedule(readMemEvent, nextCycle());
        return true;
    }

    pending[idx] = true;
    if (state == State::Idle && reservedPort == InvalidPortID &&
        !arbitrateEvent.scheduled()) {
        schedule(arbitrateEvent, curTick());
    }
    return false;
}

// Round-robin: next waiting Mat after the last granted one.
void
AutoLoadL::arbitrate()
{
    if (state != State::Idle || reservedPort != InvalidPortID)
        return;

    const PortID n = instPorts.size();
    for (PortID j = 1; j <= n; ++j) {
        const PortID idx = (lastGranted + j) % n;
        if (pending[idx]) {
            pending[idx] = false;
            reservedPort = idx;
            DPRINTF(AutoLoadL, "arbiter grants mat port %d\n", idx);
            instPorts[idx]->sendRetryReq();  // Mat resends -> accepted
            return;
        }
    }
}

// read_mem: wait for the cache bank / arbiter to accept the read.
void
AutoLoadL::processReadMem()
{
    const AutoLoadLPayload *req = curPkt->getConstPtr<AutoLoadLPayload>();
    // ASSUMPTION: `source` is a row index, one 64-bit vec_buf word per row.
    RequestPtr request = std::make_shared<Request>(
        req->source * sizeof(uint64_t), sizeof(uint64_t), 0, requestorId);
    PacketPtr rd = new Packet(request, MemCmd::ReadReq);
    rd->allocate();

    if (cacheBankPort.sendTimingReq(rd)) {
        state = State::RevVec;
        DPRINTF(AutoLoadL, "read_mem -> rev_vec\n");
    } else {
        blockedReadReq = rd;
    }
}

void
AutoLoadL::handleReqRetry()
{
    if (blockedReadReq == nullptr)
        return;
    PacketPtr rd = blockedReadReq;
    if (cacheBankPort.sendTimingReq(rd)) {
        blockedReadReq = nullptr;
        state = State::RevVec;
        DPRINTF(AutoLoadL, "read_mem -> rev_vec (after retry)\n");
    }
}

// rev_vec: the row arrived; latch it.
bool
AutoLoadL::handleResponse(PacketPtr pkt)
{
    panic_if(state != State::RevVec, "bank response outside rev_vec");
    dataReadFromBank = *pkt->getConstPtr<uint64_t>();
    delete pkt;
    state = State::WriteL;
    DPRINTF(AutoLoadL, "rev_vec -> write_L: data %#llx\n",
            (unsigned long long)dataReadFromBank);
    schedule(writeLEvent, nextCycle());
    return true;
}

// write_L: one cycle driving the Mat's vec_buf load. The data travels with
// the response (the Mat-level interface calls loadVec on receipt).
void
AutoLoadL::processWriteL()
{
    state = State::ReportFinish;
    DPRINTF(AutoLoadL, "write_L -> report_finish\n");
    schedule(reportEvent, nextCycle());
}

// report_finish: pulse the response to the granted Mat.
void
AutoLoadL::processReport()
{
    PacketPtr pkt = curPkt;
    AutoLoadLPayload payload = *pkt->getConstPtr<AutoLoadLPayload>();
    payload.data = dataReadFromBank;
    pkt->makeResponse();
    pkt->setData(reinterpret_cast<const uint8_t *>(&payload));

    if (instPorts[curPort]->sendTimingResp(pkt))
        finishFetch();
    else
        blockedResp = pkt;
}

void
AutoLoadL::handleRespRetry()
{
    if (blockedResp == nullptr)
        return;
    PacketPtr pkt = blockedResp;
    if (instPorts[curPort]->sendTimingResp(pkt)) {
        blockedResp = nullptr;
        finishFetch();
    }
}

void
AutoLoadL::finishFetch()
{
    DPRINTF(AutoLoadL, "report_finish -> idle\n");
    curPkt = nullptr;
    curPort = InvalidPortID;
    state = State::Idle;
    for (bool p : pending) {
        if (p) {
            if (!arbitrateEvent.scheduled())
                schedule(arbitrateEvent, curTick());
            break;
        }
    }
}

} // namespace gem5
