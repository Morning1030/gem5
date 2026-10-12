#include "mem/cache/pic/tlb_req_scheduler.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICTLB.hh"

namespace gem5
{

TLBReqScheduler::TLBReqScheduler(const Params &p) :
    ClockedObject(p),
    stepEvent([this]{ step(); }, name() + ".stepEvent"),
    virtualization(p.virtualization),
    translateLatency(p.translate_latency),
    stateLatency(p.state_latency)
{
    const int n = p.port_cpu_side_connection_count;
    panic_if(n < 1 || n > 2,
             "TLBReqScheduler takes 1 or 2 cpu_side connections, got %d", n);
    for (int i = 0; i < n; ++i) {
        cpuPorts.emplace_back(p.name + csprintf(".cpu_side[%d]", i), i, this);
    }
}

Port &
TLBReqScheduler::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cpu_side" &&
               static_cast<size_t>(idx) < cpuPorts.size()) {
        return cpuPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------

AddrRangeList
TLBReqScheduler::CPUSidePort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

bool
TLBReqScheduler::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
    if (!owner->handleRequest(pkt, id)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
TLBReqScheduler::CPUSidePort::recvRespRetry()
{
    owner->respRetry(id);
}

// ---------------------------------------------------------------------------
// Request in / FSM
// ---------------------------------------------------------------------------

bool
TLBReqScheduler::handleRequest(PacketPtr pkt, int side)
{
    panic_if(!pkt->isRead() || pkt->getSize() != 8,
             "TLBReqScheduler: expected an 8-byte ReadReq on cpu_side[%d], "
             "got %s", side, pkt->print());

    // Slot full, or this side's query is being served.
    if (pending[side] || (active && activeSide == side))
        return false;

    DPRINTF(PICTLB, "TLBReqScheduler: query vaddr %#x from side %d\n",
            pkt->getAddr(), side);
    pending[side] = pkt;
    if (state == State::Idle)
        scheduleStep();
    return true;
}

void
TLBReqScheduler::scheduleStep()
{
    if (!stepEvent.scheduled())
        schedule(stepEvent, clockEdge(stateLatency));
}

void
TLBReqScheduler::step()
{
    switch (state) {
      case State::Idle:
        arbitrate();
        break;
      case State::Translating:
        paddr = translate(active->getAddr());
        DPRINTF(PICTLB, "TLBReqScheduler: vaddr %#x -> paddr %#x\n",
                active->getAddr(), paddr);
        state = State::ReturnPaddr;
        scheduleStep();
        break;
      case State::ReturnPaddr:
        sendResp();
        break;
    }
}

void
TLBReqScheduler::arbitrate()
{
    // Round-robin: start after the last winner.
    for (int i = 1; i <= 2; ++i) {
        const int s = (lastGrant + i) % 2;
        if (!pending[s])
            continue;

        active = pending[s];
        pending[s] = nullptr;
        activeSide = s;
        lastGrant = s;
        paddr = active->getAddr();   // identity if virtualization is off

        DPRINTF(PICTLB, "TLBReqScheduler: grant side %d vaddr %#x\n",
                s, paddr);

        if (virtualization) {
            state = State::Translating;
            if (!stepEvent.scheduled())
                schedule(stepEvent, clockEdge(translateLatency));
        } else {
            state = State::ReturnPaddr;
            scheduleStep();
        }
        return;
    }
}

Addr
TLBReqScheduler::translate(Addr vaddr)
{
    // Identity for now. SE mode with a Process: look up process->pTable.
    return vaddr;
}

void
TLBReqScheduler::sendResp()
{
    CPUSidePort &port = cpuPorts[activeSide];
    panic_if(port.blockedResp, "TLBReqScheduler: response already blocked");

    active->makeResponse();
    const uint64_t resp = paddr;
    active->setData(reinterpret_cast<const uint8_t *>(&resp));

    DPRINTF(PICTLB, "TLBReqScheduler: return paddr %#x to side %d\n",
            paddr, activeSide);

    // Stay busy until the requester accepts the response.
    if (!port.sendTimingResp(active)) {
        port.blockedResp = active;
        return;
    }
    finishQuery();
}

void
TLBReqScheduler::respRetry(int side)
{
    CPUSidePort &port = cpuPorts[side];
    panic_if(!port.blockedResp, "TLBReqScheduler: spurious response retry");
    PacketPtr pkt = port.blockedResp;
    port.blockedResp = nullptr;
    if (!port.sendTimingResp(pkt)) {
        port.blockedResp = pkt;
        return;
    }
    finishQuery();
}

void
TLBReqScheduler::finishQuery()
{
    active = nullptr;
    activeSide = -1;
    state = State::Idle;

    for (auto &port : cpuPorts) {
        if (port.needRetry) {
            port.needRetry = false;
            port.sendRetryReq();
        }
    }
    if (pending[0] || pending[1])
        scheduleStep();
}

}  // namespace gem5
