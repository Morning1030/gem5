#include "mem/cache/pic/mat_sink.hh"

#include <cstring>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICBankArb.hh"

namespace gem5
{

MatSink::MatSink(const Params &p) :
    ClockedObject(p),
    respEvent([this]{ sendReady(); }, name() + ".respEvent"),
    latency(p.latency),
    refuseFirst(p.refuse_first),
    readDefault(p.read_default),
    retryDelay(p.retry_delay),
    retryEvent([this]{ sendRetries(); }, name() + ".retryEvent")
{
    for (int i = 0; i < p.port_arr_side_connection_count; ++i) {
        ports.emplace_back(p.name + csprintf(".arr_side[%d]", i), i, this);
    }
    refusedOnce.assign(ports.size(), false);
    owesRetry.assign(ports.size(), false);
}

Port &
MatSink::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "arr_side" && static_cast<size_t>(idx) < ports.size())
        return ports[idx];
    return ClockedObject::getPort(if_name, idx);
}

AddrRangeList
MatSink::ArrPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

bool
MatSink::handleRequest(PacketPtr pkt, int port)
{
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "MatSink: expected an 8-byte access, got %s", pkt->print());

    if (refuseFirst && !refusedOnce[port]) {
        refusedOnce[port] = true;
        owesRetry[port] = true;
        DPRINTF(PICBankArb, "MatSink: Mat[%d] refuses %#x, retry in %u "
                "cycles\n", port, pkt->getAddr(),
                static_cast<unsigned>(retryDelay));
        if (!retryEvent.scheduled())
            schedule(retryEvent, clockEdge(retryDelay));
        return false;
    }

    const uint64_t key = (uint64_t(port) << 32) | pkt->getAddr();
    if (pkt->isWrite()) {
        std::memcpy(&mem[key], pkt->getConstPtr<uint8_t>(), 8);
        DPRINTF(PICBankArb, "MatSink: Mat[%d] write %#x\n", port,
                pkt->getAddr());
    } else {
        const auto it = mem.find(key);
        const uint64_t v = (it == mem.end()) ? readDefault : it->second;
        pkt->setData(reinterpret_cast<const uint8_t *>(&v));
        DPRINTF(PICBankArb, "MatSink: Mat[%d] read %#x\n", port,
                pkt->getAddr());
    }
    pkt->makeResponse();

    Entry e;
    e.pkt = pkt;
    e.port = port;
    e.when = clockEdge(latency);
    queue.push_back(e);
    if (!waitingRetry && !respEvent.scheduled())
        schedule(respEvent, e.when);
    return true;
}

void
MatSink::sendReady()
{
    while (!waitingRetry && !queue.empty() &&
           queue.front().when <= curTick()) {
        Entry e = queue.front();
        queue.pop_front();
        if (!ports[e.port].sendTimingResp(e.pkt)) {
            blocked = e;
            waitingRetry = true;
            return;
        }
    }
    if (!waitingRetry && !queue.empty() && !respEvent.scheduled())
        schedule(respEvent, queue.front().when);
}

void
MatSink::sendRetries()
{
    for (size_t i = 0; i < ports.size(); ++i) {
        if (owesRetry[i]) {
            owesRetry[i] = false;
            DPRINTF(PICBankArb, "MatSink: Mat[%d] sends retry\n",
                    static_cast<int>(i));
            ports[i].sendRetryReq();
        }
    }
}

void
MatSink::respRetry()
{
    panic_if(!waitingRetry, "MatSink: unexpected response retry");
    if (ports[blocked.port].sendTimingResp(blocked.pkt)) {
        waitingRetry = false;
        sendReady();
    }
}

}  // namespace gem5
