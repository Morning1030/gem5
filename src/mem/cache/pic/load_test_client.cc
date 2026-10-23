#include "mem/cache/pic/load_test_client.hh"

#include <sstream>

#include "base/logging.hh"
#include "learning_gem5/PIC/Control/scheduler.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

// gem5 only (test): parse the LOAD list from the config.
LoadTestClient::LoadTestClient(const Params &p) :
    ClockedObject(p),
    port(p.name + ".port", this),
    sendEvent([this]{ sendNext(); }, name() + ".sendEvent"),
    requestorId(p.system->getRequestorId(this)),
    exitWhenDone(p.exit_when_done),
    startDelay(p.start_delay)
{
    for (const std::string &line : p.loads) {
        std::istringstream in(line);
        std::string src, dst, row, bpr, off;
        in >> src >> dst >> row >> bpr >> off;
        fatal_if(off.empty(), "LoadTestClient: bad load '%s'", line);
        loads.push_back({std::stoull(src, nullptr, 0),
                         std::stoull(dst, nullptr, 0),
                         static_cast<uint16_t>(std::stoul(row, nullptr, 0)),
                         static_cast<uint16_t>(std::stoul(bpr, nullptr, 0)),
                         static_cast<uint16_t>(std::stoul(off, nullptr, 0))});
    }
}

Port &
LoadTestClient::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    return ClockedObject::getPort(if_name, idx);
}

void
LoadTestClient::startup()
{
    schedule(sendEvent, clockEdge(startDelay));
}

// Scala: CMD_resolver_new.enq_LOAD -> io.load_P_req. Send the next LOAD
// as the Scheduler would (LSPayload: src, dst, row, byte_per_row,
// offset).
void
LoadTestClient::sendNext()
{
    if (next >= loads.size()) {
        inform("%s: %d loads done", name(), loads.size());
        if (exitWhenDone)
            exitSimLoop("LoadTestClient done");
        return;
    }

    const Load &l = loads[next];
    LSPayload payload{l.src, l.dst, l.row, l.bytePerRow, l.offset};

    RequestPtr req = std::make_shared<Request>(0, sizeof(payload), 0,
                                               requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    pkt->setData(reinterpret_cast<const uint8_t *>(&payload));

    sentAt = curTick();
    if (!port.sendTimingReq(pkt))
        held = pkt;
}

// gem5 only: resend the LOAD Load_ctl refused.
void
LoadTestClient::retry()
{
    panic_if(!held, "%s: unexpected retry", name());
    if (port.sendTimingReq(held))
        held = nullptr;
}

// Scala: busy_load falling -- the LOAD is done; send the next one.
bool
LoadTestClient::handleResp(PacketPtr pkt)
{
    const Load &l = loads[next];
    inform("%s: load %d done (src %#x dst %#x %u x %u B stride %u) "
           "in %llu ticks", name(), next, l.src, l.dst, l.row,
           l.bytePerRow, l.offset,
           static_cast<unsigned long long>(curTick() - sentAt));
    delete pkt;
    ++next;
    schedule(sendEvent, clockEdge(Cycles(1)));
    return true;
}

}  // namespace gem5
