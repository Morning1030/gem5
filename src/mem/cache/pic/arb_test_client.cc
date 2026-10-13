#include "mem/cache/pic/arb_test_client.hh"

#include <cstring>
#include <sstream>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICBankArb.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

ArbTestClient::ArbTestClient(const Params &p) :
    ClockedObject(p),
    port(p.name + ".port", this),
    sendEvent([this]{ sendNext(); }, name() + ".sendEvent"),
    requestorId(p.system->getRequestorId(this)),
    isP2S(p.kind == "p2s"),
    isRaw(p.kind == "raw"),
    exitWhenDone(p.exit_when_done),
    startDelay(p.start_delay)
{
    fatal_if(p.kind != "p2s" && p.kind != "autoload" && p.kind != "raw",
             "ArbTestClient: kind must be 'p2s', 'autoload' or 'raw', "
             "got '%s'", p.kind);
    for (const std::string &line : p.ops) {
        std::istringstream in(line);
        std::string type, addr, data;
        in >> type >> addr >> data;
        fatal_if(type != "W" && type != "R",
                 "ArbTestClient: bad op '%s'", line);
        fatal_if(type == "W" && !isP2S && !isRaw,
                 "ArbTestClient: kind 'autoload' cannot write ('%s')", line);
        fatal_if(type == "R" && isP2S,
                 "ArbTestClient: kind 'p2s' cannot read ('%s')", line);
        const bool any = (data == "*");
        ops.push_back({type == "W", std::stoull(addr, nullptr, 0),
                       any ? 0 : std::stoull(data, nullptr, 0), any});
    }
}

Port &
ArbTestClient::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    return ClockedObject::getPort(if_name, idx);
}

void
ArbTestClient::startup()
{
    schedule(sendEvent, clockEdge(startDelay));
}

void
ArbTestClient::sendNext()
{
    if (next >= ops.size()) {
        inform("%s: %u ops done, %u reads checked", name(),
               static_cast<unsigned>(ops.size()),
               readsChecked);
        if (exitWhenDone)
            exitSimLoop("ArbTestClient done");
        return;
    }

    const Op &op = ops[next];
    RequestPtr req = std::make_shared<Request>(
        op.addr, isP2S ? sizeof(P2SWritePayload) : sizeof(uint64_t), 0,
        requestorId);
    PacketPtr pkt = new Packet(req, op.write ? MemCmd::WriteReq
                                             : MemCmd::ReadReq);
    pkt->allocate();
    if (op.write && isP2S) {
        P2SWritePayload payload{op.addr, op.data};
        pkt->setData(reinterpret_cast<const uint8_t *>(&payload));
    } else if (op.write) {
        pkt->setData(reinterpret_cast<const uint8_t *>(&op.data));
    }

    DPRINTF(PICBankArb, "%s: op %u %s addr %#x data %#x\n", name(),
            static_cast<unsigned>(next),
            op.write ? "W" : "R", op.addr, op.data);
    if (!port.sendTimingReq(pkt))
        held = pkt;
}

void
ArbTestClient::retry()
{
    panic_if(!held, "%s: unexpected request retry", name());
    PacketPtr pkt = held;
    if (port.sendTimingReq(pkt))
        held = nullptr;
}

bool
ArbTestClient::handleResp(PacketPtr pkt)
{
    const Op &op = ops[next];
    if (!op.write) {
        uint64_t got = 0;
        std::memcpy(&got, pkt->getConstPtr<uint8_t>(), sizeof(got));
        if (op.anyValue) {
            inform("%s: read %#x = %#x", name(), op.addr, got);
        }
        panic_if(!op.anyValue && got != op.data,
                 "%s: read of %#x returned %#x, expected %#x", name(),
                 op.addr, got, op.data);
        ++readsChecked;
    }
    delete pkt;
    ++next;
    schedule(sendEvent, nextCycle());
    return true;
}

}  // namespace gem5
