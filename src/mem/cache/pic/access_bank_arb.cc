#include "mem/cache/pic/access_bank_arb.hh"

#include <cstring>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICBankArb.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{
// Array-address field layout, confirmed against the real RTL --
// SysConfig.scala:282-305's get_bankID()/get_in_bank_matID(), evaluated on
// the accessCacheFullAddrLen(=17-bit) branch (the one that applies here,
// since ReqPackage.addr is always a full 17-bit address at this point --
// see AccessBank_Arb.scala:81-82). With the paper's default config
// (core_addrLen=11, numBanks=4, matPerBank=16, accessCacheFullAddrLen=17):
//
//   bit:    16 15 14 13 | 12 11 |  10  9  |  8 7 6 5 4 3 2 1 0
//   field: [ matID_in_bank ][bankID][arrayID_in_mat][ wordline_offset ]
//   width:        4            2          2                 9
constexpr unsigned kOffsetBits       = 9;  // wordline_offset, bits[8:0]
constexpr unsigned kArrayIdInMatBits = 2;  // arrayID_in_mat, bits[10:9]
constexpr uint32_t kWithinMatMask =
    (1u << (kOffsetBits + kArrayIdInMatBits)) - 1;

/** Sender state on a packet sent to a Mat: whose request it is. */
class ArbMatSenderState : public Packet::SenderState
{
  public:
    ArbMatSenderState(unsigned _clientId, PacketPtr _clientPkt)
        : clientId(_clientId), clientPkt(_clientPkt)
    {}

    const unsigned clientId;
    /** The client's own packet, answered when the Mat answers a read. */
    const PacketPtr clientPkt;
};

const char *
clientName(unsigned clientId)
{
    static const char *names[kNumBankArbClients] = {
        "LoadPostProcess", "BankFetch", "ACC", "AutoLoadL",
        "P2S_L", "P2S_R", "P2S_R_T"};
    return clientId < kNumBankArbClients ? names[clientId] : "?";
}
} // namespace

AccessBankArb::AccessBankArb(const Params &p)
    : ClockedObject(p),
      bankReadyVec(p.num_banks, true),
      requestorId(p.system->getRequestorId(this)),
      matsPerBank(p.mats_per_bank),
      firstPicMat(p.first_pic_mat),
      bankBits(floorLog2(p.num_banks)),
      // Runs after the clients' same-cycle requests are queued.
      tickEvent([this]{ processTickEvent(); }, name() + ".tickEvent", false,
                Event::Progress_Event_Pri)
{
    fatal_if(p.num_banks == 0 || !isPowerOf2(p.num_banks),
             "AccessBankArb: num_banks must be a non-zero power of 2");
    fatal_if(!isPowerOf2(p.mats_per_bank),
             "AccessBankArb: mats_per_bank must be a power of 2");
    fatal_if(p.first_pic_mat >= p.mats_per_bank,
             "AccessBankArb: first_pic_mat must be < mats_per_bank");

    // Default decode, matching get_bankID() (SysConfig.scala:282-305):
    // addr[12:11] for the paper's default 4-bank config.
    const unsigned num_banks = p.num_banks;
    arrayAddrToBankFn = [num_banks](uint32_t addr) {
        constexpr unsigned shift = kOffsetBits + kArrayIdInMatBits; // = 11
        return static_cast<unsigned>((addr >> shift) & (num_banks - 1));
    };

    // p2s_side's connection index doubles as (BankArbClient - kClientP2S_L).
    fatal_if(p.port_p2s_side_connection_count != 3,
             "AccessBankArb: p2s_side must have exactly 3 connections "
             "(P2S_L, P2S_R, P2S_R_T), got %d",
             p.port_p2s_side_connection_count);

    // One port per client, indexed by BankArbClient.
    clientPorts.reserve(kNumBankArbClients);
    clientPorts.emplace_back(p.name + ".load_post_side",
                             kClientLoadPostProcess, this);
    clientPorts.emplace_back(p.name + ".bank_fetch_side",
                             kClientBankFetch, this);
    clientPorts.emplace_back(p.name + ".acc_side", kClientAccumulator, this);
    clientPorts.emplace_back(p.name + ".autoload_side",
                             kClientAutoLoadVec, this);
    for (unsigned i = 0; i < 3; ++i) {
        clientPorts.emplace_back(p.name + csprintf(".p2s_side[%d]", i),
                                 kClientP2S_L + i, this);
    }

    const unsigned numMats =
        p.num_banks * (p.mats_per_bank - p.first_pic_mat);
    const int n = p.port_mat_side_connection_count;
    fatal_if(n != 0 && static_cast<unsigned>(n) != numMats,
             "AccessBankArb: mat_side needs 0 (stub mode) or %u "
             "connections, got %d", numMats, n);
    for (int i = 0; i < n; ++i) {
        matPorts.emplace_back(
            p.name + csprintf(".mat_side[%d]", i), i, this);
    }
}

Port &
AccessBankArb::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "p2s_side" && idx >= 0 && idx < 3)
        return clientPorts[kClientP2S_L + idx];
    if (if_name == "load_post_side")
        return clientPorts[kClientLoadPostProcess];
    if (if_name == "bank_fetch_side")
        return clientPorts[kClientBankFetch];
    if (if_name == "acc_side")
        return clientPorts[kClientAccumulator];
    if (if_name == "autoload_side")
        return clientPorts[kClientAutoLoadVec];
    if (if_name == "mat_side" &&
        static_cast<size_t>(idx) < matPorts.size()) {
        return matPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// ClientPort
// ---------------------------------------------------------------------------

// gem5 only: Scala io.request(clientId).ready is always true here -- every
// request is queued on arrival.
bool
AccessBankArb::ClientPort::recvTimingReq(PacketPtr pkt)
{
    return owner->enqueue(pkt, clientId);
}

// gem5 only: response back-pressure. Keep responses in order behind any
// this client refused.
void
AccessBankArb::ClientPort::sendPacket(PacketPtr pkt)
{
    if (!respQueue.empty() || !sendTimingResp(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: %s refused a response, "
                "holding it\n", clientName(clientId));
        respQueue.push_back(pkt);
    }
}

void
AccessBankArb::ClientPort::recvRespRetry()
{
    panic_if(respQueue.empty(), "AccessBankArb: response retry from %s "
             "with nothing held", clientName(clientId));
    while (!respQueue.empty() && sendTimingResp(respQueue.front()))
        respQueue.pop_front();
}

AddrRangeList
AccessBankArb::ClientPort::getAddrRanges() const
{
    // A private point-to-point link, not decoded off a shared bus.
    return {AddrRange(0, MaxAddr)};
}

// ---------------------------------------------------------------------------
// Request queue
// ---------------------------------------------------------------------------

// gem5 only: the client packet formats. P2S and ReadPostProcess send a
// P2SWritePayload {arrayAddr, bitSlice}; AutoLoadL, BankFetch and ACC send
// an 8-byte access whose packet address is the 17-bit array address.
ReqPackage
AccessBankArb::toReqPackage(unsigned clientId, PacketPtr pkt) const
{
    ReqPackage req;
    switch (clientId) {
      case kClientLoadPostProcess:
      case kClientP2S_L:
      case kClientP2S_R:
      case kClientP2S_R_T: {
        panic_if(!pkt->isWrite(), "AccessBankArb: %s sent %s, only writes "
                 "are allowed", clientName(clientId), pkt->print());
        panic_if(pkt->getSize() != sizeof(P2SWritePayload),
                 "AccessBankArb: %s sent a %u-byte write payload, "
                 "expected %d", clientName(clientId), pkt->getSize(),
                 sizeof(P2SWritePayload));
        const auto *payload = pkt->getConstPtr<P2SWritePayload>();
        req.addr = static_cast<uint32_t>(payload->arrayAddr);
        req.optype = AccessArrayType::WRITE;
        req.dataWrittenToBank = payload->bitSlice;
        break;
      }
      case kClientAutoLoadVec:
      case kClientBankFetch:
      case kClientAccumulator:
        panic_if(pkt->getSize() != sizeof(uint64_t),
                 "AccessBankArb: %s sent a %u-byte request, expected %d",
                 clientName(clientId), pkt->getSize(), sizeof(uint64_t));
        panic_if(clientId != kClientAccumulator && !pkt->isRead(),
                 "AccessBankArb: %s sent %s, only reads are allowed",
                 clientName(clientId), pkt->print());
        req.addr = static_cast<uint32_t>(pkt->getAddr());
        if (pkt->isRead()) {
            req.optype = AccessArrayType::READ;
        } else {
            panic_if(!pkt->isWrite(), "AccessBankArb: %s sent %s",
                     clientName(clientId), pkt->print());
            req.optype = AccessArrayType::WRITE;
            req.dataWrittenToBank = pkt->getLE<uint64_t>();
        }
        break;
      default:
        panic("AccessBankArb: unknown client %u", clientId);
    }
    return req;
}

// gem5 only: replaces the RTL RRArbiter input side. The request joins the
// one FIFO in arrival order; the client never has to retry.
bool
AccessBankArb::enqueue(PacketPtr pkt, unsigned clientId)
{
    const ReqPackage req = toReqPackage(clientId, pkt);
    reqQueue.push_back({clientId, pkt, req});
    if (reqQueue.size() > maxQueueDepth)
        maxQueueDepth = reqQueue.size();

    DPRINTF(PICBankArb, "AccessBankArb: queued %s %s addr=%#x "
            "(queue %d, max %d)\n", clientName(clientId),
            req.optype == AccessArrayType::READ ? "read" : "write",
            req.addr, reqQueue.size(), maxQueueDepth);

    scheduleTick();
    return true;
}

void
AccessBankArb::scheduleTick()
{
    if (!tickEvent.scheduled()) {
        // At most one head per cycle: if this cycle's tick already ran,
        // the next one is on the next edge.
        const Tick when = clockEdge();
        schedule(tickEvent, lastTick && when <= *lastTick ?
                 clockEdge(Cycles(1)) : when);
    }
}

// Scala: AccessBank_Arb, states access/block (bankAccessScheduler.scala
// :95-125) -- the access goes out only when its bank's mem_ready is set; a
// blocked access holds everything behind it. gem5 only: the access is the
// FIFO head instead of an RRArbiter grant.
void
AccessBankArb::processTickEvent()
{
    lastTick = curTick();
    if (reqQueue.empty())
        return;

    const QueuedReq &head = reqQueue.front();
    const unsigned bankId = arrayAddrToBankFn(head.req.addr);

    if (!bankReadyVec[bankId]) {
        // Scala: state block, io.mem_ready(bankID) low. Poll every cycle;
        // CacheController does not notify when the bank frees up.
        DPRINTF(PICBankArb, "AccessBankArb: bank %u busy, %s waits at the "
                "head\n", bankId, clientName(head.clientId));
        if (!tickEvent.scheduled())
            schedule(tickEvent, clockEdge(Cycles(1)));
        return;
    }

    if (heldMatPkt) {
        // The Mat refused the head earlier; matRetry() reschedules us.
        if (!heldRetryReady)
            return;
        heldRetryReady = false;
        if (!matPorts[heldMatIdx].sendTimingReq(heldMatPkt))
            return;
        DPRINTF(PICBankArb, "AccessBankArb: Mat[%u] took the held %s "
                "access\n", heldMatIdx, clientName(head.clientId));
        heldMatPkt = nullptr;
    } else if (!issueHead(head, bankId)) {
        return;
    }

    // The head reached its Mat: a write is ACKed now, a read is answered
    // when the Mat's data comes back (handleMatResponse). Pop first: the
    // client may send its next request from inside the ACK.
    const QueuedReq done = head;
    reqQueue.pop_front();
    if (done.req.optype == AccessArrayType::WRITE || matPorts.empty())
        respond(done.clientId, done.pkt, nullptr);

    // A request queued during the ACK has already scheduled the next tick.
    if (!reqQueue.empty() && !tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

bool
AccessBankArb::issueHead(const QueuedReq &head, unsigned bankId)
{
    // Stub mode (no Mats connected): nothing to write or read.
    if (matPorts.empty())
        return true;

    const unsigned idx = matIndex(head.req.addr, bankId);
    panic_if(idx >= matPorts.size(), "AccessBankArb: Mat index %u out of "
             "range (%d Mats)", idx, matPorts.size());

    const bool isRead = (head.req.optype == AccessArrayType::READ);
    RequestPtr r = std::make_shared<Request>(
        head.req.addr & kWithinMatMask, sizeof(uint64_t), 0, requestorId);
    PacketPtr pkt = new Packet(r, isRead ? MemCmd::ReadReq
                                         : MemCmd::WriteReq);
    pkt->allocate();
    if (!isRead) {
        pkt->setData(
            reinterpret_cast<const uint8_t *>(&head.req.dataWrittenToBank));
    }
    pkt->pushSenderState(new ArbMatSenderState(head.clientId, head.pkt));

    DPRINTF(PICBankArb, "AccessBankArb: %s -> Mat[%u] %s\n",
            clientName(head.clientId), idx, pkt->print());

    if (!matPorts[idx].sendTimingReq(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: Mat[%u] refused, the head "
                "waits for its retry\n", idx);
        heldMatPkt = pkt;
        heldMatIdx = idx;
        heldRetryReady = false;
        return false;
    }
    return true;
}

void
AccessBankArb::respond(unsigned clientId, PacketPtr pkt,
                       const uint8_t *readData)
{
    if (pkt->isRead()) {
        // Stub mode answers reads with zeros.
        const uint64_t zero = 0;
        pkt->setData(readData ? readData
                              : reinterpret_cast<const uint8_t *>(&zero));
    }
    pkt->makeResponse();
    DPRINTF(PICBankArb, "AccessBankArb: %s to %s\n",
            pkt->isRead() ? "read data" : "write ack", clientName(clientId));
    clientPorts[clientId].sendPacket(pkt);
}

// ---------------------------------------------------------------------------
// Mat side
// ---------------------------------------------------------------------------

unsigned
AccessBankArb::matIndex(uint32_t addr, unsigned bankId) const
{
    const unsigned matId = (addr >> (kOffsetBits + kArrayIdInMatBits +
                                     bankBits)) & (matsPerBank - 1);
    panic_if(matId < firstPicMat,
             "AccessBankArb: addr %#x targets matID %u, a normal cache Mat; "
             "the PIC channel cannot access it (first PIC matID is %u)",
             addr, matId, firstPicMat);
    return (matId - firstPicMat) * bankReadyVec.size() + bankId;
}

bool
AccessBankArb::handleMatResponse(PacketPtr pkt)
{
    auto *ss = safe_cast<ArbMatSenderState *>(pkt->popSenderState());
    if (pkt->isRead())
        respond(ss->clientId, ss->clientPkt, pkt->getConstPtr<uint8_t>());
    // A write was already ACKed when the Mat accepted it.
    delete ss;
    delete pkt;
    return true;
}

void
AccessBankArb::matRetry(int idx)
{
    panic_if(!heldMatPkt || heldMatIdx != static_cast<unsigned>(idx),
             "AccessBankArb: unexpected retry from Mat[%d]", idx);
    heldRetryReady = true;
    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));
}

bool
AccessBankArb::MatSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner->handleMatResponse(pkt);
}

void
AccessBankArb::MatSidePort::recvReqRetry()
{
    owner->matRetry(id);
}

void
AccessBankArb::setBankReady(unsigned bankId, bool ready)
{
    fatal_if(bankId >= bankReadyVec.size(),
             "AccessBankArb::setBankReady: bank %u out of range (%d "
             "banks)", bankId, bankReadyVec.size());
    bankReadyVec[bankId] = ready;
}

bool
AccessBankArb::bankReady(unsigned bankId) const
{
    fatal_if(bankId >= bankReadyVec.size(),
             "AccessBankArb::bankReady: bank %u out of range (%d banks)",
             bankId, bankReadyVec.size());
    return bankReadyVec[bankId];
}

void
AccessBankArb::setArrayAddrToBank(std::function<unsigned(uint32_t)> fn)
{
    arrayAddrToBankFn = std::move(fn);
}

} // namespace gem5
