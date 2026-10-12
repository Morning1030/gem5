#include "mem/cache/pic/access_bank_arb.hh"

#include <algorithm>
#include <cstring>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PICBankArb.hh"
#include "sim/system.hh"

namespace gem5
{

namespace
{
// mod: p2s_arbiter
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
//
// MSB -> LSB: matID_in_bank(4), bankID(2), arrayID_in_mat(2),
// wordline_offset(9) = 4+2+2+9 = 17 bits. Bank sits *below*
// matID_in_bank, not at the top of the address -- get_bankID() only ever
// needs bankID = addr[12:11], so that's the only field extracted below;
// matID_in_bank/arrayID_in_mat/offset are irrelevant to this arbiter (they
// only matter once a request is already inside the right bank).
constexpr unsigned kOffsetBits       = 9;  // wordline_offset, bits[8:0]
constexpr unsigned kArrayIdInMatBits = 2;  // arrayID_in_mat, bits[10:9]
constexpr uint32_t kWithinMatMask =
    (1u << (kOffsetBits + kArrayIdInMatBits)) - 1;

/** Sender state on a packet sent to a Mat. */
class ArbMatSenderState : public Packet::SenderState
{
  public:
    ArbMatSenderState(unsigned _clientId, bool _isRead)
        : clientId(_clientId), isRead(_isRead)
    {}

    const unsigned clientId;
    const bool isRead;
};
} // namespace

AccessBankArb::AccessBankArb(const Params &p)
    : ClockedObject(p),
      numClients(kNumBankArbClients),
      reqValid(kNumBankArbClients, false),
      reqPkg(kNumBankArbClients),
      // Start so the first tick() scans from client 0, matching an
      // RRArbiter that resets its rotating-priority pointer to 0.
      lastGranted(kNumBankArbClients - 1),
      state(ArbState::Access),
      blockedClientId(0),
      blockedBankId(0),
      bankReadyVec(p.num_banks, true),
      autoLoadPort(p.name + ".autoload_side", this),
      tickEvent([this]{ processTickEvent(); }, name() + ".tickEvent"),
      requestorId(p.system->getRequestorId(this)),
      matsPerBank(p.mats_per_bank),
      firstPicMat(p.first_pic_mat),
      bankBits(floorLog2(p.num_banks)),
      heldMat(p.num_banks)
{
    fatal_if(p.num_banks == 0 || !isPowerOf2(p.num_banks),
             "AccessBankArb: num_banks must be a non-zero power of 2");
    fatal_if(!isPowerOf2(p.mats_per_bank),
             "AccessBankArb: mats_per_bank must be a power of 2");
    fatal_if(p.first_pic_mat >= p.mats_per_bank,
             "AccessBankArb: first_pic_mat must be < mats_per_bank");

    // mod: p2s_arbiter
    // Default decode, matching get_bankID() (SysConfig.scala:282-305):
    // bankID sits right above arrayID_in_mat/wordline_offset and right
    // below matID_in_bank -- addr[12:11] for the paper's default 4-bank
    // config (see the field-layout note above). This is no longer a
    // placeholder for that default config; setArrayAddrToBank() remains
    // the override point if num_banks/matPerBank/core_addrLen differ.
    const unsigned num_banks = p.num_banks;
    arrayAddrToBankFn = [num_banks](uint32_t addr) {
        constexpr unsigned shift = kOffsetBits + kArrayIdInMatBits; // = 11
        return static_cast<unsigned>((addr >> shift) & (num_banks - 1));
    };

    // mod: p2s_arbiter
    // Exactly one connection per P2S engine -- index doubles as
    // (BankArbClient - kClientP2S_L), see the file header comment.
    fatal_if(p.port_p2s_side_connection_count != 3,
             "AccessBankArb: p2s_side must have exactly 3 connections "
             "(P2S_L, P2S_R, P2S_R_T), got %d",
             p.port_p2s_side_connection_count);
    for (int i = 0; i < p.port_p2s_side_connection_count; ++i) {
        p2sPorts.emplace_back(
            p.name + csprintf(".p2s_side[%d]", i), i, this);
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
    if (if_name == "p2s_side" &&
        static_cast<size_t>(idx) < p2sPorts.size()) {
        return p2sPorts[idx];
    }
    if (if_name == "autoload_side") {
        return autoLoadPort;
    }
    if (if_name == "mat_side" &&
        static_cast<size_t>(idx) < matPorts.size()) {
        return matPorts[idx];
    }
    return ClockedObject::getPort(if_name, idx);
}

// ---------------------------------------------------------------------------
// P2SPort
// ---------------------------------------------------------------------------

void
AccessBankArb::P2SPort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    DPRINTF(PICBankArb, "AccessBankArb: sending ack %s to P2S[%d]\n",
            pkt->print(), id);
    if (!sendTimingResp(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: P2S[%d] refused ack, will retry\n",
                id);
        blockedPacket = pkt;
    }
}

AddrRangeList
AccessBankArb::P2SPort::getAddrRanges() const
{
    // Private point-to-point link to one P2S engine, not decoded off a
    // shared bus -- claim the whole address space like other such links
    // in this tree (e.g. DMAEngine::P2SPort).
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

void
AccessBankArb::P2SPort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(PICBankArb, "AccessBankArb: sending retry req to P2S[%d]\n",
                id);
        sendRetryReq();
    }
}

bool
AccessBankArb::P2SPort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(PICBankArb, "AccessBankArb: got write req %s from P2S[%d]\n",
            pkt->print(), id);

    if (blockedPacket || needRetry) {
        // Our previous ack to this same P2S engine hasn't gone out yet --
        // don't accept a new write until it has.
        needRetry = true;
        return false;
    }
    if (!owner->handleRequest(pkt, id)) {
        // Arbiter still holds an ungranted request from this client --
        // mirrors io.accessArray(clientId).valid staying high.
        needRetry = true;
        return false;
    }
    return true;
}

void
AccessBankArb::P2SPort::recvRespRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
    // May now be able to accept a new request from this same P2S engine.
    trySendRetry();
}

// ---------------------------------------------------------------------------
// AutoLoadPort
// ---------------------------------------------------------------------------

void
AccessBankArb::AutoLoadPort::sendPacket(PacketPtr pkt)
{
    panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

    DPRINTF(PICBankArb, "AccessBankArb: sending read resp %s to AutoLoadL\n",
            pkt->print());
    if (!sendTimingResp(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: AutoLoadL refused resp, "
                "will retry\n");
        blockedPacket = pkt;
    }
}

AddrRangeList
AccessBankArb::AutoLoadPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

void
AccessBankArb::AutoLoadPort::trySendRetry()
{
    if (needRetry && blockedPacket == nullptr) {
        needRetry = false;
        DPRINTF(PICBankArb, "AccessBankArb: sending retry req to AutoLoadL\n");
        sendRetryReq();
    }
}

bool
AccessBankArb::AutoLoadPort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(PICBankArb, "AccessBankArb: got read req %s from AutoLoadL\n",
            pkt->print());

    if (blockedPacket || needRetry) {
        needRetry = true;
        return false;
    }
    if (!owner->handleAutoLoadRequest(pkt)) {
        needRetry = true;
        return false;
    }
    return true;
}

void
AccessBankArb::AutoLoadPort::recvRespRetry()
{
    assert(blockedPacket != nullptr);
    PacketPtr pkt = blockedPacket;
    blockedPacket = nullptr;

    sendPacket(pkt);
    trySendRetry();
}

// ---------------------------------------------------------------------------
// Request/response handling
// ---------------------------------------------------------------------------

bool
AccessBankArb::handleRequest(PacketPtr pkt, int portIdx)
{
    panic_if(pkt->getSize() != sizeof(P2SWritePayload),
             "AccessBankArb: P2S[%d] sent a %u-byte write payload, "
             "expected %zu", portIdx, pkt->getSize(),
             sizeof(P2SWritePayload));

    const auto *payload = pkt->getConstPtr<P2SWritePayload>();
    const unsigned clientId = kClientP2S_L + portIdx;

    if (!submitP2SWrite(clientId, static_cast<uint32_t>(payload->arrayAddr),
                         payload->bitSlice)) {
        return false;
    }

    panic_if(pendingReqPkt.count(clientId) != 0,
             "AccessBankArb: client %u already has a pending packet",
             clientId);
    pendingReqPkt.emplace(clientId, pkt);

    DPRINTF(PICBankArb, "AccessBankArb: client %u posted addr=%#x\n",
            clientId, payload->arrayAddr);

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
    return true;
}

// Address domains don't line up yet: AutoLoadL sends a flat byte Addr
// (its own "row * 8" placeholder), while req.addr elsewhere in this file
// is the 17-bit array-address layout the bank-decode expects. Truncated
// straight through for now -- same "no real bank storage" caveat as the
// rest of this arbiter; needs the real AutoLoadL address encoding.
bool
AccessBankArb::handleAutoLoadRequest(PacketPtr pkt)
{
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "AccessBankArb: AutoLoadL sent a %u-byte read request, "
             "expected %zu", pkt->getSize(), sizeof(uint64_t));

    // One read at a time, until its response has been sent.
    if (autoLoadPkt) {
        return false;
    }

    ReqPackage req;
    req.addr = static_cast<uint32_t>(pkt->getAddr());
    req.optype = AccessArrayType::READ;

    if (!postRequest(kClientAutoLoadVec, req)) {
        return false;
    }
    autoLoadPkt = pkt;

    panic_if(pendingReqPkt.count(kClientAutoLoadVec) != 0,
             "AccessBankArb: AutoLoadL already has a pending packet");
    pendingReqPkt.emplace(kClientAutoLoadVec, pkt);

    DPRINTF(PICBankArb, "AccessBankArb: AutoLoadL posted read addr=%#x\n",
            req.addr);

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
    return true;
}

void
AccessBankArb::processTickEvent()
{
    std::optional<Grant> grant = tick();
    if (grant && grant->lateCommit) {
        DPRINTF(PICBankArb, "AccessBankArb: client %u committed after "
                "Block\n", grant->clientId);
    } else if (grant) {
        auto it = pendingReqPkt.find(grant->clientId);
        panic_if(it == pendingReqPkt.end(),
                 "AccessBankArb: granted client %u with no pending packet",
                 grant->clientId);
        PacketPtr pkt = it->second;
        pendingReqPkt.erase(it);

        if (grant->clientId == kClientAutoLoadVec) {
            DPRINTF(PICBankArb, "AccessBankArb: client %u fired (bank %s), "
                    "acking AutoLoadL\n", grant->clientId,
                    grant->committedThisCycle ? "ready" : "busy");
            // With Mats connected the response comes from the Mat
            // (handleMatResponse); in stub mode answer with zeros here.
            if (matPorts.empty()) {
                const uint64_t zero = 0;
                finishAutoLoad(reinterpret_cast<const uint8_t *>(&zero));
            }
        } else {
            const int portIdx =
                static_cast<int>(grant->clientId) - kClientP2S_L;
            DPRINTF(PICBankArb, "AccessBankArb: client %u fired (bank %s), "
                    "acking P2S[%d]\n", grant->clientId,
                    grant->committedThisCycle ? "ready" : "busy", portIdx);

            pkt->makeResponse();
            sendResponse(pkt, portIdx);
            // The client's one pending-request slot just freed up -- let
            // it send its next write if one was refused earlier.
            p2sPorts[portIdx].trySendRetry();
        }
    }

    // Keep ticking every cycle while there's pending work: either a
    // client waiting to be granted, or we're frozen in Block waiting on
    // a bank.
    const bool anyPending = std::any_of(reqValid.begin(), reqValid.end(),
                                         [](bool v) { return v; });
    if (anyPending || isBlocked()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

void
AccessBankArb::sendResponse(PacketPtr pkt, int portIdx)
{
    p2sPorts[portIdx].sendPacket(pkt);
}

bool
AccessBankArb::postRequest(unsigned clientId, const ReqPackage &req)
{
    fatal_if(clientId >= numClients,
             "AccessBankArb::postRequest: clientId %u out of range (%u "
             "clients)", clientId, numClients);
    if (reqValid[clientId]) {
        // Mirrors holding io.accessArray(clientId).valid high while ready
        // is still low: the caller must wait for the pending request to be
        // granted before posting a new one.
        return false;
    }
    reqValid[clientId] = true;
    reqPkg[clientId] = req;
    return true;
}

bool
AccessBankArb::isPending(unsigned clientId) const
{
    fatal_if(clientId >= numClients,
             "AccessBankArb::isPending: clientId %u out of range (%u "
             "clients)", clientId, numClients);
    return reqValid[clientId];
}

bool
AccessBankArb::submitP2SWrite(unsigned clientId, uint32_t arrayAddr,
                               uint64_t dataWrittenToBank)
{
    ReqPackage req;
    req.addr = arrayAddr;
    req.optype = AccessArrayType::WRITE;
    req.dataWrittenToBank = dataWrittenToBank;
    return postRequest(clientId, req);
}

std::optional<AccessBankArb::Grant>
AccessBankArb::tick()
{
    if (state == ArbState::Block) {
        // io.out.ready == false the whole time we're here: nobody else can
        // be granted until this specific write lands.
        if (!bankReady(blockedBankId))
            return std::nullopt;  // still frozen

        commit(blockedClientId, blockedReq, blockedBankId);
        state = ArbState::Access;
        return Grant{blockedClientId, /*committedThisCycle=*/true,
                     /*lateCommit=*/true};
    }

    // Access state: round-robin scan for the next pending client.
    for (unsigned i = 1; i <= numClients; ++i) {
        unsigned idx = (lastGranted + i) % numClients;
        if (!reqValid[idx])
            continue;

        // Fire: valid && ready. The client is dequeued right here,
        // regardless of what happens next -- matches how P2S treats
        // io.accessArray.fire as "write done".
        reqValid[idx] = false;
        lastGranted = idx;
        const ReqPackage req = reqPkg[idx];
        const unsigned bankId = arrayAddrToBankFn(req.addr);

        if (bankReady(bankId)) {
            commit(idx, req, bankId);
            return Grant{idx, /*committedThisCycle=*/true};
        }

        // Bank busy: freeze everyone else until this write actually lands.
        state = ArbState::Block;
        blockedReq = req;
        blockedClientId = idx;
        blockedBankId = bankId;
        return Grant{idx, /*committedThisCycle=*/false};
    }
    return std::nullopt;  // nothing pending
}

void
AccessBankArb::commit(unsigned clientId, const ReqPackage &req,
                       unsigned bankId)
{
    // Stub mode (no Mats connected): nothing to write or read.
    if (matPorts.empty())
        return;
    sendToMat(clientId, req, bankId);
}

unsigned
AccessBankArb::matIndex(uint32_t addr, unsigned bankId) const
{
    const unsigned matId = (addr >> (kOffsetBits + kArrayIdInMatBits +
                                     bankBits)) & (matsPerBank - 1);
    panic_if(matId < firstPicMat,
             "AccessBankArb: addr %#x targets matID %u, which has no Mat "
             "(first is %u)", addr, matId, firstPicMat);
    return (matId - firstPicMat) * bankReadyVec.size() + bankId;
}

void
AccessBankArb::sendToMat(unsigned clientId, const ReqPackage &req,
                         unsigned bankId)
{
    const unsigned idx = matIndex(req.addr, bankId);
    panic_if(idx >= matPorts.size(), "AccessBankArb: Mat index %u out of "
             "range (%zu Mats)", idx, matPorts.size());

    const bool isRead = (req.optype == AccessArrayType::READ);
    RequestPtr r = std::make_shared<Request>(
        req.addr & kWithinMatMask, sizeof(uint64_t), 0, requestorId);
    PacketPtr pkt = new Packet(r, isRead ? MemCmd::ReadReq
                                         : MemCmd::WriteReq);
    pkt->allocate();
    if (!isRead) {
        pkt->setData(
            reinterpret_cast<const uint8_t *>(&req.dataWrittenToBank));
    }
    pkt->pushSenderState(new ArbMatSenderState(clientId, isRead));

    DPRINTF(PICBankArb, "AccessBankArb: client %u -> Mat[%u] %s\n",
            clientId, idx, pkt->print());

    if (!matPorts[idx].sendTimingReq(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: Mat[%u] refused, bank %u "
                "not ready\n", idx, bankId);
        heldMat[bankId].pkt = pkt;
        heldMat[bankId].matIdx = idx;
    }
}

bool
AccessBankArb::handleMatResponse(PacketPtr pkt)
{
    auto *ss = safe_cast<ArbMatSenderState *>(pkt->popSenderState());
    if (ss->isRead) {
        panic_if(!autoLoadPkt, "AccessBankArb: read response with no "
                 "AutoLoadL request in flight");
        finishAutoLoad(pkt->getConstPtr<uint8_t>());
    }
    // A write was already acked to its client at grant.
    delete ss;
    delete pkt;
    return true;
}

void
AccessBankArb::finishAutoLoad(const uint8_t *data)
{
    PacketPtr pkt = autoLoadPkt;
    autoLoadPkt = nullptr;
    pkt->setData(data);
    pkt->makeResponse();
    autoLoadPort.sendPacket(pkt);
    autoLoadPort.trySendRetry();
}

void
AccessBankArb::matRetry(int idx)
{
    HeldMatReq &h = heldMat[idx % bankReadyVec.size()];
    panic_if(!h.pkt || h.matIdx != static_cast<unsigned>(idx),
             "AccessBankArb: unexpected retry from Mat[%d]", idx);
    PacketPtr pkt = h.pkt;
    if (matPorts[idx].sendTimingReq(pkt)) {
        h.pkt = nullptr;
    }
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
             "AccessBankArb::setBankReady: bank %u out of range (%zu "
             "banks)", bankId, bankReadyVec.size());
    bankReadyVec[bankId] = ready;
}

bool
AccessBankArb::bankReady(unsigned bankId) const
{
    fatal_if(bankId >= bankReadyVec.size(),
             "AccessBankArb::bankReady: bank %u out of range (%zu banks)",
             bankId, bankReadyVec.size());
    return bankReadyVec[bankId] && heldMat[bankId].pkt == nullptr;
}

void
AccessBankArb::setArrayAddrToBank(std::function<unsigned(uint32_t)> fn)
{
    arrayAddrToBankFn = std::move(fn);
}

} // namespace gem5
