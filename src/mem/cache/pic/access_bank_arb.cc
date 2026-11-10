#include "mem/cache/pic/access_bank_arb.hh"

#include <algorithm>
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
    ArbMatSenderState(unsigned _clientId, bool _isRead, uint32_t _addr)
        : clientId(_clientId), isRead(_isRead), addr(_addr)
    {}

    const unsigned clientId;
    const bool isRead;
    /** Full 17-bit array address of the access. */
    const uint32_t addr;
};
} // namespace

AccessBankArb::AccessBankArb(const Params &p)
    : ClockedObject(p),
      numClients(kNumBankArbClients),
      reqValid(kNumBankArbClients, false),
      // Deterministic simulation startup: scan from client 0 first.
      lastGranted(kNumBankArbClients - 1),
      state(ArbState::Access),
      blockedClientId(0),
      blockedBankId(0),
      bankReadyVec(p.num_banks, true),
      autoLoadPort(p.name + ".autoload_side", this),
      loadPostPort(p.name + ".load_post_side", kClientLoadPostProcess, this),
      bankFetchPort(p.name + ".bank_fetch_side", kClientBankFetch, this),
      accPort(p.name + ".acc_side", kClientAccumulator, this),
      // Collect the clients' requests before choosing this cycle's grant.
      tickEvent([this]{ processTickEvent(); }, name() + ".tickEvent", false,
                Event::Progress_Event_Pri),
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
            p.name + csprintf(".p2s_side[%d]", i), kClientP2S_L + i, this);
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
    if (if_name == "load_post_side") {
        return loadPostPort;
    }
    if (if_name == "bank_fetch_side") {
        return bankFetchPort;
    }
    if (if_name == "acc_side") {
        return accPort;
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

    DPRINTF(PICBankArb, "AccessBankArb: sending ack %s to client %d\n",
            pkt->print(), id);
    if (!sendTimingResp(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: client %d refused ack, will retry\n",
                id);
        blockedPacket = pkt;
    }
}

AddrRangeList
AccessBankArb::P2SPort::getAddrRanges() const
{
    // Private point-to-point link to one P2S engine, not decoded off a
    // shared bus -- claim the whole address space like other such links
    // in this tree (e.g. DMA_read_page_split::P2SPort).
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

void
AccessBankArb::P2SPort::trySendRetry()
{
    if (canRetry() && owner->canAccept(id)) {
        needRetry = false;
        DPRINTF(PICBankArb, "AccessBankArb: sending retry req to client %d\n",
                id);
        sendRetryReq();
    }
}

bool
AccessBankArb::P2SPort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(PICBankArb, "AccessBankArb: got write req %s from client %d\n",
            pkt->print(), id);

    if (blockedPacket || needRetry) {
        // Our previous ack to this same P2S engine hasn't gone out yet --
        // don't accept a new write until it has.
        needRetry = true;
        owner->waitForGrant(id);
        return false;
    }
    if (!owner->handleRequest(pkt, id)) {
        // The client retains its request while valid waits for ready.
        needRetry = true;
        owner->waitForGrant(id);
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
    if (canRetry() && owner->canAccept(kClientAutoLoadVec)) {
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
        owner->waitForGrant(kClientAutoLoadVec);
        return false;
    }
    if (!owner->handleAutoLoadRequest(pkt)) {
        needRetry = true;
        owner->waitForGrant(kClientAutoLoadVec);
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
// BankPort
// ---------------------------------------------------------------------------

void
AccessBankArb::BankPort::sendPacket(PacketPtr pkt)
{
    DPRINTF(PICBankArb, "AccessBankArb: sending resp %s to client %u\n",
            pkt->print(), clientId);
    // Keep responses in order: queue behind any this client refused.
    if (!respQueue.empty() || !sendTimingResp(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: client %u resp queued\n",
                clientId);
        respQueue.push_back(pkt);
    }
}

AddrRangeList
AccessBankArb::BankPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(0, MaxAddr));
    return ranges;
}

void
AccessBankArb::BankPort::trySendRetry()
{
    if (canRetry() && owner->canAccept(clientId)) {
        needRetry = false;
        DPRINTF(PICBankArb, "AccessBankArb: sending retry req to client %u\n",
                clientId);
        sendRetryReq();
    }
}

bool
AccessBankArb::BankPort::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(PICBankArb, "AccessBankArb: got req %s from client %u\n",
            pkt->print(), clientId);

    if (!respQueue.empty() || needRetry) {
        needRetry = true;
        owner->waitForGrant(clientId);
        return false;
    }
    if (!owner->handleBankRequest(pkt, clientId)) {
        needRetry = true;
        owner->waitForGrant(clientId);
        return false;
    }
    return true;
}

void
AccessBankArb::BankPort::recvRespRetry()
{
    assert(!respQueue.empty());
    while (!respQueue.empty() && sendTimingResp(respQueue.front())) {
        respQueue.pop_front();
    }
    trySendRetry();
}

// ---------------------------------------------------------------------------
// Request/response handling
// ---------------------------------------------------------------------------

bool
AccessBankArb::handleRequest(PacketPtr pkt, unsigned clientId)
{
    panic_if(pkt->getSize() != sizeof(P2SWritePayload),
             "AccessBankArb: client %u sent a %u-byte write payload, "
             "expected %zu", clientId, pkt->getSize(),
             sizeof(P2SWritePayload));

    const auto *payload = pkt->getConstPtr<P2SWritePayload>();

    if (!submitP2SWrite(clientId, static_cast<uint32_t>(payload->arrayAddr),
                         payload->bitSlice)) {
        return false;
    }

    panic_if(grantedPkt, "AccessBankArb: two packets accepted in one grant");
    grantedPkt = pkt;

    DPRINTF(PICBankArb, "AccessBankArb: client %u posted addr=%#x\n",
            clientId, payload->arrayAddr);

    scheduleTick();
    return true;
}

// AutoLoadL sends the full 17-bit word address, without a byte offset.
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

    panic_if(grantedPkt, "AccessBankArb: two packets accepted in one grant");
    grantedPkt = pkt;

    DPRINTF(PICBankArb, "AccessBankArb: AutoLoadL posted read addr=%#x\n",
            req.addr);

    scheduleTick();
    return true;
}

bool
AccessBankArb::handleBankRequest(PacketPtr pkt, unsigned clientId)
{
    panic_if(clientId == kClientBankFetch && !pkt->isRead(),
             "AccessBankArb: BankFetch only supports reads");
    panic_if(pkt->getSize() != sizeof(uint64_t),
             "AccessBankArb: client %u sent a %u-byte request, "
             "expected %zu", clientId, pkt->getSize(), sizeof(uint64_t));

    ReqPackage req;
    req.addr = static_cast<uint32_t>(pkt->getAddr());
    if (pkt->isRead()) {
        req.optype = AccessArrayType::READ;
    } else {
        panic_if(!pkt->isWrite(), "AccessBankArb: Accumulator sent %s",
                 pkt->print());
        req.optype = AccessArrayType::WRITE;
        req.dataWrittenToBank = pkt->getLE<uint64_t>();
    }

    // One not-yet-granted request at a time, like every other client.
    if (!postRequest(clientId, req)) {
        return false;
    }

    panic_if(grantedPkt, "AccessBankArb: two packets accepted in one grant");
    grantedPkt = pkt;

    DPRINTF(PICBankArb, "AccessBankArb: %s posted %s addr=%#x\n",
            clientId == kClientBankFetch ? "BankFetch" : "Accumulator",
            pkt->isRead() ? "read" : "write", req.addr);

    scheduleTick();
    return true;
}

void
AccessBankArb::processTickEvent()
{
    lastTick = curTick();
    std::optional<Grant> grant = tick();
    if (grant && grant->lateCommit) {
        DPRINTF(PICBankArb, "AccessBankArb: client %u committed after "
                "Block\n", grant->clientId);
    } else if (grant) {
        panic_if(!grantedPkt,
                 "AccessBankArb: granted client %u with no pending packet",
                 grant->clientId);
        PacketPtr pkt = grantedPkt;
        grantedPkt = nullptr;
        DPRINTF(PICBankArb, "AccessBankArb: client %u fired (bank %s)\n",
                grant->clientId,
                grant->committedThisCycle ? "ready" : "busy");

        if (grant->clientId == kClientAutoLoadVec) {
            // With Mats connected the response comes from the Mat
            // (handleMatResponse); in stub mode answer with zeros here.
            if (matPorts.empty()) {
                const uint64_t zero = 0;
                finishAutoLoad(reinterpret_cast<const uint8_t *>(&zero));
            }
        } else if (grant->clientId == kClientAccumulator ||
                   grant->clientId == kClientBankFetch) {
            if (pkt->isRead()) {
                // These are already fired reads, not pending RR inputs.
                // Match returning data to this client's grant order.
                readFifo(grant->clientId).push_back(pkt);
                if (matPorts.empty()) {
                    const uint64_t zero = 0;
                    finishBankRead(grant->clientId,
                                   reinterpret_cast<const uint8_t *>(&zero));
                }
            } else {
                pkt->makeResponse();
                bankPort(grant->clientId).sendPacket(pkt);
            }
            // Any next request still needs its own RR grant.
            bankPort(grant->clientId).trySendRetry();
        } else {

            P2SPort &port = writePort(grant->clientId);
            pkt->makeResponse();
            port.sendPacket(pkt);
            // A waiting next write must receive its own RR grant.
            port.trySendRetry();
        }
    }

    // Keep ticking every cycle while there's pending work: either a
    // client waiting to be granted, or we're frozen in Block waiting on
    // a bank.
    const bool anyPending = std::any_of(reqValid.begin(), reqValid.end(),
                                         [](bool v) { return v; });
    // A client answering a retry right away (the Accumulator) may already
    // have scheduled the next tick from inside this one.
    if ((anyPending || isBlocked()) && !tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

AccessBankArb::P2SPort &
AccessBankArb::writePort(unsigned clientId)
{
    if (clientId == kClientLoadPostProcess)
        return loadPostPort;
    return p2sPorts[clientId - kClientP2S_L];
}

bool
AccessBankArb::postRequest(unsigned clientId, const ReqPackage &req)
{
    fatal_if(clientId >= numClients,
             "AccessBankArb::postRequest: clientId %u out of range (%u "
             "clients)", clientId, numClients);
    if (!canAccept(clientId)) {
        // valid is remembered, but the caller retains the packet and its
        // payload until this client is selected and retries successfully.
        waitForGrant(clientId);
        return false;
    }
    reqValid[clientId] = false;
    firedClient = clientId;
    firedReq = req;
    DPRINTF(PICBankArb, "AccessBankArb: client %u accepted (RTL fire) "
            "addr=%#x\n", clientId, req.addr);
    return true;
}

bool
AccessBankArb::canAccept(unsigned clientId) const
{
    return state == ArbState::Access && reservedClient == clientId &&
           !firedClient;
}

void
AccessBankArb::scheduleTick()
{
    if (!tickEvent.scheduled()) {
        const Tick when = clockEdge();
        schedule(tickEvent, lastTick && when <= *lastTick ?
                 clockEdge(Cycles(1)) : when);
    }
}

void
AccessBankArb::waitForGrant(unsigned clientId)
{
    if (!reqValid[clientId]) {
        reqValid[clientId] = true;
        DPRINTF(PICBankArb, "AccessBankArb: client %u waiting for grant\n",
                clientId);
    }
    scheduleTick();
}

bool
AccessBankArb::clientCanRetry(unsigned clientId) const
{
    if (clientId == kClientAutoLoadVec)
        return !autoLoadPkt && autoLoadPort.canRetry();
    if (clientId == kClientAccumulator || clientId == kClientBankFetch)
        return (clientId == kClientBankFetch ? bankFetchPort : accPort).canRetry();
    if (clientId == kClientLoadPostProcess)
        return loadPostPort.canRetry();
    if (clientId >= kClientP2S_L && clientId <= kClientP2S_R_T)
        return p2sPorts[clientId - kClientP2S_L].canRetry();
    return false;
}

void
AccessBankArb::retryClient(unsigned clientId)
{
    if (clientId == kClientAutoLoadVec)
        autoLoadPort.trySendRetry();
    else if (clientId == kClientAccumulator || clientId == kClientBankFetch)
        bankPort(clientId).trySendRetry();
    else
        writePort(clientId).trySendRetry();
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
        // be granted until this specific access lands.
        if (!bankReadyVec[blockedBankId])
            return std::nullopt;  // still frozen

        HeldMatReq &held = heldMat[blockedBankId];
        if (held.pkt) {
            // A refused packet is already the blocked request. Wait for
            // the Mat's retry, then resend it here instead of committing
            // blockedReq a second time.
            if (!held.retryReady)
                return std::nullopt;
            held.retryReady = false;
            if (!matPorts[held.matIdx].sendTimingReq(held.pkt))
                return std::nullopt;
            held.pkt = nullptr;
        } else if (!commit(blockedClientId, blockedReq, blockedBankId)) {
            return std::nullopt;
        }

        state = ArbState::Access;
        return Grant{blockedClientId, /*committedThisCycle=*/true,
                     /*lateCommit=*/true};
    }

    if (!reservedClient) {
        for (unsigned i = 1; i <= numClients; ++i) {
            const unsigned idx = (lastGranted + i) % numClients;
            if (reqValid[idx] && clientCanRetry(idx)) {
                reservedClient = idx;
                break;
            }
        }
    }
    if (reservedClient && !firedClient && clientCanRetry(*reservedClient))
        retryClient(*reservedClient);

    if (!firedClient)
        return std::nullopt;

    // The selected client has now returned from sendTimingReq(true), so
    // it has advanced its dequeue pointer before any ACK is delivered.
    const unsigned idx = *firedClient;
    const ReqPackage req = firedReq;
    firedClient.reset();
    reservedClient.reset();
    lastGranted = idx;
    const unsigned bankId = arrayAddrToBankFn(req.addr);

    if (bankReady(bankId) && commit(idx, req, bankId))
        return Grant{idx, /*committedThisCycle=*/true};

    state = ArbState::Block;
    blockedReq = req;
    blockedClientId = idx;
    blockedBankId = bankId;
    return Grant{idx, /*committedThisCycle=*/false};
}

bool
AccessBankArb::commit(unsigned clientId, const ReqPackage &req,
                       unsigned bankId)
{
    // Stub mode (no Mats connected): nothing to write or read.
    if (matPorts.empty())
        return true;
    return sendToMat(clientId, req, bankId);
}

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
    pkt->pushSenderState(new ArbMatSenderState(clientId, isRead, req.addr));

    DPRINTF(PICBankArb, "AccessBankArb: client %u -> Mat[%u] %s\n",
            clientId, idx, pkt->print());

    if (!matPorts[idx].sendTimingReq(pkt)) {
        DPRINTF(PICBankArb, "AccessBankArb: Mat[%u] refused, bank %u "
                "not ready\n", idx, bankId);
        heldMat[bankId].pkt = pkt;
        heldMat[bankId].matIdx = idx;
        heldMat[bankId].retryReady = false;
        return false;
    }
    return true;
}

bool
AccessBankArb::handleMatResponse(PacketPtr pkt)
{
    auto *ss = safe_cast<ArbMatSenderState *>(pkt->popSenderState());
    if (ss->isRead) {
        // Send the read data back to the client that asked for it.
        if (ss->clientId == kClientAutoLoadVec) {
            panic_if(!autoLoadPkt, "AccessBankArb: read response with no "
                     "AutoLoadL request in flight");
            finishAutoLoad(pkt->getConstPtr<uint8_t>());
        } else if (ss->clientId == kClientAccumulator ||
                   ss->clientId == kClientBankFetch) {
            auto &fifo = readFifo(ss->clientId);
            panic_if(fifo.empty(), "AccessBankArb: read response with no "
                     "client %u read in flight", ss->clientId);
            panic_if(fifo.front()->getAddr() != ss->addr,
                     "AccessBankArb: client %u read response addr %#x, "
                     "oldest read addr %#x", ss->clientId, ss->addr,
                     fifo.front()->getAddr());
            finishBankRead(ss->clientId, pkt->getConstPtr<uint8_t>());
        } else {
            panic("AccessBankArb: read response for client %u, which has "
                  "no read port", ss->clientId);
        }
    }
    // A write was already acked to its client at grant.
    delete ss;
    delete pkt;
    return true;
}

AccessBankArb::BankPort &
AccessBankArb::bankPort(unsigned clientId)
{
    panic_if(clientId != kClientAccumulator && clientId != kClientBankFetch,
             "AccessBankArb: no raw request port for client %u", clientId);
    return clientId == kClientBankFetch ? bankFetchPort : accPort;
}

std::deque<PacketPtr> &
AccessBankArb::readFifo(unsigned clientId)
{
    panic_if(clientId != kClientAccumulator && clientId != kClientBankFetch,
             "AccessBankArb: no read FIFO for client %u", clientId);
    return clientId == kClientBankFetch ? bankFetchReadFifo : accReadFifo;
}

void
AccessBankArb::finishBankRead(unsigned clientId, const uint8_t *data)
{
    auto &fifo = readFifo(clientId);
    PacketPtr pkt = fifo.front();
    fifo.pop_front();
    pkt->setData(data);
    pkt->makeResponse();
    bankPort(clientId).sendPacket(pkt);
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
    // Keep the request in Block until an arbiter tick can resend it.
    // That tick returns without granting another client, so arbitration
    // resumes on the following cycle, with no duplicate commit or ACK.
    h.retryReady = true;
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
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
