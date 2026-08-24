#include "mem/cache/pic/access_bank_arb.hh"

#include "base/logging.hh"

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
} // namespace

AccessBankArb::AccessBankArb(unsigned num_banks, unsigned num_clients)
    : numClients(num_clients),
      reqValid(num_clients, false),
      reqPkg(num_clients),
      // Start so the first tick() scans from client 0, matching an
      // RRArbiter that resets its rotating-priority pointer to 0.
      lastGranted(num_clients - 1),
      state(ArbState::Access),
      blockedClientId(0),
      blockedBankId(0),
      bankReadyVec(num_banks, true)
{
    fatal_if(num_clients == 0, "AccessBankArb: num_clients must be > 0");
    fatal_if(num_banks == 0, "AccessBankArb: num_banks must be > 0");

    // mod: p2s_arbiter
    // Default decode, matching get_bankID() (SysConfig.scala:282-305):
    // bankID sits right above arrayID_in_mat/wordline_offset and right
    // below matID_in_bank -- addr[12:11] for the paper's default 4-bank
    // config (see the field-layout note above). This is no longer a
    // placeholder for that default config; setArrayAddrToBank() remains
    // the override point if num_banks/matPerBank/core_addrLen differ.
    arrayAddrToBankFn = [num_banks](uint32_t addr) {
        constexpr unsigned shift = kOffsetBits + kArrayIdInMatBits; // = 11
        return static_cast<unsigned>((addr >> shift) & (num_banks - 1));
    };
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
        return Grant{blockedClientId, /*committedThisCycle=*/true};
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
    // WRITE is the only optype currently exercised (P2S has no read port
    // in the RTL either, so there's nothing to respond with). This is
    // still the hook for actually depositing dataWrittenToBank into
    // simulated bank storage once that exists; READ/dataReadValid
    // handling was removed for now -- see the file header comment.
    (void)clientId;
    (void)req;
    (void)bankId;
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
    return bankReadyVec[bankId];
}

void
AccessBankArb::setArrayAddrToBank(std::function<unsigned(uint32_t)> fn)
{
    arrayAddrToBankFn = std::move(fn);
}

} // namespace gem5
