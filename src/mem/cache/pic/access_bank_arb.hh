/**
 * @file
 * AccessBankArb -- functional model of AccessBank_Arb (chipyard reference:
 * bankAccessScheduler.scala), the single round-robin arbiter shared by all
 * clients that route data through a PIC bank's physical port.
 *
 * This is the "path selection into the Mat/SRAM" half of the PolymorPIC
 * paper's Arbiter concept: routing DRAM data toward a p2s engine and
 * routing p2s results into the correct Mat. It deliberately knows nothing
 * about ways, sets, or tags -- that's the *other* half of the paper's
 * Arbiter (mode-based CPU/PIC path selection + PAI isolation), which stays
 * in PICLLCTags::accessBlock()/findVictim() since only the tag store has
 * per-way mode state. See the "Bank Access Arbiter" note that used to live
 * in pic_llc_tags.hh for the original design discussion; it has moved here
 * along with the code.
 *
 * ONE AccessBankArb instance is shared cluster-wide -- not one per bank --
 * matching the RTL where all clients arbitrate for a single
 * cache_addr/cache_write/cache_enable/cache_data_to_bank port and the
 * winning request's target bank is decoded from its address afterward.
 *
 * Clients (SysConfig.scala:170-177, RRArbiter(new ReqPackage(sysCfg), 7)):
 *   0 load_post_process   4 P2S_L
 *   1 bankFetch_module    5 P2S_R
 *   2 accumulator         6 P2S_R_T
 *   3 autoLoadVec
 * bankFetch_module / accumulator / autoLoadVec additionally wire
 * .dataFromBank := accessArb.io.dataReadFromBank and .dataReadValid :=
 * accessArb.io.dataReadValid(ID) in the RTL -- not modeled here yet (no
 * reading client is implemented). The three P2S clients' io.accessArray has
 * NO dataFromBank/dataReadValid fields at all -- they always send
 * optype=WRITE (P2S_L_ctl.scala:236, P2S_R_ctl.scala:407,
 * P2S_R_T_ctl.scala:149) and are structurally incapable of receiving read
 * data, so a response path was not carried over when this was split out of
 * pic_llc_tags.hh; re-add it here (dataReadFromBank()/dataReadValid()) if a
 * reading client is ever implemented.
 *
 * Per-request fields (bankAccessScheduler.scala:24-28), all P2S ever sets:
 *   addr              : 17-bit array address (bank/Mat/wordline location),
 *                        NOT a cache-line Addr.
 *                          P2S_L   -> pic_write_ptr
 *                          P2S_R   -> arrayAddrEnq
 *                          P2S_R_T -> (curArrayID << 9) + rowPtrStore
 *   optype             : always AccessArrayType.WRITE for P2S.
 *   dataWrittenToBank  : 64-bit bit-plane (extractBits() output).
 * Handshake: standard Decoupled valid/ready.
 *
 * Confirmed 17-bit addr layout (paper's default config; see
 * access_bank_arb.cc), MSB to LSB -- matID_in_bank/bankID/arrayID_in_mat/
 * offset already baked into addr by the caller (currArrayID << 9 |
 * offset), the arbiter only ever pulls the bankID sub-field back out via
 * arrayAddrToBankFn (SysConfig.scala:282-305's get_bankID()):
 *   [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | offset(9) ]
 * Note Bank sits *below* matID_in_bank, not at the very top of the
 * address.
 *
 * Arbiter state machine (bankAccessScheduler.scala):
 *   - Access state: io.out.ready = true. Round-robins to the next pending
 *     client and fires (valid && ready). The winning request's target bank
 *     is decoded (get_bankID(addr)) and checked against mem_ready(bank).
 *       - Bank ready     -> the write commits the SAME cycle; arbiter
 *         stays in Access, free to grant the next client next cycle.
 *       - Bank NOT ready -> arbiter moves to Block: cache_addr /
 *         cache_data_to_bank keep being driven from the latched request
 *         every cycle until mem_ready finally goes true.
 *   - Block state: io.out.ready = false, so EVERY other client (including
 *     the other two P2S variants) is frozen -- not just the one waiting --
 *     until this specific write lands.
 * Crucially, the fire (valid && ready) that grants a client happens up
 * front, before the bank-ready check. So from a P2S module's own point of
 * view, io.accessArray.fire == true means the write is done and it can
 * advance its dequeue pointer immediately, even though the arbiter may
 * still be spinning in Block behind the scenes waiting for the bank.
 * AccessBankArb::tick() mirrors this: Grant is returned at fire time with
 * a committedThisCycle flag callers may ignore for a write-only (P2S-style)
 * client.
 */

#ifndef __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
#define __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace gem5
{

/** Request-side operation code for AccessBankArb (RTL: AccessArrayType). */
enum class AccessArrayType : uint8_t
{
    READ,
    WRITE
};

/**
 * gem5-side mirror of Chisel's ReqPackage bundle (bankAccessScheduler.scala
 * :24-28) -- the payload carried by io.accessArray : Decoupled(ReqPackage).
 */
struct ReqPackage
{
    /** 17-bit array address (bank/Mat/wordline location). */
    uint32_t addr = 0;
    /** READ/WRITE; P2S clients always use WRITE. */
    AccessArrayType optype = AccessArrayType::WRITE;
    /** 64-bit bit-plane payload for a WRITE. */
    uint64_t dataWrittenToBank = 0;
};

/** Fixed client IDs sharing the one AccessBankArb (SysConfig.scala:170-177). */
enum BankArbClient : unsigned
{
    kClientLoadPostProcess = 0, // store, plain store data into cache banks
    kClientBankFetch       = 1, // load, plain load data from cache banks
    kClientAccumulator     = 2,
    kClientAutoLoadVec     = 3, // cal, load L vectores fromcache bank → Mat's compute vector register
    kClientP2S_L           = 4,
    kClientP2S_R           = 5,
    kClientP2S_R_T         = 6,
};

/** Total clients sharing the single AccessBank_Arb in the paper design. */
static constexpr unsigned kNumBankArbClients = 7;

/**
 * Functional model of AccessBank_Arb: ONE round-robin arbiter, shared by
 * all kNumBankArbClients Decoupled(ReqPackage) request ports, driving a
 * single access into the bank hierarchy per grant (not one arbiter per
 * bank -- see the file header comment).
 *
 * Models the Access/Block state machine from bankAccessScheduler.scala:
 * a grant that targets a not-yet-ready bank still fires (the client is
 * dequeued immediately, matching how P2S treats io.accessArray.fire as
 * "write done"), but the arbiter then freezes every other client in
 * Block state until that specific write actually lands.
 */
class AccessBankArb
{
  public:
    explicit AccessBankArb(unsigned num_banks,
                            unsigned num_clients = kNumBankArbClients);

    /**
     * Assert io.accessArray(clientId).valid with @p req.
     * Returns false if that client already has an outstanding,
     * not-yet-granted request (mirrors holding valid high until fire).
     */
    bool postRequest(unsigned clientId, const ReqPackage &req);

    //query funct to check if a client has a pending request, without posting a new one
    bool isPending(unsigned clientId) const;

    /**
     * Convenience wrapper for P2S clients: builds a WRITE ReqPackage and
     * posts it. Response side (dataReadFromBank/dataReadValid) should be
     * ignored by the caller -- P2S has no read port in the RTL either.
     */
    bool submitP2SWrite(unsigned clientId, uint32_t arrayAddr,
                         uint64_t dataWrittenToBank); //arrayAddr = 17 bits (<32)

    // return what client is deque and if the request is committed this cycle (fire && bank ready)
    struct Grant
    {
        unsigned clientId; //client that fired
        bool committedThisCycle; // bank is ready = commited
    };

    /**
     * Run one arbiter clock edge. Must be called once per cycle by
     * whatever owns the bank-access clock (e.g. the cache controller)
     * while there is pending work.
     *
     * @return the grant that fired this cycle, or std::nullopt if the
     *         arbiter is idle (no pending client) or still frozen in
     *         Block waiting on a bank.
     */
    std::optional<Grant> tick();

    /** True while frozen in Block state (no new client can be granted). */
    bool isBlocked() const { return state == ArbState::Block; }

    //Defaults to always-ready (no bank contention modeled) until wired to a real SRAM-timing source.
    void setBankReady(unsigned bankId, bool ready); //for SRAM to set ready/busy
    bool bankReady(unsigned bankId) const; // query if the bank is ready or not

    /**
     * Install the array-address -> bank-index decode function. Defaults to
     * get_bankID() (SysConfig.scala:282-305) for the paper's default
     * config: addr[12:11], i.e. bankID sits between matID_in_bank (above)
     * and arrayID_in_mat/wordline_offset (below) -- see access_bank_arb.cc.
     * Override if num_banks/matPerBank/core_addrLen differ from that
     * default.
     */
    void setArrayAddrToBank(std::function<unsigned(uint32_t)> fn);

    // dataReadFromBank()/dataReadValid() intentionally not present -- see
    // the file header comment: no reading client is implemented yet.

  private:
    enum class ArbState { Access, Block };

    void commit(unsigned clientId, const ReqPackage &req, unsigned bankId);

    unsigned numClients;
    std::vector<bool> reqValid;
    std::vector<ReqPackage> reqPkg;
    /** ID of the client granted last time, so round-robin can resume after
     *  it (matches RRArbiter's rotating-priority behavior). */
    unsigned lastGranted;

    ArbState state;
    ReqPackage blockedReq;
    unsigned blockedClientId;
    unsigned blockedBankId;

    std::vector<bool> bankReadyVec;
    std::function<unsigned(uint32_t)> arrayAddrToBankFn;
};

} // namespace gem5

#endif // __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
