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
 *
 * mod: p2s_arbiter
 * ---------------------------------------------------------------------
 * SimObject conversion -- P2S <-> AccessBankArb port wiring
 * ---------------------------------------------------------------------
 * AccessBankArb is now a ClockedObject with a real gem5 port for each of
 * the three P2S engines (p2s_side, a VectorResponsePort -- modeled on
 * DMAEngine's p2s_side in pic_dma_engine.hh). This replaces an earlier
 * approach where P2S_L/R/R_T held a raw pointer to a shared AccessBankArb
 * and called postRequest()/submitP2SWrite() directly as plain C++ calls
 * (a direct Decoupled-style wire, matching the RTL literally) -- that
 * worked when P2S was one module, but doesn't fit three independent
 * SimObject instances needing to reach one shared arbiter, so the link is
 * now a normal gem5 request/response port pair instead:
 *
 *   p2s_side[0] <-> P2S_L.cb_port      p2s_side[1] <-> P2S_R.cb_port
 *   p2s_side[2] <-> P2S_R_T.cb_port
 *
 * Port index doubles as (BankArbClient - kClientP2S_L), i.e. index 0/1/2
 * map to kClientP2S_L/kClientP2S_R/kClientP2S_R_T. Client ids 0-3
 * (load_post_process/bankFetch_module/accumulator/autoLoadVec) have no
 * gem5 SimObject yet and stay unconnected -- postRequest()/tick() still
 * accept them by id for whenever that changes.
 *
 * Request: a P2S engine sends a MemCmd::WriteReq packet carrying a
 * P2SWritePayload (pic_dpm_types.hh) on its cb_port. handleRequest()
 * unpacks it into a ReqPackage and calls submitP2SWrite(); the packet is
 * held in pendingReqPkt until granted. Only one not-yet-granted request
 * per client is allowed at a time (postRequest() returns false on a
 * second one), matching the RTL's single Decoupled register per client --
 * a P2S engine must wait for its response before sending the next write.
 *
 * Response: fires at *grant* (tick() returning a Grant for that client),
 * not at bank-commit -- see the "fire vs. commit" paragraph above. The
 * held packet is turned into a plain WriteResp (makeResponse(), no
 * payload -- P2S needs nothing back but the ack) and sent out that
 * client's p2s_side port.
 *
 * Bank side ("cache banks" in the req/resp flow this was built for): per
 * the file-header design above, this is deliberately left as the
 * existing functional stub -- bankReadyVec defaults every bank to
 * always-ready, so a granted write commits the same cycle. There is no
 * cache-bank SimObject in the tree yet (bank state today lives only
 * inside PICLLCTags, reached by plain function calls, never by ports),
 * and the RTL-documented fire semantics mean P2S never actually observes
 * bank-commit latency anyway. setBankReady()/setArrayAddrToBank() remain
 * the hooks for wiring in a real SRAM-timing model later without
 * disturbing the P2S-facing protocol above.
 *
 * Driving tick(): the SimObject schedules its own tickEvent one cycle
 * out whenever a request arrives or a grant leaves work behind (another
 * client still pending, or still Blocked on a bank) -- see
 * processTickEvent() in access_bank_arb.cc. tick()/postRequest()/
 * submitP2SWrite() themselves are untouched and still public, so the
 * arbiter logic stays unit-testable without constructing a SimObject.
 */

#ifndef __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
#define __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__

#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/AccessBankArb.hh"
#include "sim/clocked_object.hh"

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
 *
 * mod: p2s_arbiter -- now the SimObject P2S_L/P2S_R/P2S_R_T connect their
 * cb_port to (p2s_side, a VectorResponsePort); see the "SimObject
 * conversion" section of the file header comment for the port protocol.
 * The arbiter state machine below (postRequest/tick/the Access-Block
 * fields) is unchanged from the plain-C++-class version.
 */
class AccessBankArb : public ClockedObject
{
  public:
    // mod: p2s_arbiter
    // Must come before anything else that names Params -- see the
    // identical note in pic_dma_engine.hh's DMAEngine class.
    typedef AccessBankArbParams Params;

    explicit AccessBankArb(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    /**
     * mod: p2s_arbiter
     * Port on the P2S-facing side that receives bit-plane WRITE requests.
     * One instance per connection (vector port); id doubles as
     * (BankArbClient - kClientP2S_L) -- see the file header comment.
     * Modeled directly on DMAEngine::P2SPort (pic_dma_engine.hh).
     */
    class P2SPort : public ResponsePort
    {
      private:
        int id;
        AccessBankArb *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        P2SPort(const std::string &name, int id, AccessBankArb *owner) :
            ResponsePort(name), id(id), owner(owner), needRetry(false),
            blockedPacket(nullptr)
        { }

        /** Send @p pkt (already turned into a response) back to this
         *  port's P2S engine, buffering it if the port refuses. */
        void sendPacket(PacketPtr pkt);

        /** Not address-decoded -- a private point-to-point link to one
         *  P2S engine, not a bus-visible memory range. */
        AddrRangeList getAddrRanges() const override;

        /** Send a retry to the P2S engine if one is owed and we're free
         *  to accept a new request now (called once this port's client
         *  has been granted, freeing its one pending-request slot). */
        void trySendRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

  public:
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

    // ---- Arbiter state machine (unchanged from the plain-class version) ----

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

    // mod: p2s_arbiter
    // ---- P2S port-facing glue (new; declared after the arbiter state
    // above so the constructor's member-init list can stay in the same
    // relative order as before with these appended at the end) ----

    /** Handle one bit-plane WRITE request from P2S port index @p portIdx
     *  (0/1/2 = P2S_L/P2S_R/P2S_R_T). Unpacks the P2SWritePayload and
     *  posts it to the arbiter under clientId = kClientP2S_L + portIdx;
     *  the packet itself is held in pendingReqPkt until granted. Returns
     *  false if that client already has an outstanding request (mirrors
     *  postRequest()'s valid-held-high case) -- caller should set
     *  needRetry and wait for trySendRetry(). */
    bool handleRequest(PacketPtr pkt, int portIdx);

    /** One arbiter clock edge: calls tick(), and if it granted a client,
     *  turns that client's held request packet into a response and sends
     *  it out the matching p2s_side port, then frees that port to accept
     *  a new request via trySendRetry(). Reschedules itself one cycle out
     *  while any client is pending or the arbiter is still Blocked. */
    void processTickEvent();

    /** Send @p pkt out p2s_side port index @p portIdx. */
    void sendResponse(PacketPtr pkt, int portIdx);

    std::vector<P2SPort> p2sPorts;

    /** clientId (kClientP2S_L..kClientP2S_R_T) -> the P2S write-request
     *  packet held until that client is granted; see handleRequest(). */
    std::unordered_map<unsigned, PacketPtr> pendingReqPkt;

    EventFunctionWrapper tickEvent;
};

} // namespace gem5

#endif // __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
