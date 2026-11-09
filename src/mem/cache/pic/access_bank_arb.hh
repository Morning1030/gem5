/**
 * AccessBankArb models the single shared RTL AccessBank_Arb.
 * Clients: 0 LoadPostProcess, 1 BankFetch, 2 ACC,
 * 3 AutoLoadVec, 4 P2S_L, 5 P2S_R, 6 P2S_R_T.
 *
 * Request acceptance is RTL fire: recvTimingReq returns false until the
 * client is selected. Only pending valid bits are recorded; refused
 * packets and their payloads stay with their clients. An Access-state
 * tick reserves one client and sends its retry. The successful resend
 * returns true, advances lastGranted, and drives this cycle's bank access.
 * No per-client payload FIFO exists in the arbiter.
 *
 * A fire does not require the bank to be ready. If the target bank is
 * busy or its Mat refuses the access, the one accepted request is latched
 * in Block; nobody else fires until that request lands. Mat retries only
 * mark the held packet ready for a Block-state tick. The completion tick
 * grants nobody else; normal arbitration resumes the following cycle.
 *
 * WRITE ACKs confirm fire, not bank completion. They are sent after the
 * successful resend has returned to its client, so the client can safely
 * dequeue before the ACK arrives. READ responses carry Mat data back to
 * the selected client; they are separate from request acceptance.
 *
 * Request fields are {addr:17, optype:1, writeData:64} in RTL. Addr layout:
 * [mat_in_bank:4 | bank:2 | array_in_mat:2 | wordline:9]. Mat ports are
 * indexed (mat_in_bank - first_pic_mat) * num_banks + bank and receive
 * only the 11-bit within-Mat word index. Without Mat ports, stub reads
 * return zero. setBankReady() supplies normal-cache bank contention.
 */

#ifndef __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
#define __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
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
 */
class AccessBankArb : public ClockedObject
{
  public:
    // mod: p2s_arbiter
    // Must come before anything else that names Params -- see the
    // identical note in dma_read_page_split.hh's DMA_read_page_split class.
    typedef AccessBankArbParams Params;

    explicit AccessBankArb(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    /** Port to one Mat. */
    class MatSidePort : public RequestPort
    {
      private:
        int id;
        AccessBankArb *owner;

      public:
        MatSidePort(const std::string &name, int id, AccessBankArb *owner) :
            RequestPort(name), id(id), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override { }
    };

    /**
     * mod: p2s_arbiter
     * Port for a WRITE client: the three P2S engines (p2s_side) and
     * ReadPostProcess (load_post_side). id is the BankArbClient id
     * (kClientP2S_L.. for p2s_side, kClientLoadPostProcess).
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

        /** Send a retry only when RR has reserved this client's grant
         *  and the port is able to accept its resend. */
        void trySendRetry();
        bool canRetry() const { return needRetry && !blockedPacket; }

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Port for the single cache-wide AutoLoadL (kClientAutoLoadVec). Same
     * one-read-at-a-time protocol: request acceptance is fire, while
     * the response carries the data returned by the Mat later.
     */
    class AutoLoadPort : public ResponsePort
    {
      private:
        AccessBankArb *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        AutoLoadPort(const std::string &name, AccessBankArb *owner) :
            ResponsePort(name), owner(owner), needRetry(false),
            blockedPacket(nullptr)
        { }

        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;
        void trySendRetry();
        bool canRetry() const { return needRetry && !blockedPacket; }

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Raw 8-byte request port for ACC and BankFetch. BankFetch only
     * reads; ACC reads and writes. Writes are ACKed at fire; read data
     * returns with the Mat response. Each port queues refused responses.
     */
    class BankPort : public ResponsePort
    {
      private:
        AccessBankArb *owner;
        const unsigned clientId;
        bool needRetry;
        std::deque<PacketPtr> respQueue;

      public:
        BankPort(const std::string &name, unsigned clientId,
                 AccessBankArb *owner) :
            ResponsePort(name), owner(owner), clientId(clientId),
            needRetry(false)
        { }

        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;
        void trySendRetry();
        bool canRetry() const { return needRetry && respQueue.empty(); }

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

  public:
    /**
     * Present io.request(clientId).valid and bits. True is the selected
     * client's RTL fire. On false only the valid bit is remembered; the
     * caller retains the payload and retries when the arbiter selects it.
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
        /** True when this is the late commit of a request that was already
         *  granted earlier (it went to Block); the client was acked back
         *  then, so there is nothing more to ack. */
        bool lateCommit = false;
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

    // dataReadFromBank()/dataReadValid() are not separate calls: read data
    // goes back on the reading client's port (handleMatResponse()).

  private:
    enum class ArbState { Access, Block };

    /** True only once the target Mat accepts this access. */
    bool commit(unsigned clientId, const ReqPackage &req, unsigned bankId);

    // ---- Shared arbiter state and input-valid bookkeeping ----

    unsigned numClients;
    std::vector<bool> reqValid;
    std::optional<unsigned> reservedClient;
    std::optional<unsigned> firedClient;
    ReqPackage firedReq;
    /** ID of the client granted last time, so round-robin can resume after
     *  it (matches RRArbiter's rotating-priority behavior). */
    unsigned lastGranted;

    ArbState state;
    ReqPackage blockedReq;
    unsigned blockedClientId;
    unsigned blockedBankId;

    std::vector<bool> bankReadyVec;
    std::function<unsigned(uint32_t)> arrayAddrToBankFn;

    bool canAccept(unsigned clientId) const;
    void waitForGrant(unsigned clientId);
    bool clientCanRetry(unsigned clientId) const;
    void retryClient(unsigned clientId);
    void scheduleTick();

    // mod: p2s_arbiter
    // ---- Client request/response ports ----

    /** Accept the selected WRITE client's resend. False leaves the
     *  packet with its client; only a pending valid bit is recorded. */
    bool handleRequest(PacketPtr pkt, unsigned clientId);

    /** Accept the selected AutoLoadL read; retain it until Mat data
     *  returns. A refused read remains owned by AutoLoadL. */
    bool handleAutoLoadRequest(PacketPtr pkt);

    /** Accept a selected ACC read/write or BankFetch read. Both use
     *  full 17-bit word addresses; fire is separate from read data. */
    bool handleBankRequest(PacketPtr pkt, unsigned clientId);

    /** Run the clock edge, then ACK an accepted write or register an
     *  accepted read for its eventual Mat response. Repeat each cycle
     *  while a client is waiting or the arbiter is in Block. */
    void processTickEvent();

    /** The WRITE port of client @p clientId. */
    P2SPort &writePort(unsigned clientId);

    std::vector<P2SPort> p2sPorts;
    AutoLoadPort autoLoadPort;
    /** Client 0 (kClientLoadPostProcess): ReadPostProcess bank writes. */
    P2SPort loadPostPort;
    /** Client 1 (kClientBankFetch): BankFetch cache-bank reads. */
    BankPort bankFetchPort;
    /** Client 2 (kClientAccumulator). */
    BankPort accPort;

    /** The one packet accepted by this tick's selected client, held only
     *  until processTickEvent routes its ACK or read-response bookkeeping. */
    PacketPtr grantedPkt = nullptr;

    EventFunctionWrapper tickEvent;
    std::optional<Tick> lastTick;

    // ---- Mat side ----

    /** Mat array index for @p addr, which targets bank @p bankId. */
    unsigned matIndex(uint32_t addr, unsigned bankId) const;

    /** Send an access to its Mat. False means the refused packet is held. */
    bool sendToMat(unsigned clientId, const ReqPackage &req, unsigned bankId);

    /** Mat answered an access. */
    bool handleMatResponse(PacketPtr pkt);

    /** Mark the held packet eligible for resend by the Block-state tick. */
    void matRetry(int idx);

    /** Send the read data back to AutoLoadL and free its slot. */
    void finishAutoLoad(const uint8_t *data);

    std::vector<MatSidePort> matPorts;
    RequestorID requestorId;
    const unsigned matsPerBank;
    const unsigned firstPicMat;
    const unsigned bankBits;

    /** Packet a Mat refused. Global Block prevents any other access from
     *  being granted while it is held, so only one slot can be occupied. */
    struct HeldMatReq
    {
        PacketPtr pkt = nullptr;
        unsigned matIdx = 0;
        bool retryReady = false;
    };
    std::vector<HeldMatReq> heldMat;

    /** The one AutoLoadL read in flight, until its response is sent. */
    PacketPtr autoLoadPkt = nullptr;

    /** Send Mat data to the oldest granted read of this client. */
    void finishBankRead(unsigned clientId, const uint8_t *data);
    std::deque<PacketPtr> &readFifo(unsigned clientId);
    BankPort &bankPort(unsigned clientId);

    /** BankFetch reads granted, waiting for Mat data in grant order. */
    std::deque<PacketPtr> bankFetchReadFifo;

    /** ACC reads already granted, still waiting for their Mat response,
     *  oldest first. RTL Accumulator.scala proc_read sends the next read
     *  right after a grant without waiting for dataReadValid. */
    std::deque<PacketPtr> accReadFifo;
};

} // namespace gem5

#endif // __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
