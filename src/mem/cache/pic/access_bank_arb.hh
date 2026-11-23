/**
 * AccessBankArb: the one shared path from the seven PIC clients into the
 * cache-bank Mats (RTL AccessBank_Arb, bankAccessScheduler.scala).
 * Clients: 0 LoadPostProcess, 1 BankFetch, 2 ACC,
 * 3 AutoLoadVec, 4 P2S_L, 5 P2S_R, 6 P2S_R_T.
 *
 * gem5 only: the RTL RRArbiter and its per-client valid/ready handshake are
 * replaced by ONE request FIFO. recvTimingReq on every client port always
 * returns true: the request is queued in arrival order and no client ever
 * waits for a grant or gets a retry. Each cycle the head of the FIFO goes to
 * its Mat, so the bank side still sees at most one access per cycle, as in
 * the RTL.
 *
 * Head-of-line wait (RTL state block): if the head's bank is in use by the
 * cache path (setBankReady) or its Mat refuses the access, the head stays at
 * the front and nothing behind it moves until it lands.
 *
 * Responses: a WRITE is ACKed when its Mat accepts it, so an ACK means the
 * data is in the array; a client that counts ACKs knows when every write it
 * sent has landed. A READ is answered with the Mat's data. With one FIFO,
 * a read queued after a write to the same word always sees that write.
 *
 * Request fields are {addr:17, optype:1, writeData:64} in RTL. Addr layout:
 * [mat_in_bank:4 | bank:2 | array_in_mat:2 | wordline:9]. Mat ports are
 * indexed (mat_in_bank - first_pic_mat) * num_banks + bank and receive
 * only the 11-bit within-Mat word index. Without Mat ports (stub mode),
 * writes are ACKed and reads return zero when they reach the head.
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
    kClientLoadPostProcess = 0, // load: DRAM data written into the cache banks
    kClientBankFetch       = 1, // store: cache-bank data read out to DRAM
    kClientAccumulator     = 2,
    kClientAutoLoadVec     = 3, // cal: L vectors read into a Mat's vector register
    kClientP2S_L           = 4,
    kClientP2S_R           = 5,
    kClientP2S_R_T         = 6,
};

/** Total clients sharing the single AccessBank_Arb in the paper design. */
static constexpr unsigned kNumBankArbClients = 7;

class AccessBankArb : public ClockedObject
{
  public:
    // Must come before anything else that names Params -- see the
    // identical note in dma_read_page_split.hh's DMA_read_page_split class.
    typedef AccessBankArbParams Params;

    explicit AccessBankArb(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    /** Scala: AccessBank_Arb io.mem_ready(bankID). CacheController sets a
     *  bank not ready while the cache path uses it. Defaults to ready. */
    void setBankReady(unsigned bankId, bool ready);
    bool bankReady(unsigned bankId) const;

    /**
     * Install the array-address -> bank-index decode function. Defaults to
     * get_bankID() (SysConfig.scala:282-305) for the paper's default
     * config: addr[12:11]. Override if num_banks/matPerBank/core_addrLen
     * differ from that default.
     */
    void setArrayAddrToBank(std::function<unsigned(uint32_t)> fn);

  private:
    /**
     * One client's port (Scala: io.request(clientId), Decoupled(ReqPackage),
     * plus the read-data return). All seven clients use this class; the
     * payload format depends on the client (see toReqPackage()).
     */
    class ClientPort : public ResponsePort
    {
      private:
        AccessBankArb *owner;
        const unsigned clientId;
        /** Responses the client refused, oldest first. */
        std::deque<PacketPtr> respQueue;

      public:
        ClientPort(const std::string &name, unsigned clientId,
                   AccessBankArb *owner) :
            ResponsePort(name), owner(owner), clientId(clientId)
        { }

        /** Send an ACK or read response, in order behind refused ones. */
        void sendPacket(PacketPtr pkt);

        /** Not address-decoded -- a private point-to-point link. */
        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

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

    /** One queued client request: the client's own packet (answered when
     *  the access lands) and the decoded RTL request fields. */
    struct QueuedReq
    {
        unsigned clientId;
        PacketPtr pkt;
        ReqPackage req;
    };

    /** gem5 only: decode a client packet into the RTL request fields. */
    ReqPackage toReqPackage(unsigned clientId, PacketPtr pkt) const;

    /** gem5 only: queue a client request; never refuses. */
    bool enqueue(PacketPtr pkt, unsigned clientId);

    /** Run processTickEvent this cycle if it has not run yet, else next. */
    void scheduleTick();

    /** Scala: AccessBank_Arb state access/block -- send the FIFO head to
     *  its Mat once its bank is ready; one head per cycle. */
    void processTickEvent();

    /** Send @p head to its Mat. False: the Mat refused, the packet is held
     *  and the head waits for the Mat's retry. */
    bool issueHead(const QueuedReq &head, unsigned bankId);

    /** Answer the client: ACK a write, or fill in a read's data. */
    void respond(unsigned clientId, PacketPtr pkt, const uint8_t *readData);

    /** Mat array index for @p addr, which targets bank @p bankId. */
    unsigned matIndex(uint32_t addr, unsigned bankId) const;

    /** Mat answered an access. */
    bool handleMatResponse(PacketPtr pkt);

    /** The Mat holding the refused head can take it now. */
    void matRetry(int idx);

    /** Indexed by BankArbClient. */
    std::vector<ClientPort> clientPorts;
    std::vector<MatSidePort> matPorts;

    /** gem5 only: every accepted request, oldest first. */
    std::deque<QueuedReq> reqQueue;
    /** Largest reqQueue length seen, for the debug trace. */
    size_t maxQueueDepth = 0;

    /** The head's Mat packet after its Mat refused it (only the head can
     *  be refused, so at most one). */
    PacketPtr heldMatPkt = nullptr;
    unsigned heldMatIdx = 0;
    bool heldRetryReady = false;

    std::vector<bool> bankReadyVec;
    std::function<unsigned(uint32_t)> arrayAddrToBankFn;

    RequestorID requestorId;
    const unsigned matsPerBank;
    const unsigned firstPicMat;
    const unsigned bankBits;

    EventFunctionWrapper tickEvent;
    std::optional<Tick> lastTick;
};

} // namespace gem5

#endif // __MEM_CACHE_PIC_ACCESS_BANK_ARB_HH__
