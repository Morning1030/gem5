//because c++ dont allow one class to be defined twice, so check
#ifndef __MEM_CACHE_PIC_PIC_DMA_ENGINE_HH__  //if not defined yet
#define __MEM_CACHE_PIC_PIC_DMA_ENGINE_HH__ // then define it

//c++ standard version
#include <deque>       //fifo
#include <unordered_map>
#include <vector>

// gem5 header files
#include "base/types.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/DMAEngine.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Expand a two-dimensional descriptor into physically contiguous chunks.
 *
 * Pure address arithmetic -- page/block/max-chunk splitting only cares
 * about the virtual address (page boundaries fall out of its low bits),
 * so this has no dependency on translation and can be gtested without
 * instantiating anything. Translation happens later, per chunk, in
 * DMAEngine::issue() right before the chunk is sent, so TLB cost (and,
 * eventually, TLB misses) land at the moment they actually occur instead
 * of all being paid up front. The cases that matter are
 *
 *   - dramVaddr not aligned to a cache block
 *   - a row landing exactly on a page boundary
 *   - bytesPerRow larger than a page
 *   - rowStrideBytes == bytesPerRow (fully contiguous; chunks may then be
 *     merged across rows, which is precisely why t=false is fast)
 *   - rowStrideBytes < bytesPerRow (overlapping rows; rejected)
 *
 * @param blockBytes Requests must never span a cache block, or the LLC
 *                   will panic.  This is a hard gem5 constraint, not an
 *                   optimisation.
 * @return Empty deque if the descriptor is malformed (zero rows/
 *         bytesPerRow, or a rejected rowStride). Never empty otherwise --
 *         a valid descriptor always yields at least one chunk per row.
 */
std::deque<AddrChunk> splitRequests(const DMADescriptor &desc,
                                    uint32_t maxChunkBytes,
                                    uint32_t blockBytes);

/**
 * mod: p2s_arbiter
 * DMAEngine -- SimObject that issues DMA reads for P2S engines and
 * reassembles the responses.
 *
 * This used to be split across two files: a port-less, clock-less engine
 * here plus a separate DMA SimObject (learning_gem5/PIC/dma.{hh,cc}) that
 * supplied ports and drove the engine through a Host struct of callbacks.
 * They've been merged back into this one class on request -- the ports
 * and the chunk-splitting/reassembly logic now live together, and the
 * Host indirection is gone (this class just calls its own ports/methods
 * directly). splitRequests() above is untouched and still usable in
 * isolation without a SimObject.
 *
 * Port topology -- modeled on learning_gem5/part2/simple_cache.hh's
 * CPUSidePort/MemSidePort pattern:
 *   - p2sPorts    : VectorResponsePort, one connection per P2S engine.
 *                   Connection index doubles as P2SDest (0=P2S_L,
 *                   1=P2S_R, 2=P2S_R_T) -- each P2S engine's own
 *                   RequestPort (its "dma_port") connects to one index.
 *   - memSidePort : RequestPort into the real memory hierarchy.
 *
 * Request/response granularity -- **one gem5 request from P2S = exactly
 * one row, one response**: a P2S engine sends one ReadReq-shaped packet
 * per row it wants (address = row's DRAM address, size = bytesPerRow),
 * and gets back exactly one response packet carrying that row's bytes.
 * handleRequest() turns that single request into a P2SOpState with
 * numRows=1, rowsPerGroup=1 (so groupReady() fires exactly once per op)
 * and hands the reassembled bytes straight back on the same packet via
 * pkt->makeResponse().
 *
 * This was a deliberate choice, not an arbitrary default: gem5's
 * RequestPort/ResponsePort pair is fundamentally a 1-request/1-response
 * protocol -- a ResponsePort has no channel to push several independent
 * replies for one accepted request. Pinning every op to one row/one
 * group each is what makes that port pair viable at all: the engine
 * still earns its keep for that single row (page-crossing chunk
 * splitting, serialized TLB latency, sender-state-tagged reassembly --
 * see PICDMASenderState in pic_dpm_types.hh -- so responses for
 * *different* concurrently-outstanding rows can safely land out of
 * order), it just never needs its own multi-group batching for a single
 * op. If P2S ever needs to batch multiple rows under one request, that
 * needs a second, DMA-initiated port (or a redesigned protocol), not
 * just a parameter change here.
 */
class DMAEngine : public ClockedObject
{
  public:
    // mod: p2s_arbiter
    // Must come before anything else in the class that names Params:
    // member declarations (unlike member function bodies) are parsed
    // sequentially, not with full-class lookup, so a Params use above
    // this line would silently resolve to the inherited
    // ClockedObject::Params instead.
    typedef DMAEngineParams Params;

  private:
    /**
     * Port on the P2S-side that receives row-read requests. One instance
     * per connection (vector port); id doubles as P2SDest.
     */
    class P2SPort : public ResponsePort
    {
      private:
        int id;
        DMAEngine *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        P2SPort(const std::string &name, int id, DMAEngine *owner) :
            ResponsePort(name), id(id), owner(owner), needRetry(false),
            blockedPacket(nullptr)
        { }

        /** Send @p pkt (already a response) back to this port's P2S
         *  engine, buffering it if the port refuses. */
        void sendPacket(PacketPtr pkt);

        /** Not address-decoded -- this is a private point-to-point link
         *  to one P2S engine, not a bus-visible memory range. */
        AddrRangeList getAddrRanges() const override;

        /** Send a retry to the P2S engine if one is owed and we're free
         *  to accept a new request now. */
        void trySendRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Port on the memory-side. Thin pass-through: this class tracks its
     * own held-for-retry packet (retryPkt/blocked, unchanged from before
     * the merge) internally, so this port must NOT also buffer.
     */
    class MemSidePort : public RequestPort
    {
      private:
        DMAEngine *owner;

      public:
        MemSidePort(const std::string &name, DMAEngine *owner) :
            RequestPort(name), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override { }
    };

  public:
    explicit DMAEngine(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    /**
     * Register a new op: split its descriptor into chunks, size its
     * reassembly buffers, and take ownership of it.
     *
     * @param op  Caller-filled state -- desc, dest, and rowsPerGroup (plus
     *            whatever downstream tags it wants carried through, e.g.
     *            bitWidth/transpose) must already be set. opId is assigned
     *            here and does not need to be filled in beforehand.
     * @return    The assigned opId, or 0 if the descriptor is malformed.
     */
    uint32_t startOp(P2SOpState op);

    /**
     * Issue as many chunks as the outstanding limit, the buffer credits
     * and the TLB port allow. Returns the tick at which it should next
     * be woken, or MaxTick if there is nothing pending.
     */
    Tick issue();

    /** Consume one DMA read response, split it into the corresponding
     *  op, and reassemble the data into the buffer. Returns true if a
     *  group completed (and, since every op here is one row/one group,
     *  that also means the op is done and its response has been sent). */
    bool handleResponse(PacketPtr pkt);

    /**
     * Port told us it can accept a request again. If a packet is
     * waiting from a previous refusal (retryPkt), resend it -- blocked
     * only clears once that resend actually goes through, not just
     * because the port pinged us.
     */
    void retry();

    /** Look up an in-flight op by id. */
    P2SOpState *findOp(uint32_t opId);

  private:
    /** Handle one row-read request from P2S engine @p port_id. Builds a
     *  single-row P2SOpState, starts it, and kicks off issuing. Returns
     *  false only if the descriptor was malformed (pkt size 0). */
    bool handleRequest(PacketPtr pkt, int port_id);

    /** Send a completed row's response back out P2SPort @p port_id. */
    void sendResponse(PacketPtr pkt, int port_id);

    /** Run one issue() pass; reschedules itself at whatever Tick issue()
     *  says to wake up at, if any. */
    void processIssueEvent();

    /** VA -> PA. Placeholder: identity-mapped (SE-mode-style
     *  passthrough). No MMU/TLB hookup exists yet for this DMA path. */
    bool translate(Addr vaddr, Addr &paddr);

    /** Build (but don't send) a ReadReq packet for chunk @p c at
     *  physical address @p paddr. */
    PacketPtr buildReadPacket(Addr paddr, const AddrChunk &c);

    /** Send a request packet out memSidePort. No buffering here on
     *  purpose -- this class already holds retryPkt itself and calls
     *  retry() once the port can accept again (see MemSidePort::
     *  recvReqRetry()); a second buffer on the port would double up. */
    bool sendDMAReq(PacketPtr pkt);

    /** Tick at which the next clock edge lands. */
    Tick nextCycleTick();

    /** Reassembly window, in wordline groups, for destination @p dest.
     *  Every op here is exactly one row/one group (see handleRequest),
     *  so this only ever needs to admit that one group -- if P2S starts
     *  batching multiple rows into one op, this needs real per-dest
     *  buffer-depth tracking instead of a constant. */
    uint32_t bufRows(P2SDest dest);

    /** Called once every descriptor row making up @p group has arrived
     *  and been reassembled into op.buf -- turns the original P2S
     *  request packet into the response and sends it back. */
    void groupReady(P2SOpState &op, uint32_t group);

    std::vector<P2SPort> p2sPorts;
    MemSidePort memSidePort;

    RequestorID requestorId;

    EventFunctionWrapper issueEvent;

    /** opId -> the P2S request packet that op was started for, so
     *  groupReady() knows which packet to turn into a response and which
     *  P2SPort to send it out (via op.dest). */
    std::unordered_map<uint32_t, PacketPtr> pendingReqPkt;

    // ---- Static config (from Params; unchanged in spirit from the old
    // Host struct's scalar fields) ----
    const uint32_t chunkBytes;      //!< Max bytes per single DMA request.
    const uint32_t blockBytes;      //!< Cache block size (chunk ceiling).
    const uint32_t maxOutstanding;  //!< Concurrent in-flight reads allowed.
    const bool     tlbPipelined = false;
    const Tick     tlbLatencyTicks; //!< Cost of one (serialised) translation.

    std::unordered_map<uint32_t, P2SOpState> opTable;
    uint32_t nextOpId = 1;

    /** Single shared TLB port: translations serialise behind this. */
    Tick tlbFreeTick = 0;

    bool blocked = false;

    /** Retry slot for a packet the downstream port refused. */
    PacketPtr retryPkt = nullptr;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_PIC_DMA_ENGINE_HH__
// if someone has already included(defined) this header, don't do it again
