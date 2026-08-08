//because c++ dont allow one class to be defined twice, so check
#ifndef __MEM_CACHE_PIC_PIC_DMA_ENGINE_HH__  //if not defined yet
#define __MEM_CACHE_PIC_PIC_DMA_ENGINE_HH__ // then define it

//c++ standard version
#include <deque>       //fifo
#include <functional>  //Host's std::function hooks
#include <unordered_map>

// gem5 header files
#include "base/types.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"

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
 * Issues DMA reads for a descriptor and reassembles the responses.
 *
 * DMAEngine owns no port, no clock, and nothing about the wider system --
 * everything it needs that lives outside itself (sending a request,
 * translating an address, knowing the current tick, handing off a
 * completed group) goes through the Host hooks passed in at construction.
 * Whatever embeds a DMAEngine -- a DPM SimObject, a unit test double,
 * anything -- just fills in a Host and calls startOp() / issue() /
 * handleResponse() / retry() from its own port callbacks and scheduling.
 * DMAEngine never assumes anything about what that caller is; it only
 * knows the shape of the hooks it was given.
 *
 * Deliberately knows nothing about bit width, transposition, or the PIC
 * array: it consumes DMADescriptor and produces filled rows.  P2S_L,
 * P2S_R and P2S_R_T all share this one engine, differing only in how they
 * parameterise the descriptor.
 */
class DMAEngine
{
  public:
    /**
     * Everything DMAEngine needs but doesn't own.  The scalar fields are
     * read-only config, read once per call; the std::function fields are
     * the engine's only way to reach outside itself.
     */
    struct Host
    {
        // ---- Static config ----
        uint32_t chunkBytes      = 0;  //!< Max bytes per single DMA request.
        uint32_t blockBytes      = 0;  //!< Cache block size (chunk ceiling).
        uint32_t maxOutstanding  = 0;  //!< Concurrent in-flight reads allowed.
        bool     tlbPipelined    = false;
        Tick     tlbLatencyTicks = 0;  //!< Cost of one (serialised) translation.

        // ---- Callbacks ----

        /** VA -> PA.  Return false on a translation fault. */
        std::function<bool(Addr vaddr, Addr &paddr)> translate;

        /** Build (but don't send) a ReadReq packet for chunk @p c at
         *  physical address @p paddr. */
        std::function<PacketPtr(Addr paddr, const AddrChunk &c)>
            buildReadPacket;

        /** Send a request packet out the real port.  Return false if the
         *  port refused it -- DMAEngine holds the packet and calls
         *  retry() once told the port can accept again. */
        std::function<bool(PacketPtr pkt)> sendDMAReq;

        /** Tick at which the next clock edge lands, for scheduling
         *  backoff (outstanding-limit / TLB stalls). */
        std::function<Tick()> nextCycleTick;

        /** Reassembly window, in wordline groups, for a given destination
         *  buffer.  Bounds how far ahead of group completion the engine
         *  is allowed to keep issuing reads. */
        std::function<uint32_t(P2SDest dest)> bufRows;

        /** Called once every descriptor row making up @p group has
         *  arrived and been reassembled into op.buf. */
        std::function<void(P2SOpState &op, uint32_t group)> groupReady;
    };

    explicit DMAEngine(Host host);

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
     * and the TLB port allow.  Returns the tick at which it should next
     * be woken, or MaxTick if there is nothing pending.
     */
    Tick issue();

    /** Consume one DMA read response. split the response packet into the
     *  corresponding op, and reassemble the data into the buffer, Returns
     *  true if a group completed. */
    bool handleResponse(PacketPtr pkt);

    /**
     * Port told us it can accept a request again.  If a packet is
     * waiting from a previous refusal (retryPkt), resend it -- blocked
     * only clears once that resend actually goes through, not just
     * because the port pinged us.
     */
    void retry();

    /** Look up an in-flight op by id. */
    P2SOpState *findOp(uint32_t opId);

  private:
    Host host;

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
