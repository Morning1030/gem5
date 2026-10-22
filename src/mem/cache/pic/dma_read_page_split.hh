//because c++ dont allow one class to be defined twice, so check
#ifndef __MEM_CACHE_PIC_DMA_READ_PAGE_SPLIT_HH__  //if not defined yet
#define __MEM_CACHE_PIC_DMA_READ_PAGE_SPLIT_HH__ // then define it

//c++ standard version
#include <deque>       //fifo
#include <map>
#include <vector>

// gem5 header files
#include "base/types.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/DMA_read_page_split.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Expand a two-dimensional descriptor into physically contiguous chunks.
 *
 * Pure address arithmetic -- page/block/max-chunk splitting only cares
 * about the virtual address (page boundaries fall out of its low bits),
 * so this has no dependency on translation and can be gtested without
 * instantiating anything. The cases that matter are
 *
 *   - dramVaddr not aligned to a cache block
 *   - a row landing exactly on a page boundary
 *   - bytesPerRow larger than a page
 *   - rowStrideBytes == bytesPerRow (fully contiguous; chunks may then be
 *     merged across rows, which is precisely why t=false is fast)
 *   - rowStrideBytes < bytesPerRow (overlapping rows; rejected)
 *
 * DMA_read_page_split only ever calls it with one row inside one page.
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
 * DMA_read_page_split -- port of DMA_read_page_split.scala.
 *
 * Serves one row read (DMAReadReq) at a time, the same as the RTL:
 *
 *   Idle -> PreProcess -> QueryTLB -> QueryWaitResp -> DMAReading
 *             ^                                           |
 *             +------------ next page segment ------------+
 *
 * PreProcess cuts the next segment at the page boundary; QueryTLB asks
 * TLBReqScheduler for its page; DMAReading reads it from DRAM. The RTL
 * hands each segment to icenet's StreamReader (nXacts=4, maxBytes=64);
 * here the segment is cut into <= chunk_bytes reads that never cross a
 * cache block, with at most max_outstanding in flight.
 *
 * Responses are put back in address order and sent out data_side as
 * 64-bit StreamBeats (data, keep, last), like the RTL dataOut stream
 * after its 6-entry data_queue. last is set on the final beat of the
 * final segment of a request with lastRow set. The request on req_side
 * is answered when the row has been fully read and queued -- the RTL
 * io.busy falling, which is what Load_ctl waits on.
 *
 * Ports:
 *   req_side  <- Load_ctl.dma_side       (RTL io.req / io.busy)
 *   data_side -> ReadPostProcess.data_side (RTL io.dataOut)
 *   tlb_side  -> TLBReqScheduler.cpu_side[0]
 *   mem_side  -> memory
 *
 * Not modeled: the TileLink beat timing inside StreamReader.
 */
class DMA_read_page_split : public ClockedObject
{
  public:
    // mod: p2s_arbiter
    // Must come before anything else in the class that names Params:
    // member declarations (unlike member function bodies) are parsed
    // sequentially, not with full-class lookup, so a Params use above
    // this line would silently resolve to the inherited
    // ClockedObject::Params instead.
    typedef DMA_read_page_splitParams Params;

  private:
    /** Port Load_ctl sends row reads on. */
    class ReqSidePort : public ResponsePort
    {
      private:
        DMA_read_page_split *owner;

      public:
        bool needRetry = false;
        PacketPtr blockedResp = nullptr;

        ReqSidePort(const std::string &name, DMA_read_page_split *owner) :
            ResponsePort(name), owner(owner)
        { }

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /** Request port shared by data_side, tlb_side and mem_side. */
    class OutPort : public RequestPort
    {
      private:
        DMA_read_page_split *owner;

      public:
        OutPort(const std::string &name, DMA_read_page_split *owner) :
            RequestPort(name), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override { }
    };

    enum class State
    {
        Idle,
        PreProcess,
        QueryTLB,
        QueryWaitResp,
        DMAReading,
    };

  public:
    explicit DMA_read_page_split(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    bool handleRequest(PacketPtr pkt);
    void reqRespRetry();

    /** Route a response or retry by which port it came in on. */
    bool handleResponse(const OutPort *port, PacketPtr pkt);
    void handleRetry(const OutPort *port);

    /** One FSM step; reschedules itself while there is work. */
    void step();
    void scheduleStep();

    void preProcess();
    void sendTlbQuery();
    bool handleTlbResponse(PacketPtr pkt);
    void issueReads();
    bool handleMemResponse(PacketPtr pkt);

    /** Turn chunks that are back, in order, into beats. */
    void drainChunks();
    void pushBeats(Addr paddr, const uint8_t *data, uint32_t size,
                   bool lastChunk);

    /** Send queued beats out data_side. */
    void sendBeats();

    /** Row done: answer Load_ctl and go idle. */
    void finishRequest();

    ReqSidePort reqSidePort;
    OutPort dataSidePort;
    OutPort tlbSidePort;
    OutPort memSidePort;

    RequestorID requestorId;
    EventFunctionWrapper stepEvent;
    EventFunctionWrapper sendEvent;

    const uint32_t chunkBytes;
    const uint32_t blockBytes;
    const uint32_t maxOutstanding;
    const bool skipSamePage;

    State state = State::Idle;

    /** Current request (RTL nByte/baseAddr_DRAM registers). */
    PacketPtr activeReq = nullptr;
    Addr finalVaddr = 0;
    bool lastRow = false;

    /** RTL v_addr_ptr / cur_block_len. */
    Addr vaddrPtr = 0;
    uint32_t curBlockLen = 0;

    /** RTL p_page_id / v_page_valid_id / p_page_id_valid. */
    bool pageValid = false;
    Addr vpn = 0;
    Addr ppn = 0;

    PacketPtr tlbRetryPkt = nullptr;

    /** Chunks of the current segment. */
    std::deque<AddrChunk> chunks;
    uint32_t nextIssue = 0;
    uint32_t nextDrain = 0;
    uint32_t outstanding = 0;
    std::map<uint32_t, PacketPtr> arrived;
    PacketPtr memRetryPkt = nullptr;

    /** RTL data_queue, drained one beat per cycle. */
    std::deque<StreamBeat> beatQueue;
    bool dataBlocked = false;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_DMA_READ_PAGE_SPLIT_HH__
// if someone has already included(defined) this header, don't do it again
