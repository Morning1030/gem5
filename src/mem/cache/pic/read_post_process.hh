#ifndef __MEM_CACHE_PIC_READ_POST_PROCESS_HH__
#define __MEM_CACHE_PIC_READ_POST_PROCESS_HH__

#include <deque>

#include "base/types.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/qport.hh"
#include "params/ReadPostProcess.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * ReadPostProcess -- port of ReadPostProcess.scala (load_post_process in
 * PolymorPIC_main_in_cache.scala).
 *
 * Load_ctl first sends a setup (ReadPostProcessReq: bank base/end
 * address, padding, dataDir). The data for the whole load then arrives
 * from DMA_read_page_split as 64-bit StreamBeats with a keep mask. The
 * Aligner packs the kept bytes into full 64-bit words, and each word goes
 * to:
 *
 *   dataDir == Bank : AccessBankArb as a WRITE to array address
 *                     bank_baseAddr, bank_baseAddr + 1, ...
 *   dataDir == P2S* : the P2S engine's input buffer. P2S's request port
 *                     is connected to Load_ctl, so the words go to
 *                     Load_ctl.buf_side, which answers P2S.
 *
 * The setup request is answered when the last word has been accepted --
 * the RTL io.busy falling, which Load_ctl's wait_post_process_finish
 * waits on.
 *
 * Ports:
 *   req_side  <- Load_ctl.post_side          (RTL io.req / io.busy)
 *   data_side <- DMA_read_page_split.data_side (RTL io.unaligned_dataIn)
 *   bank_side -> AccessBankArb.load_post_side (RTL io.accessBank)
 *   buf_side  -> Load_ctl.buf_side           (RTL io.dataToBuf*)
 *
 * Not implemented: PaddingUnit. A setup with padding and padSize > 0
 * panics.
 */
class ReadPostProcess : public ClockedObject
{
  public:
    typedef ReadPostProcessParams Params;

  private:
    /** Response port that queues its responses (req_side, data_side). */
    class InPort : public QueuedResponsePort
    {
      private:
        RespPacketQueue queue;
        ReadPostProcess *owner;

      public:
        bool needRetry = false;

        InPort(const std::string &name, ReadPostProcess *owner) :
            QueuedResponsePort(name, queue), queue(*owner, *this),
            owner(owner)
        { }

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override { }
        bool recvTimingReq(PacketPtr pkt) override;
    };

    class OutPort : public RequestPort
    {
      private:
        ReadPostProcess *owner;

      public:
        OutPort(const std::string &name, ReadPostProcess *owner) :
            RequestPort(name), owner(owner)
        { }

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override { }
    };

    enum class State { Idle, WaitFinish };

    /** One aligned 64-bit word waiting to go out. */
    struct Word
    {
        uint64_t data;
        bool last;
    };

  public:
    explicit ReadPostProcess(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    bool handleRequest(const InPort *port, PacketPtr pkt);
    bool handleSetup(PacketPtr pkt);
    bool handleBeat(PacketPtr pkt);
    void align(const StreamBeat &beat);

    void sendWord();
    void handleAck(PacketPtr pkt);
    void outRetry();
    void finish();

    InPort reqSidePort;
    InPort dataSidePort;
    OutPort bankSidePort;
    OutPort bufSidePort;

    RequestorID requestorId;
    EventFunctionWrapper sendEvent;

    State state = State::Idle;

    /** Setup registers (RTL baseAddr_Array_reg, endAddr_Array_reg,
     *  dataDir_reg). */
    PacketPtr setupPkt = nullptr;
    uint64_t arrayAddr = 0;
    uint64_t endAddr = 0;
    DataDir dataDir = DataDir::Bank;

    /** Aligner registers (RTL data / nbytes). Holds 0..7 bytes. */
    uint64_t alignData = 0;
    unsigned alignBytes = 0;

    /** Aligner output. */
    std::deque<Word> words;

    /** A word sent but not yet acked; one at a time like Decoupled. */
    bool waitingAck = false;
    PacketPtr outRetryPkt = nullptr;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_READ_POST_PROCESS_HH__
