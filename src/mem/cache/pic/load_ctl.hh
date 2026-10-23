#ifndef __MEM_CACHE_PIC_LOAD_CTL_HH__
#define __MEM_CACHE_PIC_LOAD_CTL_HH__

#include <array>
#include <memory>
#include <vector>

#include "base/types.hh"
#include "mem/cache/pic/pic_dpm_types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/qport.hh"
#include "params/Load_ctl.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * Load_ctl -- port of Load_ctl.scala.
 *
 * Only an arbiter and an FSM; it never touches the data.
 *
 * Arbiter: RRArbiter over the five LoadControllerClients
 * (SysConfig.scala):
 *   0 generalLoad  -- cmd_side (Scheduler LOAD, dataDir = bank)
 *   1 im2colLoad   -- no gem5 module yet, never valid
 *   2 p2sL, 3 p2sR, 4 p2sRT -- p2s_side[0..2] (dataDir = that P2S buffer)
 * A P2S engine sends one ReadReq per row (addr = row vaddr, size =
 * bytes); that becomes a one-row Load_req.
 *
 * FSM (one load at a time):
 *
 *   load_idle -> set_post_process -> req_dma <-> wait_dma_finish
 *                                                      |
 *                         load_idle <- wait_post_process_finish
 *
 * set_post_process sends ReadPostProcess its setup; req_dma sends one row
 * to DMA_read_page_split; wait_dma_finish waits for that row to be read
 * (the page split's response = RTL dma_busy falling); after the last row,
 * wait_post_process_finish waits for ReadPostProcess's response (RTL
 * process_busy falling), then the client is answered.
 *
 * P2S data path: P2S's dma_port is connected here, so the answer to a
 * P2S read must come from here. ReadPostProcess sends that load's words
 * to buf_side; they are collected and returned as the response data.
 * In the RTL these words go straight from ReadPostProcess to the P2S
 * buffer (dataToBuf); this hop only exists because of gem5's
 * one-request-one-response ports.
 *
 * Ports:
 *   cmd_side     <- Scheduler.ld_port
 *   p2s_side[3]  <- P2S_L/P2S_R/P2S_R_T.dma_port
 *   dma_side     -> DMA_read_page_split.req_side
 *   post_side    -> ReadPostProcess.req_side
 *   buf_side     <- ReadPostProcess.buf_side
 */
class Load_ctl : public ClockedObject
{
  public:
    typedef Load_ctlParams Params;

    /** LoadControllerClients in SysConfig.scala. */
    enum Client : unsigned
    {
        kGeneralLoad = 0,
        kIm2colLoad = 1,
        kP2SL = 2,
        kP2SR = 3,
        kP2SRT = 4,
        kNumClients = 5,
    };

  private:
    /** Response port that queues its responses. */
    class InPort : public QueuedResponsePort
    {
      private:
        RespPacketQueue queue;
        Load_ctl *owner;
        int id;  //!< client id, or -1 for buf_side

      public:
        bool needRetry = false;

        InPort(const std::string &name, Load_ctl *owner, int id) :
            QueuedResponsePort(name, queue), queue(*owner, *this),
            owner(owner), id(id)
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
        Load_ctl *owner;

      public:
        OutPort(const std::string &name, Load_ctl *owner) :
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
        SetPostProcess,
        ReqDma,
        WaitDmaFinish,
        WaitPostProcessFinish,
    };

  public:
    explicit Load_ctl(const Params &p);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  private:
    bool handleClientRequest(int client, PacketPtr pkt);
    bool handleBufWord(PacketPtr pkt);
    void handleResponse(const OutPort *port, PacketPtr pkt);
    void handleRetry(const OutPort *port);

    void step();
    void scheduleStep();

    void arbitrate();
    void sendPostProcessReq();
    void sendDmaReq();
    void finishLoad();

    InPort &clientPort(int client);
    void retryClients();

    InPort cmdSidePort;
    std::vector<std::unique_ptr<InPort>> p2sPorts;
    InPort bufSidePort;
    OutPort dmaSidePort;
    OutPort postSidePort;

    RequestorID requestorId;
    EventFunctionWrapper stepEvent;

    State state = State::Idle;

    /** RRArbiter inputs: one waiting request per client. */
    std::array<PacketPtr, kNumClients> pendingPkt{};
    std::array<LoadReq, kNumClients> pendingReq{};
    unsigned lastGrant = kNumClients - 1;

    /** Granted client and its packet. */
    int activeClient = -1;
    PacketPtr activePkt = nullptr;

    /** Load registers (RTL bytes_per_row_reg, offset_byte_reg,
     *  row_num_reg, baseAddr_DRAM_reg, baseAddr_Array_reg,
     *  endAddr_Array_reg, ifPad, padInfo_reg, dataDir). */
    LoadReq cur;
    uint64_t bytesPerRow = 0;
    uint32_t rowNum = 0;
    Addr dramAddr = 0;
    Addr endAddrArray = 0;

    bool postDone = false;
    PacketPtr outRetryPkt = nullptr;

    /** Words for the active P2S read (stand-in for its buffer). */
    std::vector<uint8_t> bufData;
};

}  // namespace gem5

#endif  // __MEM_CACHE_PIC_LOAD_CTL_HH__
