#ifndef __LEARNING_GEM5_DMA_WRITE_PAGE_SPLIT_HH__
#define __LEARNING_GEM5_DMA_WRITE_PAGE_SPLIT_HH__

#include <vector>

// gem5 header files
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/DMAWritePageSplit.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class DMAWritePageSplit : public ClockedObject
{
  public:
    enum class PICPortID {TLB, DRAM};
  private:
    /** Port Load_ctl sends row reads on. */
    class CPUSidePort : public ResponsePort
    {
      private:
        DMAWritePageSplit *owner;
      protected:
        Tick recvAtomic(PacketPtr pkt) override {panic("recvAtomic unimplemented.");}
        void recvFunctional(PacketPtr pkt) override {panic("recvFunctional unimplemented.");}
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override {return {};}

      public:
        CPUSidePort(const std::string &name, DMAWritePageSplit* owner);
        PacketPtr blockedPacket;
        bool needRetry;
        void sendPacket(PacketPtr pkt);
    };

    /** Request port shared by data_side, tlb_side and mem_side. */
    class MemSidePort : public RequestPort
    {
      private:
        DMAWritePageSplit *owner;
        PICPortID picPortID;
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
      public:
        MemSidePort(const std::string &name, DMAWritePageSplit *owner, PICPortID picPortID);
        PacketPtr blockedPacket;
        void sendPacket(PacketPtr pkt);
    };

    CPUSidePort instPort;
    MemSidePort tlbPort;
    MemSidePort dramPort;
    RequestorID requestorId;
    PacketPtr pendingReqPkt;
    EventFunctionWrapper tlbQueryEvent;
    EventFunctionWrapper issueWriteEvent;

    Addr vaddrPtr;
    Addr paddrPtr;
    uint32_t validNBytes;
    std::vector<uint8_t> cacheBankData;

  public:
    DMAWritePageSplit(const DMAWritePageSplitParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PICPortID picPortID, PacketPtr pkt);
    void processTlbQueryEvent();
    void processIssueWriteEvent();
};

}  // namespace gem5

#endif
