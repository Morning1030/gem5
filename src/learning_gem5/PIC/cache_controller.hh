#ifndef __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
#define __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/qport.hh"
#include "learning_gem5/PIC/Switch/pic_payloads.hh"
#include "mem/cache/base.hh"
#include "mem/cache/tags/base_set_assoc.hh"
#include "mem/packet.hh"
#include "params/CacheController.hh"

namespace gem5
{

class PICTags : public BaseSetAssoc
{
  public:
    using BaseSetAssoc::BaseSetAssoc;

    CacheBlk* findVictim(const CacheBlk::KeyType& key,
                         const std::size_t size,
                         std::vector<CacheBlk*>& evict_blks,
                         const uint64_t partition_id=0) override;
    bool getSetWayValid(const uint32_t setID, const uint32_t wayID);
    Addr getSetWayAddr(const uint32_t setID, const uint32_t wayID);
    Addr getSetWayTag(const uint32_t setID, const uint32_t wayID);
    bool isWayPICMode(const uint32_t wayID) const;
    void setWayPICMode(const uint32_t wayID, bool picMode);
    void setWayRangePICMode(const uint32_t beginWay, const uint32_t endWay, bool picMode);

    // 直接用 (set, way) 拿 block，O(1)
    CacheBlk*
    getBlk(const uint32_t setID, const uint32_t wayID) const
    {
      return static_cast<CacheBlk*>(indexingPolicy->getEntry(setID, wayID));
    }

  private:
    std::vector<bool> PIC_mode; // each element indicates one way
};

class CacheController : public BaseCache
{
  private:
    class PicCtrlPort : public QueuedResponsePort
    {
      private:
        CacheController *owner;
        RespPacketQueue queue;
      public:
        PicCtrlPort(const std::string& name, CacheController *owner)
          : QueuedResponsePort(name, queue), owner(owner),
            queue(*owner, *this) {}

        AddrRangeList getAddrRanges() const override
        { return {}; }

      protected:
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handlePicCtrlReq(pkt); }
        Tick recvAtomic(PacketPtr) override { panic("unimplemented"); }
        void recvFunctional(PacketPtr) override { panic("unimplemented"); }
    };

    class PicFlushPort : public QueuedResponsePort
    {
      private:
        CacheController *owner;
        RespPacketQueue queue;
      public:
        PicFlushPort(const std::string& name, CacheController *owner)
          : QueuedResponsePort(name, queue), owner(owner),
            queue(*owner, *this) {}

        AddrRangeList getAddrRanges() const override { return {}; }

      protected:
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handleFlushReq(pkt); }   // 只有一種，不用分派
        Tick recvAtomic(PacketPtr) override { panic("unimplemented"); }
        void recvFunctional(PacketPtr) override { panic("unimplemented"); }
    };

    // ---- Flush transaction（MSHR with request.control）----
    struct FlushTxn
    {
      PacketPtr pkt = nullptr;   // FlushReq
      Addr      wbAddr = 0;      // 送出的 Release[Data] 的 block 位址
      bool      wbSecure = false;
      bool      wbDirty = false; // ReleaseData or Release

    } flushTxn;

    const Cycles flushLookupLat;
    const Cycles flushMissDoneLat;
    const Cycles flushReleaseAckLat;
    const Cycles flushReleaseDataAckLat;

    EventFunctionWrapper flushScheduleEvent;   // RTL: s_release + dir write INVALID
    EventFunctionWrapper flushDoneEvent;       // RTL: w_releaseack → s_flush (X)
    void processFlushSchedule();
    void processFlushDone();
    CacheBlk *findFlushBlk(uint32_t setID, Addr tag);
    
    const Cycles flushProbeLat;
    const Cycles flushProbeDataLat;
    RequestorID  flushRequestorId;

    EventFunctionWrapper flushReleaseEvent;    // RTL: s_release（等 w_rprobeackfirst）
    void processFlushRelease();

    bool innerHasCopy(Addr addr, bool secure);       // RTL: meta.clients != 0
    bool probeInnerToN(CacheBlk *blk, Addr addr);    // 回傳 true = ProbeAckData
    PacketPtr makeCleanEvict(CacheBlk *blk);         // 等同 Cache::cleanEvictBlk

    EventFunctionWrapper flushWbWaitEvent;    // RTL: 等 outer 接受 Release[Data]
    void processFlushWbWait();

    Tick lastDirQueryTick = MaxTick;
    Tick flushAcceptTick  = MaxTick;

    PicCtrlPort picCtrlPort;
    PicFlushPort picFlushPort;
    PICTags *picTags() const { return static_cast<PICTags*>(tags); }

    // RTL: switchIdle = 0 只擋 sinkA；sinkC（Release）照常進來
    bool switchBlocked = false;
    std::deque<PacketPtr> heldSinkA;            // switch 期間被擋住的 A 類請求
    EventFunctionWrapper sinkAReplayEvent;      // 解除後每拍放行一筆
    void processSinkAReplay();
    static bool isSinkCLike(const PacketPtr pkt);

  protected:
    void recvTimingReq(PacketPtr pkt) override;


  public:
    CacheController(const CacheControllerParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    bool handlePicCtrlReq(PacketPtr pkt);
    bool handleQueryWayState(PacketPtr pkt);
    bool handleDrainQuery(PacketPtr pkt);
    bool handleFlushReq(PacketPtr pkt);
    bool handleWayMode(PacketPtr pkt);
    // 不用 setBlocked：它會擋住整個 cpuSidePort，連 L1 的 writeback 也進不來
    void blockForSwitch() { switchBlocked = true; }
    void unblockForSwitch()
    {
        if (switchBlocked) {
            switchBlocked = false;
            if (!heldSinkA.empty() && !sinkAReplayEvent.scheduled())
                schedule(sinkAReplayEvent, clockEdge(Cycles(1)));
        }
    }
    bool handleSwitchBusy(PacketPtr pkt);

    // RTL: Scheduler 的 isScheNoOtherWorks（組合邏輯）
    bool isScheNoOtherWorks() const
    {
        return mshrQueue.isEmpty() && writeBuffer.isEmpty() &&
               flushTxn.pkt == nullptr;
    }

    // RTL: runtime_PIC_Mode_levels 暫存器直接接到 Directory（不是訊息）
    void setPicWays(uint32_t beginWay, uint32_t endWay, bool pic)
    {
        picTags()->setWayRangePICMode(beginWay, endWay, pic);
    }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
