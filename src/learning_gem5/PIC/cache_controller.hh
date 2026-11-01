#ifndef __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
#define __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include "mem/qport.hh"
#include "learning_gem5/PIC/Switch/pic_payloads.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/pic/access_bank_arb.hh"
#include "mem/cache/tags/pic_llc_tags.hh"
#include "mem/packet.hh"
#include "params/CacheController.hh"

namespace gem5
{

class SwitchController;

class CacheController : public Cache
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

    // MSHR drain 時要通知的 switch；SwitchController::init() 註冊，沒接就是 nullptr
    SwitchController *switchCtrl = nullptr;

    // RTL: switchIdle = 0 只擋 sinkA；sinkC（Release）照常進來
    bool switchBlocked = false;
    // switch_req fire 那拍就知道下一拍（pre_check）switchIdle = 0；
    // preCheckEvent 的優先權較低，要靠這個讓那拍一開始就擋住
    Tick sinkABlockFrom = MaxTick;
    bool sinkABlocked() const
    { return switchBlocked || curTick() >= sinkABlockFrom; }
    // switch 期間被擋住的 A 類請求。RTL 是 sinkA.ready = 0 讓上游卡住；
    // gem5 的 cpu_side 是一個 port，回 false（setBlocked）會連 sinkC 的
    // writeback 一起擋，所以收下來排隊：上游以為已被接受，互連上的
    // head-of-line blocking 沒有模擬到
    std::deque<PacketPtr> heldSinkA;
    const unsigned maxHeldSinkA;                // 0 = 不限
    struct PicStats : public statistics::Group
    {
        explicit PicStats(statistics::Group *parent);
        statistics::Scalar heldSinkAPeak;
    } picStats;
    EventFunctionWrapper sinkAReplayEvent;      // 解除後每拍放行一筆
    void processSinkAReplay();
    static bool isSinkCLike(const PacketPtr pkt);

    // ---- Cache / PIC bank 衝突（RTL: BankSellPIC L71、L112-116）----
    // cache 路徑用到 bank 時 picMem_ready = false，cache 永遠優先，
    // PIC 請求要等。nullptr = 不模擬（AccessBankArb 的 bank 永遠 ready）
    AccessBankArb *bankArb;
    const Cycles bankBusyDelay;            // 收到請求 → 第一個 beat 用 bank
    const unsigned beatBytes;              // 一個 beat 幾 bytes
    std::vector<unsigned> bankBusyCnt;     // 多筆 cache 存取可能重疊，用計數器
    // 讀寫 blk 的 [offset, offset + size) 時，標記用到的 bank
    void occupyBanks(Addr offset, unsigned size);
    // bank 在 [start, end) 期間忙碌
    void markBank(unsigned bank, Tick start, Tick end);

  protected:
    void recvTimingReq(PacketPtr pkt) override;

    // ---- 碰到 cache 資料的地方 → occupyBanks ----
    // handleFill / handleSnoop 不是 virtual：fill 改由 recvTimingResp 處理；
    // snoop 回資料（handleSnoop）這次先不算
    void satisfyRequest(PacketPtr pkt, CacheBlk *blk,
                        bool deferred_response = false,
                        bool pending_downgrade = false) override;
    void recvTimingResp(PacketPtr pkt) override;
    bool access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                PacketList &writebacks) override;
    // override 單參數版本會遮住 BaseCache::evictBlock(blk, writebacks)
    using Cache::evictBlock;
    [[nodiscard]] PacketPtr evictBlock(CacheBlk *blk) override;


  public:
    CacheController(const CacheControllerParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    // constructor 已確認 tags 是 PICLLCTags
    PICLLCTags *picTags() const { return static_cast<PICLLCTags*>(tags); }

    bool handlePicCtrlReq(PacketPtr pkt);
    bool handleQueryWayState(PacketPtr pkt);
    bool handleFlushReq(PacketPtr pkt);
    // 不用 setBlocked：它會擋住整個 cpuSidePort，連 L1 的 writeback 也進不來
    void blockForSwitch()
    {
        switchBlocked = true;
        sinkABlockFrom = MaxTick;
    }
    // switch_req fire 時呼叫：從 when（pre_check 那拍）起擋 sinkA
    void blockForSwitchFrom(Tick when) { sinkABlockFrom = when; }
    // 在 switch_idle 那拍呼叫。RTL: switchIdle 是組合訊號，那拍 sinkA
    // 就收得下一筆，所以第一筆在這拍放（clockEdge()），之後每拍一筆
    void unblockForSwitch()
    {
        if (switchBlocked) {
            switchBlocked = false;
            if (!heldSinkA.empty() && !sinkAReplayEvent.scheduled())
                schedule(sinkAReplayEvent, clockEdge());
        }
    }

    void setSwitch(SwitchController *s) { switchCtrl = s; }

    // RTL: Scheduler 的 isScheNoOtherWorks（組合邏輯）
    bool isScheNoOtherWorks() const
    {
        return mshrQueue.isEmpty() && writeBuffer.isEmpty() &&
               flushTxn.pkt == nullptr;
    }
    // MSHR 清空會呼叫 SwitchController::notifyMshrDrained()；writeBuffer
    // 和 flush 沒有 hook（WriteQueue::markInService 不是 virtual）
    bool mshrEmpty() const { return mshrQueue.isEmpty(); }

    // RTL: runtime_PIC_Mode_levels 暫存器直接接到 Directory（不是訊息）
    void setPicWays(uint32_t beginWay, uint32_t endWay, bool pic)
    {
        picTags()->setDirPicWays(beginWay, endWay, pic);
    }

    // RTL: switch_resp 時 for_assert := runtime，Bank 才看到新的 PIC 狀態
    void commitPicWays() { picTags()->commitBankPicState(); }
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
