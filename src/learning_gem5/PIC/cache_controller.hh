#ifndef __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
#define __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__

#include <cstdint>
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

    PicCtrlPort picCtrlPort;
    PicFlushPort picFlushPort;
    PICTags *picTags() const { return static_cast<PICTags*>(tags); }

  public:
    CacheController(const CacheControllerParams &params);
    Port &getPort(const std::string &if_name, PortID idx = InvalidPortID) override;
    bool handlePicCtrlReq(PacketPtr pkt);
    bool handleQueryWayState(PacketPtr pkt);
    bool handleDrainQuery(PacketPtr pkt);
    bool handleFlushReq(PacketPtr pkt);
    bool handleWayMode(PacketPtr pkt);
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
