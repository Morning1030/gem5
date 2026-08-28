#ifndef __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
#define __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__

#include <cstdint>
#include <string>
#include <vector>
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
    bool isWayPICMode(const uint32_t wayID) const;
    void setWayPICMode(const uint32_t wayID, bool picMode);

  private:
    std::vector<bool> PIC_mode; // each element indicates one way
};

class CacheController : public BaseCache
{
  private:
    class CPUSidePort : public BaseCache::CpuSidePort
    {
      private:
        CacheController* owner;

      public:
        CPUSidePort(const std::string& name, CacheController *owner);

      protected:
        bool recvTimingReq(PacketPtr pkt) override;
    };

    class MemSidePort : public BaseCache::MemSidePort
    {
      private:
        CacheController *owner;

      public:
        MemSidePort(const std::string& name, CacheController *owner);
    };

    CPUSidePort cpuSidePort;
    MemSidePort memSidePort;
    PICTags *tags;

  public:
    CacheController(CacheControllerParams *params);

    bool handleQueryWayState(PacketPtr pkt);
    bool handleFlushReq(PacketPtr pkt);
    bool handleCache2PIC(PacketPtr pkt);
    bool handlePIC2Cache(PacketPtr pkt);
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
