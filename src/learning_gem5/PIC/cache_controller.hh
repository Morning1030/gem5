#ifndef __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
#define __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__

#include <cstdint>
#include <string>
#include <vector>
#include "mem/cache/base.hh"
#include "mem/cache/tags/pic_llc_tags.hh"
#include "mem/packet.hh"
#include "params/CacheController.hh"

namespace gem5
{

// mod: consolidate on PICLLCTags -- this used to own a separate, thinner
// PICTags class (findVictim/getSetWayValid/getSetWayAddr/isWayPICMode/
// setWayPICMode). PICLLCTags is the more complete implementation (flush
// protocol, per-Mat busy gate) and is now the only tag store in the PIC
// tree; see mem/cache/tags/pic_llc_tags.hh.
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
    PICLLCTags *tags;

  public:
    CacheController(CacheControllerParams *params);

    bool handleQueryWayState(PacketPtr pkt);
    bool handleFlushReq(PacketPtr pkt);
    bool handleCache2PIC(PacketPtr pkt);
    bool handlePIC2Cache(PacketPtr pkt);
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_CACHE_CONTROLLER_HH__
