#ifndef __LEARNING_GEM5_PIC_MAT_MAT_HH__
#define __LEARNING_GEM5_PIC_MAT_MAT_HH__

#include <cstdint>
#include <string>

#include "learning_gem5/PIC/mat/mat_fsm.hh"
#include "mem/cache/tags/pic_llc_tags.hh"
#include "mem/port.hh"
#include "params/Mat.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/** {addr, optype, data} -- same shape as AutoLoadLPayload's read, plus a
 *  write side. RTL: the PolyArray SRAM port is a plain 64-bit
 *  addr/write_enable/dataIn/dataOut port (PolyArray.scala:27-29) --
 *  bit-slicing is a P2S-side concept, never seen once data reaches this
 *  port, so this carries no bit-slice metadata either. */
enum class MatCacheOp : uint8_t { READ, WRITE };
struct MatCacheReqPayload
{
    uint64_t addr;
    MatCacheOp optype;
    uint64_t data;  // valid on WRITE; the read result on a READ response
};

/**
 * One PIC Mat, one SimObject -- replaces the earlier MatBank (which
 * batched a whole bank's Mats behind one shared tick event). Each Mat
 * runs on its own schedule: it only self-schedules a step while its own
 * FSM has work to do, matching "each Mat computes at a different time"
 * rather than lockstepping a bank's 15 Mats together.
 *
 * Owns one plain mat_fsm::MatFSM (not a SimObject itself -- see
 * mat_fsm.hh). load_l_port reaches the cache-wide AutoLoadL directly
 * (its round-robin arbiter already treats every Mat as an independent
 * client, one fetch in flight across the whole cache at a time -- see
 * AutoLoadL.hh).
 *
 * cache_port carries MatCacheReqPayload to CacheController, but nothing
 * in mat_fsm/mat_datapath drives it yet: MatFSM's cal()/tickBackground()
 * are still built synchronous/timing-free (the original design mandate),
 * and reading/writing through a real port means tolerating a real
 * round-trip response latency mid-command -- that needs a real design
 * pass on its own, not a port declaration. handleCacheResp() exists so
 * the response path is wired end to end once that lands.
 */
class Mat : public ClockedObject
{
  private:
    class LoadPort : public RequestPort
    {
      private:
        Mat *owner;

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleLoadResp(pkt); }
        void recvReqRetry() override { owner->handleLoadRetry(); }

      public:
        LoadPort(const std::string &name, Mat *owner)
            : RequestPort(name, owner), owner(owner) {}
    };

    class CachePort : public RequestPort
    {
      private:
        Mat *owner;

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleCacheResp(pkt); }
        void recvReqRetry() override { owner->handleCacheRetry(); }

      public:
        CachePort(const std::string &name, Mat *owner)
            : RequestPort(name, owner), owner(owner) {}
    };

    RequestorID requestorId;
    EventFunctionWrapper tickEvent;

    LoadPort loadPort;
    CachePort cachePort;

    mat_fsm::MatFSM fsm;
    mat_fsm::SetUpIO setUp;
    PacketPtr blockedReq = nullptr;  // request_vec.valid, not granted
    bool reqFired = false;   // granted after a retry
    bool respArrived = false;
    bool done = false;
    unsigned loads = 0;

    unsigned cycles = 0;
    const unsigned maxCycles;
    const Cycles startCycles;
    const unsigned bankIndex;
    const unsigned way;
    PICLLCTags *tags;  // may be nullptr -- busy reporting is then skipped

    bool requestVec();
    bool responseVec();
    bool handleLoadResp(PacketPtr pkt);
    void handleLoadRetry();
    // Not yet called by anything -- see the class doc comment.
    bool handleCacheResp(PacketPtr pkt);
    void handleCacheRetry();
    void processTick();

  public:
    Mat(const MatParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void startup() override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_MAT_HH__
