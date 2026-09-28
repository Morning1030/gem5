#ifndef __LEARNING_GEM5_PIC_MAT_MAT_BANK_HH__
#define __LEARNING_GEM5_PIC_MAT_MAT_BANK_HH__

#include <memory>
#include <string>
#include <vector>

#include "learning_gem5/PIC/mat/mat_fsm.hh"
#include "mem/cache/tags/pic_llc_tags.hh"
#include "mem/port.hh"
#include "params/MatBank.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

/**
 * One cache bank's worth of Mats. Owns one plain mat_fsm::MatFSM per
 * PIC-capable way (ways below first_pic_way are the reserved normal-cache
 * level: no FSM, no port) and steps all of them from a single per-bank
 * tick event, so they advance in the same cycle.
 *
 * The bank does no arbitration for L loads: each PIC Mat has its own
 * load_l_port straight to the cache-wide AutoLoadL, whose round-robin
 * arbiter picks among all Mats. A Mat holds its own request (blockedReq)
 * until AutoLoadL grants it, exactly like request_vec.valid in
 * send_L_req.
 */
class MatBank : public ClockedObject
{
  private:
    struct MatCtx;

    class LoadPort : public RequestPort
    {
      private:
        MatBank *owner;
        unsigned mat;  // index into MatBank::mats

      protected:
        bool recvTimingResp(PacketPtr pkt) override
        { return owner->handleLoadResp(mat, pkt); }
        void recvReqRetry() override { owner->handleLoadRetry(mat); }

      public:
        LoadPort(const std::string &name, MatBank *owner, unsigned mat)
            : RequestPort(name, owner, mat), owner(owner), mat(mat) {}
    };

    struct MatCtx
    {
        unsigned way = 0;
        mat_fsm::MatFSM fsm;
        mat_fsm::SetUpIO setUp;
        PacketPtr blockedReq = nullptr;  // request_vec.valid, not granted
        bool reqFired = false;   // granted after a retry
        bool respArrived = false;
        bool done = false;
        unsigned loads = 0;
    };

    RequestorID requestorId;
    EventFunctionWrapper tickEvent;

    std::vector<MatCtx> mats;  // PIC ways only
    std::vector<std::unique_ptr<LoadPort>> loadPorts;
    unsigned cycles = 0;
    const unsigned maxCycles;
    const Cycles startCycles;
    const unsigned bankIndex;
    PICLLCTags *tags;  // may be nullptr -- busy reporting is then skipped

    bool requestVec(unsigned i);
    bool responseVec(unsigned i);
    bool handleLoadResp(unsigned i, PacketPtr pkt);
    void handleLoadRetry(unsigned i);
    void processTick();

  public:
    MatBank(const MatBankParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void startup() override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_MAT_BANK_HH__
