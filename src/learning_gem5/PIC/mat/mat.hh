#ifndef __LEARNING_GEM5_PIC_MAT_MAT_HH__
#define __LEARNING_GEM5_PIC_MAT_MAT_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <utility>

#include "learning_gem5/PIC/mat/mat_fsm.hh"
#include "mem/cache/tags/pic_llc_tags.hh"
#include "mem/port.hh"
#include "params/Mat.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

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
 * MatSetUpRouter delivers Scheduler CAL commands through startCal().
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

    /** Receives 8-byte array reads/writes from AccessBankArb. */
    class ArrayPort : public ResponsePort
    {
      private:
        Mat *owner;

      public:
        ArrayPort(const std::string &name, Mat *owner)
            : ResponsePort(name), owner(owner) {}

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("Mat: recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override
        { panic("Mat: recvFunctional unimpl."); }
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handleArrayReq(pkt); }
        void recvRespRetry() override { owner->handleArrayRespRetry(); }
    };

    RequestorID requestorId;
    EventFunctionWrapper tickEvent;

    LoadPort loadPort;
    ArrayPort arrayPort;
    EventFunctionWrapper arrayRespEvent;
    const Cycles arrayLatency;
    /** (ready tick, response) in arrival order. */
    std::deque<std::pair<Tick, PacketPtr>> arrayRespQ;
    bool arrayRespBlocked = false;

    mat_fsm::MatFSM fsm;
    mat_fsm::SetUpIO setUp;
    PacketPtr blockedReq = nullptr;  // request_vec.valid, not granted
    bool reqFired = false;   // granted after a retry
    bool respArrived = false;
    unsigned loads = 0;
    unsigned droppedArrayAccesses = 0;

    unsigned cycles = 0;
    const unsigned maxCycles;
    const bool pic;
    const unsigned bankIndex;
    const unsigned way;
    PICLLCTags *tags;  // may be nullptr -- busy reporting is then skipped

    bool requestVec();
    bool responseVec();
    bool handleLoadResp(PacketPtr pkt);
    void handleLoadRetry();
    bool handleArrayReq(PacketPtr pkt);
    void handleArrayRespRetry();
    void processArrayResp();
    void processTick();

  public:
    Mat(const MatParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    unsigned bank() const { return bankIndex; }
    unsigned wayIndex() const { return way; }

    /** One Scheduler EXE's set-up, delivered by MatSetUpRouter. */
    void startCal(const mat_fsm::SetUpIO &cmd);
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_MAT_HH__
