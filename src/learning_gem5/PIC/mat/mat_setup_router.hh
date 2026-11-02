#ifndef __LEARNING_GEM5_PIC_MAT_MAT_SETUP_ROUTER_HH__
#define __LEARNING_GEM5_PIC_MAT_MAT_SETUP_ROUTER_HH__

#include <string>
#include <unordered_map>

#include "learning_gem5/PIC/mat/mat.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/MatSetUpRouter.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * Delivers the Scheduler's EXE (one CalPayload on cb_port) to the one Mat
 * it targets. RTL: BanksShell.scala L84-99 picks the bank from the low
 * bits of set_up_addr, Bank.scala L63-76 the Mat from the high bits; every
 * other field is wired to all Mats, and exec only to the target.
 *
 * The response goes back one cycle later, without waiting for the Mat to
 * finish -- RTL CMD_resolver_new fire_EXE (L571) is back in idle_EXE the
 * next cycle; the Mat's finish is tracked separately (EXE Finish tracker,
 * L720-797, not modelled here yet).
 */
class MatSetUpRouter : public ClockedObject
{
  private:
    class CmdPort : public ResponsePort
    {
      private:
        MatSetUpRouter *owner;

      public:
        CmdPort(const std::string &name, MatSetUpRouter *owner)
            : ResponsePort(name), owner(owner) {}

        AddrRangeList getAddrRanges() const override { return {}; }

      protected:
        Tick recvAtomic(PacketPtr pkt) override
        { panic("MatSetUpRouter: recvAtomic unimpl."); }
        void recvFunctional(PacketPtr pkt) override
        { panic("MatSetUpRouter: recvFunctional unimpl."); }
        bool recvTimingReq(PacketPtr pkt) override
        { return owner->handleCmd(pkt); }
        void recvRespRetry() override { owner->handleRespRetry(); }
    };

    CmdPort cmdPort;
    const unsigned numBanks;
    /** matID (way * numBanks + bank) -> Mat. */
    std::unordered_map<unsigned, Mat *> matById;

    EventFunctionWrapper respEvent;
    PacketPtr pendingResp = nullptr;
    bool respBlocked = false;

    bool handleCmd(PacketPtr pkt);
    void sendResp();
    void handleRespRetry();

  public:
    MatSetUpRouter(const MatSetUpRouterParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_MAT_SETUP_ROUTER_HH__
