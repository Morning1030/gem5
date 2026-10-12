#ifndef __LEARNING_GEM5_PIC_ACC_MOCK_BANK_HH__
#define __LEARNING_GEM5_PIC_ACC_MOCK_BANK_HH__

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>

#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/AccMockBank.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class AccMockBank : public ClockedObject
{
  private:
    class BankSidePort : public ResponsePort
    {
      private:
        AccMockBank *owner;

      public:
        BankSidePort(const std::string &name, AccMockBank *owner);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;
    };

    BankSidePort port;

    const Addr baseSrcAddr;
    const Addr destAddr;
    const uint32_t sourceCount;
    const uint32_t rowCount;
    const bool acc32Bit;
    const uint64_t wordlineNums;
    const uint64_t arraysPerMat;
    const Cycles responseLatency;

    std::unordered_map<Addr, uint64_t> storage;

    std::deque<PacketPtr> pendingReadResponses;
    PacketPtr blockedResponse = nullptr;
    EventFunctionWrapper responseEvent;

    uint32_t writesSeen = 0;

    bool handleRequest(PacketPtr pkt);
    void processResponse();

    uint64_t sourceWord(uint32_t source, uint32_t row) const;
    uint64_t expectedRow(uint32_t row) const;
    uint64_t add32(uint64_t acc, uint64_t data) const;
    uint64_t add16(uint64_t acc, uint64_t data) const;

  public:
    AccMockBank(const AccMockBankParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void startup() override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_ACC_MOCK_BANK_HH__
