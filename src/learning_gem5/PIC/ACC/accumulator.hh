#ifndef __LEARNING_GEM5_PIC_ACCUMULATOR_HH__
#define __LEARNING_GEM5_PIC_ACCUMULATOR_HH__

#include <cstdint>
#include <string>

#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "params/Accumulator.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

struct AccRequestPayload
{
    uint64_t baseSrcPicAddr;
    uint64_t destPicAddr;
    uint32_t rowNum;
    uint8_t sourceCount;
    uint8_t acc32Bit;
    uint8_t reserved[2];
};

static_assert(sizeof(AccRequestPayload) == 24,
              "AccRequestPayload layout changed unexpectedly");

class Accumulator : public ClockedObject
{
  private:
    enum class State
    {
        IDLE,
        READ,
        WRITE_BACK
    };

    enum class BankReqKind
    {
        NONE,
        READ,
        WRITE
    };

    class ControlPort : public ResponsePort
    {
      private:
        Accumulator *owner;
        PacketPtr blockedResponse = nullptr;

      public:
        ControlPort(const std::string &name, Accumulator *owner);

        bool responseBlocked() const { return blockedResponse != nullptr; }
        void sendResponse(PacketPtr pkt);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;
    };

    class BankPort : public RequestPort
    {
      private:
        Accumulator *owner;

      public:
        BankPort(const std::string &name, Accumulator *owner);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    ControlPort instPort;
    BankPort bankPort;
    RequestorID requestorId;

    const uint64_t wordlineNums;
    const uint64_t arraysPerMat;

    State state = State::IDLE;

    PacketPtr pendingControlPkt = nullptr;

    uint64_t baseSrcPicAddr = 0;
    uint64_t destPicAddr = 0;
    uint32_t totalRows = 0;
    uint32_t rowPtr = 0;
    uint8_t sourceCount = 0;
    bool acc32Bit = true;

    uint32_t nextReadSource = 0;
    uint32_t readResponses = 0;
    uint64_t accumulatorBuf = 0;

    PacketPtr blockedBankPkt = nullptr;
    BankReqKind blockedBankKind = BankReqKind::NONE;

    EventFunctionWrapper readIssueEvent;
    EventFunctionWrapper writeIssueEvent;

    bool handleControlRequest(PacketPtr pkt);
    bool handleBankResponse(PacketPtr pkt);
    void retryBlockedBankRequest();

    void processReadIssue();
    void processWriteIssue();

    void sendBankPacket(PacketPtr pkt, BankReqKind kind);
    void bankRequestAccepted(BankReqKind kind);

    void beginRow();
    void maybeFinishReads();
    void finishOperation();

    uint64_t sourceAddress(uint32_t sourceIndex) const;
    uint64_t add32(uint64_t acc, uint64_t data) const;
    uint64_t add16(uint64_t acc, uint64_t data) const;

  public:
    Accumulator(const AccumulatorParams &params);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_ACCUMULATOR_HH__
