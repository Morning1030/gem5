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

class Accumulator : public ClockedObject
{
  private:
    enum class State
    {
        IDLE,
        READ,
        WRITE_BACK
    };

    class CPUSidePort : public ResponsePort
    {
      private:
        Accumulator *owner;

      public:
        CPUSidePort(const std::string &name, Accumulator *owner);
        PacketPtr blockedPacket;
        void sendPacket(PacketPtr pkt);

        bool responseBlocked() const { return blockedPacket != nullptr; }

      protected:
        Tick recvAtomic(PacketPtr pkt) override {panic("%s atomic access unsupported", name());}
        void recvFunctional(PacketPtr pkt) override {panic("%s functional access unsupported", name());}
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override {return {};}
    };

    class MemSidePort : public RequestPort
    {
      private:
        Accumulator *owner;

      public:
        MemSidePort(const std::string &name, Accumulator *owner);
        PacketPtr blockedPacket;
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override {}
    };

    CPUSidePort instPort;
    MemSidePort cacheBankPort;
    RequestorID requestorId;

    const uint64_t wordlineNums;
    const uint64_t arraysPerMat;

    State state = State::IDLE;

    PacketPtr pendingReqPkt = nullptr;

    uint64_t baseSrcPicAddr = 0;
    uint64_t destPicAddr = 0;
    uint32_t totalRows = 0;
    uint32_t rowPtr = 0;
    uint8_t sourceCount = 0;
    bool acc32Bit = true;

    uint8_t readSourcePtr = 0;
    uint32_t readResponses = 0;
    uint64_t accumulatorBuf = 0;

    EventFunctionWrapper readEvent;
    EventFunctionWrapper writeEvent;

    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);

    void processReadEvent();
    void processWriteEvent();

    uint64_t sourceAddress(uint32_t readSourcePtr) const {
        return baseSrcPicAddr + readSourcePtr * wordlineNums * arraysPerMat + rowPtr;
    }
    uint64_t add32(uint64_t acc, uint64_t data) const;
    uint64_t add16(uint64_t acc, uint64_t data) const;

  public:
    Accumulator(const AccumulatorParams &params);
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

// uint64_t
// Accumulator::sourceAddress(uint32_t readSourcePtr) const
// {
//     const uint64_t sourceStride = wordlineNums * arraysPerMat;  // 512*4=2048
//     return baseSrcPicAddr + readSourcePtr * sourceStride + rowPtr;
// }

} // namespace gem5

#endif // __LEARNING_GEM5_PIC_ACCUMULATOR_HH__
