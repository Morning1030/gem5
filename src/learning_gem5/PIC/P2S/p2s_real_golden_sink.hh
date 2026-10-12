#ifndef __LEARNING_GEM5_PIC_P2S_REAL_GOLDEN_SINK_HH__
#define __LEARNING_GEM5_PIC_P2S_REAL_GOLDEN_SINK_HH__

#include <cstdint>
#include <string>

#include "mem/port.hh"
#include "params/P2SRealGoldenSink.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class P2SRealGoldenSink : public ClockedObject
{
  private:
    enum class Mode{L,R,RT};

    class SinkPort : public ResponsePort
    {
      private:
        P2SRealGoldenSink *owner;

      public:
        SinkPort(
            const std::string &name,
            P2SRealGoldenSink *owner);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;
    };

    SinkPort port;

    Mode mode;
    Addr basePicAddr;
    uint64_t baseArray;
    uint32_t rows;
    uint32_t cols;
    uint8_t precision;
    uint8_t bufNum;
    uint32_t wordlineNums;
    bool exitOnPass;

    uint64_t writeIndex = 0;

    bool check(PacketPtr pkt);

    uint64_t expectedBitSlice(
        uint32_t major,
        uint8_t bit) const;

    uint64_t expectedAddress(
        uint32_t major,
        uint8_t bit) const;

    uint64_t expectedWriteCount() const;

    const char *modeName() const;

  public:
    using Params = P2SRealGoldenSinkParams;

    P2SRealGoldenSink(
        const Params &params);

    Port &getPort(
        const std::string &if_name,
        PortID idx = InvalidPortID) override;
};

} // namespace gem5

#endif
