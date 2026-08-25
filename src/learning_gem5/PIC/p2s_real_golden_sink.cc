#include "learning_gem5/PIC/p2s_real_golden_sink.hh"
#include "base/logging.hh"
#include "learning_gem5/PIC/p2s.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

P2SRealGoldenSink::SinkPort::SinkPort(
    const std::string &name,
    P2SRealGoldenSink *owner)
    : ResponsePort(name, owner),
      owner(owner)
{
}

Tick
P2SRealGoldenSink::SinkPort::recvAtomic(PacketPtr pkt)
{
    panic("%s atomic unsupported", name());
}

void
P2SRealGoldenSink::SinkPort::recvFunctional(PacketPtr pkt)
{
    panic("%s functional unsupported", name());
}

bool
P2SRealGoldenSink::SinkPort::recvTimingReq(PacketPtr pkt)
{
    return owner->check(pkt);
}

void
P2SRealGoldenSink::SinkPort::recvRespRetry()
{
}

AddrRangeList
P2SRealGoldenSink::SinkPort::getAddrRanges() const
{
    return {};
}

P2SRealGoldenSink::P2SRealGoldenSink(
    const Params &params)
    : ClockedObject(params),
      port(name() + ".port", this),
      basePicAddr(params.base_pic_addr),
      baseArray(params.base_array),
      rows(params.rows),
      cols(params.cols),
      precision(params.precision),
      bufNum(params.buf_num),
      wordlineNums(params.wordline_nums),
      exitOnPass(params.exit_on_pass)
{
    if (params.mode == "L") {
        mode = Mode::L;
    }
    else if (params.mode == "R") {
        mode = Mode::R;
    }
    else if (params.mode == "RT") {
        mode = Mode::RT;
    }
    else {
        panic(
            "%s invalid P2S golden mode '%s'; expected L, R, or RT",
            name(), params.mode.c_str());
    }

    panic_if(
        precision > 7,
        "%s precision=%u exceeds 7",
        name(),
        precision);

    if (mode == Mode::L) {
        panic_if(
            cols > 64,
            "%s L checker requires cols <= 64",
            name());
    }
    else {
        panic_if(
            bufNum != 2,
            "%s R/RT checker currently requires bufNum=2",
            name());

        if (mode == Mode::RT) {
            panic_if(
                cols > 64,
                "%s RT checker requires cols <= 64",
                name());
        }
        else {
            panic_if(
                rows > 64,
                "%s R checker requires rows <= 64",
                name());
        }
    }
}

Port &
P2SRealGoldenSink::getPort(
    const std::string &if_name,
    PortID idx)
{
    if (if_name == "port")
        return port;

    return ClockedObject::getPort(
        if_name,
        idx);
}

const char *
P2SRealGoldenSink::modeName() const
{
    switch (mode) {
      case Mode::L:
        return "P2S_L";
      case Mode::R:
        return "P2S_R";
      case Mode::RT:
        return "P2S_R_T";
    }

    return "P2S_UNKNOWN";
}

uint64_t
P2SRealGoldenSink::expectedWriteCount() const
{
    const uint64_t bits =
        static_cast<uint64_t>(precision) + 1;

    if (mode == Mode::L || mode == Mode::RT) {
        return static_cast<uint64_t>(rows) * bits;
    }

    return static_cast<uint64_t>(cols) * bits;
}

uint64_t
P2SRealGoldenSink::expectedBitSlice(
    uint32_t major,
    uint8_t bit) const
{
    uint64_t expected = 0;

    if (mode == Mode::L) {
        const uint32_t row = major;

        for (uint32_t elem = 0; elem < cols; ++elem) {
            const uint8_t value =
                static_cast<uint8_t>(
                    (row + elem) & 0xff);

            if ((value >> bit) & 0x1) {
                expected |=
                    (uint64_t{1} << elem);
            }
        }

        return expected;
    }

    if (mode == Mode::R) {
        const uint32_t col = major;

        for (uint32_t row = 0; row < rows; ++row) {
            const uint8_t value =
                static_cast<uint8_t>(
                    (row * 13 +
                     col * 29 +
                     7) & 0xff);

            if ((value >> bit) & 0x1) {
                expected |=
                    (uint64_t{1} << row);
            }
        }

        return expected;
    }

    const uint32_t row = major;

    for (uint32_t col = 0; col < cols; ++col) {
        const uint8_t value = static_cast<uint8_t>((row + col) & 0xff);

        if ((value >> bit) & 0x1) {
            expected |= (uint64_t{1} << col);
        }
    }

    return expected;
}

uint64_t
P2SRealGoldenSink::expectedAddress(
    uint32_t major,
    uint8_t bit) const
{
    if (mode == Mode::L) {
        return basePicAddr +
            static_cast<Addr>(major) +
            static_cast<Addr>(bit) *
                static_cast<Addr>(rows);
    }

    static constexpr uint32_t offsetForBit[8] = {0, 1, 4, 5, 8, 9, 12, 13};

    return(baseArray + offsetForBit[bit]) * wordlineNums + major;
}

bool
P2SRealGoldenSink::check(PacketPtr pkt)
{
    panic_if(
        !pkt->isWrite(),
        "%s expected P2S WriteReq",
        name());

    panic_if(
        pkt->getSize() != sizeof(P2SWritePayload),
        "%s expected P2SWritePayload size %u, got %u",
        name(),
        static_cast<unsigned>(
            sizeof(P2SWritePayload)),
        pkt->getSize());

    static_assert(
        sizeof(P2SWritePayload) == 16,
        "P2SWritePayload checker expects 16-byte payload");

    const auto *payload =
        pkt->getConstPtr<P2SWritePayload>();

    const uint32_t bitsPerMajor =
        static_cast<uint32_t>(precision) + 1;

    const uint64_t expectedWrites =
        expectedWriteCount();

    panic_if(
        writeIndex >= expectedWrites,
        "%s received extra P2S write index=%llu",
        name(),
        static_cast<unsigned long long>(
            writeIndex));

    const uint32_t major =
        static_cast<uint32_t>(
            writeIndex / bitsPerMajor);

    const uint8_t bit =
        static_cast<uint8_t>(
            writeIndex % bitsPerMajor);

    const uint64_t expectedAddr =
        expectedAddress(major, bit);

    const uint64_t expectedData =
        expectedBitSlice(major, bit);

    panic_if(
        payload->arrayAddr != expectedAddr,
        "%s %s address mismatch "
        "index=%llu major=%u bit=%u "
        "got=%#llx expected=%#llx",
        name(),
        modeName(),
        static_cast<unsigned long long>(
            writeIndex),
        major,
        static_cast<unsigned>(bit),
        static_cast<unsigned long long>(
            payload->arrayAddr),
        static_cast<unsigned long long>(
            expectedAddr));

    panic_if(
        payload->bitSlice != expectedData,
        "%s %s data mismatch "
        "index=%llu major=%u bit=%u "
        "got=%#llx expected=%#llx",
        name(),
        modeName(),
        static_cast<unsigned long long>(
            writeIndex),
        major,
        static_cast<unsigned>(bit),
        static_cast<unsigned long long>(
            payload->bitSlice),
        static_cast<unsigned long long>(
            expectedData));

    if (mode != Mode::L || major == 0 || major == rows - 1) {
        inform(
            "%s GOLDEN PASS: mode=%s major=%u bit=%u "
            "arrayAddr=%#llx data=%#llx",
            name(),
            modeName(),
            major,
            static_cast<unsigned>(bit),
            static_cast<unsigned long long>(
                payload->arrayAddr),
            static_cast<unsigned long long>(
                payload->bitSlice));
    }

    ++writeIndex;
    delete pkt;

    if (writeIndex == expectedWrites) {
        inform(
            "%s REAL %s FULL PASS: %llu writes",
            name(),
            modeName(),
            static_cast<unsigned long long>(
                expectedWrites));

        if (exitOnPass) {
            exitSimLoop(
                name() +
                std::string(" Scheduler -> real ") +
                modeName() +
                " -> DMA PASS");
        }
    }

    return true;
}

} // namespace gem5
