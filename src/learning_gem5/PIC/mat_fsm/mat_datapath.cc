#include "mat_datapath.hh"

namespace gem5
{
namespace mat_fsm
{

MatDatapath::MatDatapath() = default;

void
MatDatapath::readCArray(uint64_t addr, const std::array<bool, NumArrays> &readEn)
{
    for (unsigned i = 0; i < NumArrays; ++i) {
        rVec_[i] = readEn[i] ? cArraySram_[i][addr % CArrayWordlineNums] : 0;
    }
}

void
MatDatapath::latchMArrayRow(uint64_t addr)
{
    const uint64_t row = mArraySram_[addr % MArrayWordlineNums];
    for (unsigned i = 0; i < WBufNumSlots; ++i) {
        rWire_[i] = static_cast<int16_t>((row >> (i * AccLaneWidth)) & 0xFFFF);
    }
    rBuf_ = rWire_;  // latch for reuse on cycles with no fresh read
}

void
MatDatapath::accumulate(unsigned idx, unsigned accWidth, bool isFirstSlice,
                         bool freshMArrayValid, int macResult)
{
    const std::array<int16_t, WBufNumSlots> &prevSource = freshMArrayValid ? rWire_ : rBuf_;

    if (accWidth == AccWidth32Bit) {
        // idx = high lane; ASSUMPTION: idx-1 (mod WBufNumSlots) = low lane.
        const unsigned idxHigh = idx;
        const unsigned idxLow = (idx + WBufNumSlots - 1) % WBufNumSlots;
        const uint32_t prevHigh = isFirstSlice ? 0 : static_cast<uint16_t>(prevSource[idxHigh]);
        const uint32_t prevLow = isFirstSlice ? 0 : static_cast<uint16_t>(prevSource[idxLow]);
        const uint32_t prev32 = (prevHigh << 16) | prevLow;
        const uint32_t result32 = prev32 + static_cast<uint32_t>(macResult);
        wBuf_[idxHigh] = static_cast<int16_t>(result32 >> 16);
        wBuf_[idxLow] = static_cast<int16_t>(result32 & 0xFFFF);
    } else {
        const int16_t prev16 = isFirstSlice ? 0 : prevSource[idx];
        wBuf_[idx] = static_cast<int16_t>(prev16 + macResult);
    }
}

void
MatDatapath::commitWBufToMArray(uint64_t addr)
{
    uint64_t word = 0;
    for (unsigned i = 0; i < WBufNumSlots; ++i) {
        word |= (static_cast<uint64_t>(static_cast<uint16_t>(wBuf_[i])) << (i * AccLaneWidth));
    }
    mArraySram_[addr % MArrayWordlineNums] = word;
}

} // namespace mat_fsm
} // namespace gem5
