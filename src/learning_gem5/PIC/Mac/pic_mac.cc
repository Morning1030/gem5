#include "learning_gem5/PIC/Mac/pic_mac.hh"

namespace gem5
{
namespace pic
{

uint32_t
macPrimitive(uint64_t leftVec, uint64_t rightVec,
             uint8_t bias, bool negative)
{
    const uint8_t rtlBias = bias & 0x0f;
    const uint64_t andResult = leftVec & rightVec;
    const uint32_t popCount =
        static_cast<uint32_t>(__builtin_popcountll(andResult));
    const uint32_t shifted =
        static_cast<uint32_t>(popCount << rtlBias);

    return negative ? static_cast<uint32_t>(0u - shifted) : shifted;
}

uint32_t
matMacSum(uint64_t leftVec,
          const std::array<MacArrayInput, 4> &arrays)
{
    uint32_t sum = 0;
    for (const auto &array : arrays) {
        if (!array.enable)
            continue;
        sum += macPrimitive(leftVec, array.rightVec,
                            array.bias, array.negative);
    }
    return sum;
}

} // namespace pic
} // namespace gem5
