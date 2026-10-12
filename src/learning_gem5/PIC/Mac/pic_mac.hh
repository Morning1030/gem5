#ifndef __LEARNING_GEM5_PIC_PIC_MAC_HH__
#define __LEARNING_GEM5_PIC_PIC_MAC_HH__

#include <array>
#include <cstdint>

namespace gem5
{
namespace pic
{

uint32_t macPrimitive(uint64_t leftVec, uint64_t rightVec,
                      uint8_t bias, bool negative);

struct MacArrayInput
{
    uint64_t rightVec = 0;
    uint8_t bias = 0;
    bool negative = false;
    bool enable = false;
};

uint32_t matMacSum(uint64_t leftVec,
                   const std::array<MacArrayInput, 4> &arrays);

} // namespace pic
} // namespace gem5

#endif
