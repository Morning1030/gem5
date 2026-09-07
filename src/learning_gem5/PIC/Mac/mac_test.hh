#ifndef __LEARNING_GEM5_PIC_MAC_TEST_HH__
#define __LEARNING_GEM5_PIC_MAC_TEST_HH__

#include <cstdint>
#include "params/MacTest.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
namespace pic
{

class MacTest : public ClockedObject
{
  private:
    const uint64_t randomCases;
    static uint32_t goldenPopcount(uint64_t value);
    static uint32_t goldenMac(uint64_t left, uint64_t right,
                              uint8_t bias, bool negative);
    void runPrimitiveDirectedTests();
    void runMatDirectedTests();
    void runRandomTests();

  public:
    MacTest(const MacTestParams &params);
    void startup() override;
};

} // namespace pic
} // namespace gem5

#endif
