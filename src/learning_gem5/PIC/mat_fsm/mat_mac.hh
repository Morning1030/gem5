#ifndef __LEARNING_GEM5_PIC_MAT_FSM_MAT_MAC_HH__
#define __LEARNING_GEM5_PIC_MAT_FSM_MAT_MAC_HH__

#include <array>

#include "mat_datapath.hh"

namespace gem5
{
namespace mat_fsm
{

// Shift-bias field width (section 6): the RTL bounds the shift to a 4-bit
// field (max useful value bitIdR+lBitSliceId = 7+7 = 14, well within
// range) -- wraps like real hardware if ever exceeded.
constexpr unsigned MacShiftBiasBits = 4;
constexpr unsigned MacShiftBiasMask = (1u << MacShiftBiasBits) - 1;

/// Control-plane inputs MatMac needs per cycle, gathered by MatFSM from
/// its own registers (bitIdR_, lBitSliceIdPtr_, etc.). A small explicit
/// bundle rather than handing MatMac the whole MatFSM.
struct MacControl
{
    unsigned lBitSliceId = 0;
    std::array<int, NumArrays> bitIdR{};
    int lastBitRBitId = 0;
    bool signedL = false;
    bool signedRLastExist = false;
    unsigned lPrecisionReg = 0;
    // arrayMode[i] == Mac, per lane. Gates that lane's MAC OUTPUT to 0 --
    // does NOT assume its SRAM data is already zero (section 8).
    std::array<bool, NumArrays> macEnable{};
};

/**
 * The MAC compute stage (cal() Datapath Spec sections 5-8/13): per-lane
 * AND -> PopCount -> Shift -> Sign, then Adder #1's 4-way reduction into
 * sum_of_mac. Stateless -- every call is a pure function of `datapath`'s
 * current rVec()/vecBuf() and the `ctrl` bundle passed in.
 */
class MatMac
{
  public:
    static int sumOfMac(const MatDatapath &datapath, const MacControl &ctrl);
};

} // namespace mat_fsm
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_FSM_MAT_MAC_HH__
