#include "mat_mac.hh"

#include <bitset>
#include <cstdint>

namespace gem5
{
namespace mat_fsm
{

int
MatMac::sumOfMac(const MatDatapath &datapath, const MacControl &ctrl)
{
    const auto &rVec = datapath.rVec();
    const uint64_t l = datapath.vecBuf();

    int64_t sum = 0;
    for (unsigned i = 0; i < NumArrays; ++i) {
        // Rule 6/8: non-MAC-mode lanes are zeroed at the MAC OUTPUT, not
        // by assuming their SRAM data is zero.
        if (!ctrl.macEnable[i]) {
            continue;
        }

        const uint64_t andResult = rVec[i] & l;
        const int64_t pop = static_cast<int64_t>(std::bitset<64>(andResult).count());

        // Rule 1: shift = bitIdR[i] + lBitSliceId, bounded to the 4-bit
        // bias field.
        const unsigned shift = (static_cast<unsigned>(ctrl.bitIdR[i]) + ctrl.lBitSliceId) & MacShiftBiasMask;

        // Rule 2: R holds its sign bit on this lane. (The formula's own
        // "arrayMode==Mac" clause is already guaranteed by the
        // macEnable[i] check above.)
        const bool rSign = ctrl.signedRLastExist && (ctrl.bitIdR[i] == ctrl.lastBitRBitId);
        // Rule 3: L has reached its own last bit index (lPrecisionReg_ IS
        // the final bit index, not a count -- so this is ==, not ==-1).
        const bool lSign = ctrl.signedL && (ctrl.lBitSliceId == ctrl.lPrecisionReg);
        // Rule 4: negate iff exactly one side is signed.
        const bool negate = rSign != lSign;

        const int64_t shifted = pop << shift;
        sum += negate ? -shifted : shifted;
    }

    // Adder #1: the four already-shifted-and-signed per-lane outputs.
    return static_cast<int>(sum);
}

} // namespace mat_fsm
} // namespace gem5
