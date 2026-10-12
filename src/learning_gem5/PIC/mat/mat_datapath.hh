#ifndef __LEARNING_GEM5_PIC_MAT_FSM_MAT_DATAPATH_HH__
#define __LEARNING_GEM5_PIC_MAT_FSM_MAT_DATAPATH_HH__

#include <array>
#include <cstdint>

namespace gem5
{
namespace mat_fsm
{

// ---------------------------------------------------------------------
// Datapath storage constants (cal() Datapath Spec, sections 2/16). Kept
// independent even though some values coincide numerically (section 1).
// ---------------------------------------------------------------------

// GIVEN: spec lists "bitID_R[4]" and "arrayMode_reg[4]" explicitly.
constexpr unsigned NumArrays = 4;

// CONFIRMED (Controller.scala:122,132,152): wBuf's slot count is a fixed
// hardware-width constant, unrelated to nCal.
constexpr unsigned WBufNumSlots = 4;

// CONFIRMED (CalInfo.scala: ACC_32BIT=true.B/ACC_16BIT=false.B).
constexpr unsigned AccWidth32Bit = 32;
constexpr unsigned AccWidth16Bit = 16;

constexpr unsigned CArrayWordlineNums = 512;  // C-array SRAM depth per PolyArray.
constexpr unsigned VectorWidth = 64;          // R vector / vecBuf width.
constexpr unsigned AccLaneWidth = 16;         // rBuf/wBuf per-lane width.
static_assert(VectorWidth / AccLaneWidth == WBufNumSlots,
              "WBufNumSlots must equal VectorWidth / AccLaneWidth");
// RTL (Mat.scala/Controller.scala): there is no separate M-array SRAM. The
// M-array lives in the same four sub-arrays as the C-array: the arrays in
// Mem mode hold partial sums, the ones in Mac mode hold C data. An M-array
// address is {arrayID(2), row(9)}, so its address space covers all arrays.
constexpr unsigned MArrayWordlineNums = NumArrays * CArrayWordlineNums;

/**
 * The "cal() Datapath Specification" storage + Adder #2 (sections
 * 3/4/8-11/14-20): C-array SRAM, shared vec_buf, M-array SRAM, rWire/rBuf
 * and wBuf. This is the physical storage half of the Mat -- MatFSM
 * (mat_fsm.hh) is the control half: it owns every address register,
 * enable, and timing decision, and simply calls into this class when it
 * wants an effect applied. MatDatapath itself has no notion of
 * cycles/timing -- every method here is an immediate (combinational)
 * operation; any RegNext-style one-cycle delay is MatFSM's job (see
 * MatFSM::tickBackground()).
 *
 * The MAC (AND -> PopCount -> Shift -> Sign -> Adder #1, sections 5-8/13)
 * lives separately in MatMac (mat_mac.hh) -- it reads rVec()/vecBuf() from
 * here but is not part of this class.
 */
class MatDatapath
{
  public:
    MatDatapath();

    // ---- C-array read (section 3) --------------------------------
    // Latches rVec_ from cArraySram_[i][addr] for lanes where readEn[i]
    // is true, 0 otherwise (no read enable, no valid output -- mirrors
    // real SRAM).
    void readCArray(uint64_t addr, const std::array<bool, NumArrays> &readEn);
    const std::array<uint64_t, NumArrays> &rVec() const { return rVec_; }

    // ---- Shared vec_buf load interface (section 10) -----------------
    // Written externally by the Mat-level vector-load path (AutoLoadL),
    // NOT by MatFSM -- MatFSM only requests/acks the transfer
    // (requestVecFire_/responseVecFire_); a real caller wires AutoLoadL's
    // response directly into this.
    void loadVec(bool enable, uint64_t vecData) { if (enable) vecBuf_ = vecData; }
    uint64_t vecBuf() const { return vecBuf_; }

    // ---- M-array read: rWire vs rBuf (sections 8/9/14/15) ------------
    // Unpacks the addressed row into rWire_'s 4x16-bit lanes, and latches
    // it into rBuf_ for reuse on later cycles with no fresh read. MatFSM
    // calls this only on the cycle its own RegNext bookkeeping says the
    // read issued last cycle is now valid.
    void latchMArrayRow(uint64_t addr);
    const std::array<int16_t, WBufNumSlots> &rWire() const { return rWire_; }
    const std::array<int16_t, WBufNumSlots> &rBuf() const { return rBuf_; }

    // ---- Adder #2: partial-sum accumulation (sections 10/12/13/16-19) --
    // previous + macResult -> wBuf_, where previous is:
    //   isFirstSlice          -> 0
    //   freshMArrayValid      -> rWire_
    //   else                  -> rBuf_
    // 16-bit: one lane (idx). 32-bit: two adjacent lanes combined/split
    // (idx = high lane). MatFSM calls this only on the cycle its own
    // RegNext bookkeeping says an accumulate was requested last cycle,
    // passing macResult = MatMac::sumOfMac(*this, ctrl) it computed.
    void accumulate(unsigned idx, unsigned accWidth, bool isFirstSlice,
                     bool freshMArrayValid, int macResult);
    const std::array<int16_t, WBufNumSlots> &wBuf() const { return wBuf_; }

    // ---- M-array writeback (section 11/20/21) ------------------------
    // Packs wBuf_'s 4 lanes into one row and writes it at `addr`.
    void commitWBufToMArray(uint64_t addr);

    // ---- Array access from outside the Mat (AccessBankArb) ----------
    void writeCArrayWord(unsigned array, unsigned row, uint64_t word)
    { arraySram_[array % NumArrays][row % CArrayWordlineNums] = word; }

    // ---- test-only storage pokes / readback -------------------------
    uint64_t cArrayWord(unsigned array, unsigned row) const { return arraySram_[array][row % CArrayWordlineNums]; }
    void setCArrayWordForTest(unsigned array, unsigned row, uint64_t word) { arraySram_[array][row % CArrayWordlineNums] = word; }
    // addr = {arrayID(2), row(9)}
    uint64_t mArrayWord(unsigned addr) const { return arraySram_[mArrayID(addr)][mArrayRow(addr)]; }
    void setMArrayWordForTest(unsigned addr, uint64_t word) { arraySram_[mArrayID(addr)][mArrayRow(addr)] = word; }
    void setRWireForTest(const std::array<int16_t, WBufNumSlots> &v) { rWire_ = v; }
    void setRBufForTest(const std::array<int16_t, WBufNumSlots> &v) { rBuf_ = v; }
    void setWBufForTest(const std::array<int16_t, WBufNumSlots> &v) { wBuf_ = v; }

  private:
    static unsigned mArrayID(uint64_t addr) { return (addr / CArrayWordlineNums) % NumArrays; }
    static unsigned mArrayRow(uint64_t addr) { return addr % CArrayWordlineNums; }

    /** The four sub-arrays: C data in Mac-mode arrays, M data in Mem-mode. */
    std::array<std::array<uint64_t, CArrayWordlineNums>, NumArrays> arraySram_{};
    std::array<uint64_t, NumArrays> rVec_{};
    uint64_t vecBuf_ = 0;
    std::array<int16_t, WBufNumSlots> rWire_{};
    std::array<int16_t, WBufNumSlots> rBuf_{};
    std::array<int16_t, WBufNumSlots> wBuf_{};
};

} // namespace mat_fsm
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_FSM_MAT_DATAPATH_HH__
