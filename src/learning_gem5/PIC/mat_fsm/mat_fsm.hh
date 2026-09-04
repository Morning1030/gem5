#ifndef __LEARNING_GEM5_PIC_MAT_FSM_MAT_FSM_HH__
#define __LEARNING_GEM5_PIC_MAT_FSM_MAT_FSM_HH__

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace gem5
{
namespace mat_fsm
{

// ---------------------------------------------------------------------
// Constants -- CONFIRMED against Controller.scala; see REGISTER_TABLE.md
// in this directory for the full citations these mirror.
// ---------------------------------------------------------------------

// GIVEN: spec lists "bitID_R[4]" and "arrayMode_reg[4]" explicitly.
constexpr unsigned NumArrays = 4;

// CONFIRMED (Controller.scala:122,132,152 / PolymorPIC_Kernal_Config's
// segNum_in_per_word=(bitlineNums/16).toInt, default bitlineNums=64 ->
// 4): wBuf's slot count is a fixed HARDWARE-WIDTH constant, unrelated to
// nCal (a runtime ISA field). wbufPtrReg_ mirrors the RTL's
// UInt(log2Ceil(segNum_in_per_word).W) -- a fixed-width register that
// wraps on overflow like any hardware adder (see tickBackground()).
constexpr unsigned WBufNumSlots = 4;

// CONFIRMED (CalInfo.scala: ACC_32BIT=true.B/ACC_16BIT=false.B). accWidth
// is represented as an unsigned bit-width (matching SetUpIO::accWidth)
// rather than the RTL's raw Bool -- these are the two values every
// accWidth-dependent comparison in this file is written against.
constexpr unsigned AccWidth32Bit = 32;
constexpr unsigned AccWidth16Bit = 16;

/// Per-lane array mode (arrayModeReg_[i]). CONFIRMED against
/// Controller.scala:249-252 / ModeInfo.scala -- three distinct RTL
/// modes: Mem (buffer/partial-sum storage lane), Mac (actively reading
/// C-array + accumulating), IdleMac (participates in neither -- a lane
/// left unused when fewer than 4 arrays are configured to work this
/// pass). cal()'s readCArrayEn gates on exact-Mac, which is false for
/// both Mem and IdleMac.
enum class ArrayMode
{
    Mem,
    Mac,
    IdleMac,
};

/// mainState. Same six states as mat_fsm.py's STATE_HANDLERS keys.
enum class MainState
{
    MainIdle,
    MainWaitL,
    PreReadMArray,
    Cal,
    PostProcess,
    PreCheck,
};

/// The one-time command setup fields consumed by mainIdle() (mirrors
/// mat_fsm.py's SetUpIO / io.set_up_io.*). Field set matches
/// CAL_Payload/ExeParams in learning_gem5/PIC/scheduler.hh and
/// pic_protocol.hh -- this struct's eventual gem5-side caller.
struct SetUpIO
{
    bool exec = false;
    unsigned nBuf = 0;
    unsigned nCal = 0;
    unsigned accWidth = AccWidth16Bit;
    unsigned R_base_bit = 0;
    unsigned R_block_row = 0;
    unsigned L_block_row = 0;
    unsigned L_precision = 0;
    uint64_t L_vec_fetch_addr = 0;
    bool signed_L = false;
    bool signed_R_last_exist = false;
};

/**
 * Functional (dataflow-only, timing-ignored) C++ port of mat_fsm.py's
 * MatModel -- the mat_fsm "cal()" CONTROL state machine. Verifies that
 * each state's control dataflow (what gets read/written/compared, and
 * which state comes next) matches the RTL (Controller.scala), independent
 * of (a) how many real hardware cycles a state would take, and (b) the
 * actual MAC/shift/signed-arithmetic datapath -- that stays a swappable
 * black box (see sumOfMac()) that no control decision here ever inspects.
 *
 * Deliberately introduces NO gem5 event/tick concepts (no ClockedObject,
 * no EventFunctionWrapper, no per-cycle scheduling) -- per the original
 * plan's explicit instruction to keep this stage's port free of timing
 * machinery. step() executes exactly one state's entire logic per call,
 * mirroring MatModel.step() 1:1; a caller (a future gem5 SimObject
 * wrapper, or a plain unit-test driver) decides when/how often to call
 * it.
 *
 * Ported from, and kept in lock-step with, mat_fsm.py -- every
 * Controller.scala-confirmed behavior there has a matching comment here
 * citing the same line numbers. See REGISTER_TABLE.md for the full
 * register/wire inventory this class implements.
 */
class MatFSM
{
  public:
    using LFetchDone = std::function<bool()>;
    // Black-box datapath hook (mirrors IO.sum_of_mac): called from
    // tickBackground() purely to keep the wiring point alive for a later
    // real datapath. Its return value is stored into wBuf_[wbufPtrReg_]
    // and NEVER read by any control decision (isWBufPtrEnd_ etc. are
    // pointer/counter-driven only). Takes `const MatFSM &` rather than
    // a mutable reference so the hook cannot accidentally influence
    // control state -- stronger than the Python model could enforce.
    using SumOfMac = std::function<int(const MatFSM &)>;

    MatFSM();

    /// Runs exactly one state's worth of logic (tickBackground() then
    /// the current state's handler), mirroring MatModel.step().
    MainState step(const SetUpIO &setUpIo);

    /// Test-only: runs ONLY the current state's handler, skipping
    /// tickBackground() and the RegNext `_prev` snapshot. Mirrors
    /// test_mat_fsm.py calling a state handler (e.g. `cal(regs, io)`)
    /// directly rather than through MatModel.step() -- lets a test poke
    /// a wire field (e.g. setIsWBufPtrEndForTest) and see the handler
    /// react to exactly that value, without step()'s tickBackground()
    /// immediately recomputing it out from under the test.
    MainState runStateHandlerOnlyForTest(const SetUpIO &setUpIo);

    MainState state() const { return state_; }

    // ---- Register accessors (read-only; for tests/introspection) -----
    // One accessor per persisted register named in REGISTER_TABLE.md.
    uint64_t cArrayEndPtr() const { return cArrayEndPtr_; }
    uint64_t readCArrayAddrReg() const { return readCArrayAddrReg_; }
    const std::array<int, NumArrays> &bitIdR() const { return bitIdR_; }
    const std::array<ArrayMode, NumArrays> &arrayModeReg() const { return arrayModeReg_; }
    unsigned accWidthReg() const { return accWidthReg_; }
    int lastBitRBidId() const { return lastBitRBidId_; }
    bool signedL() const { return signedL_; }
    bool signedRLastExist() const { return signedRLastExist_; }
    unsigned wbufPtrReg() const { return wbufPtrReg_; }
    uint64_t lVecPtrCur() const { return lVecPtrCur_; }
    uint64_t lBlockRow() const { return lBlockRow_; }
    uint64_t lPrecisionReg() const { return lPrecisionReg_; }
    uint64_t lBitSliceIdPtr() const { return lBitSliceIdPtr_; }
    uint64_t lVecAddr() const { return lVecAddr_; }
    bool isFirstSlice() const { return isFirstSlice_; }
    bool arrayCacheModeReg() const { return arrayCacheModeReg_; }
    bool skipReadMArray() const { return skipReadMArray_; }
    uint64_t writeMArrayRowAdrReg() const { return writeMArrayRowAdrReg_; }
    uint64_t readMArrayRowAdrReg() const { return readMArrayRowAdrReg_; }

    // ---- Wire accessors (valid immediately after step(); combinational,
    // recomputed every call -- mirrors the *_wire fields in regs) ------
    bool writeWBufWire() const { return writeWBufWire_; }
    bool readMArrayEnWire() const { return readMArrayEnWire_; }
    bool writeMArrayEnWire() const { return writeMArrayEnWire_; }
    bool mArrayDoutValid() const { return mArrayDoutValid_; }
    bool isWBufPtrEnd() const { return isWBufPtrEnd_; }
    bool isLastLBlockRow() const { return isLastLBlockRow_; }
    const std::array<bool, NumArrays> &readCArrayEn() const { return readCArrayEn_; }

    // ---- Test-only mutators (mirror the Python tests setting `regs[...]`
    // directly to drive a state handler in isolation without running the
    // whole FSM up to that point) ----
    void setStateForTest(MainState s) { state_ = s; }
    void setReadCArrayAddrRegForTest(uint64_t v) { readCArrayAddrReg_ = v; }
    void setCArrayEndPtrForTest(uint64_t v) { cArrayEndPtr_ = v; }
    void setArrayModeRegForTest(const std::array<ArrayMode, NumArrays> &v) { arrayModeReg_ = v; }
    void setIsFirstSliceForTest(bool v) { isFirstSlice_ = v; }
    void setIsWBufPtrEndForTest(bool v) { isWBufPtrEnd_ = v; }
    void setIsLastLBlockRowForTest(bool v) { isLastLBlockRow_ = v; }
    void setSkipReadMArrayForTest(bool v) { skipReadMArray_ = v; }
    void setWbufPtrRegForTest(unsigned v) { wbufPtrReg_ = v; }
    void setLVecPtrCurForTest(uint64_t v) { lVecPtrCur_ = v; }
    void setLVecAddrForTest(uint64_t v) { lVecAddr_ = v; }
    void setLBitSliceIdPtrForTest(uint64_t v) { lBitSliceIdPtr_ = v; }
    void setLPrecisionRegForTest(uint64_t v) { lPrecisionReg_ = v; }
    void setAccWidthRegForTest(unsigned v) { accWidthReg_ = v; }
    void setReadMArrayRowAdrRegForTest(uint64_t v) { readMArrayRowAdrReg_ = v; }

    // ---- Replaceable black boxes (mirrors IO.l_fetch_done / IO.sum_of_mac) ----
    void setLFetchDone(LFetchDone cb) { lFetchDone_ = std::move(cb); }
    void setSumOfMac(SumOfMac cb) { sumOfMac_ = std::move(cb); }

    /// Test/debug helper mirroring MatModel.run_until(): steps until
    /// `state() == target`, returning the full state-sequence trace
    /// (target included as the last entry). Throws std::runtime_error if
    /// not reached within maxSteps.
    std::vector<MainState> runUntil(const SetUpIO &setUpIo, MainState target,
                                     unsigned maxSteps = 10000);

  private:
    MainState dispatchCurrentState(const SetUpIO &setUpIo);

    void tickBackground(const SetUpIO &setUpIo);

    MainState mainIdle(const SetUpIO &setUpIo);
    MainState mainWaitL(const SetUpIO &setUpIo);
    MainState preReadMArray(const SetUpIO &setUpIo);
    MainState cal(const SetUpIO &setUpIo);
    MainState postProcess(const SetUpIO &setUpIo);
    MainState preCheck(const SetUpIO &setUpIo);

    static std::array<ArrayMode, NumArrays> computeArrayMode(unsigned nBuf, unsigned nCal);

    MainState state_ = MainState::MainIdle;

    // ---- persisted registers (see REGISTER_TABLE.md) ------------------
    uint64_t cArrayEndPtr_ = 0;
    uint64_t readCArrayAddrReg_ = 0;
    std::array<int, NumArrays> bitIdR_{};
    // CONFIRMED reset default: Mem (RegInit(VecInit(Seq.fill(4)(0.U(...)))),
    // PICMode_Mem="b00"); ArrayMode::Mem is enumerator 0 so `{}` zero-init
    // already gives the right default -- see constructor.
    std::array<ArrayMode, NumArrays> arrayModeReg_{};
    // CONFIRMED: RegInit(ACC_32BIT), Controller.scala:137.
    unsigned accWidthReg_ = AccWidth32Bit;
    int lastBitRBidId_ = 0;
    bool signedL_ = false;
    bool signedRLastExist_ = false;
    // Inert placeholder storage -- values never read by any control
    // decision (see sumOfMac_ docstring above).
    std::array<int, WBufNumSlots> wBuf_{};
    unsigned wbufPtrReg_ = 0;
    uint64_t lVecPtrCur_ = 0;
    uint64_t lBlockRow_ = 0;
    uint64_t lPrecisionReg_ = 0;
    uint64_t lBitSliceIdPtr_ = 0;
    uint64_t lVecAddr_ = 0;
    bool isFirstSlice_ = true;
    // CONFIRMED: RegInit(true.B), Controller.scala:135.
    bool arrayCacheModeReg_ = true;
    bool skipReadMArray_ = false;
    uint64_t writeMArrayRowAdrReg_ = 0;
    uint64_t readMArrayRowAdrReg_ = 0;
    int rBuf_ = 0;
    bool writeWBuf_ = false;  // RegNext of writeWBufWire_

    // ---- one-cycle-delay shadow copies (explicit RegNext plumbing, per
    // mat_fsm.py's tick_background() using *_prev fields) --------------
    bool readMArrayEnWirePrev_ = false;
    bool writeWBufWirePrev_ = false;

    // ---- wires (combinational; recomputed every step()) ---------------
    bool writeWBufWire_ = false;
    bool readMArrayEnWire_ = false;
    bool writeMArrayEnWire_ = false;
    bool mArrayDoutValid_ = false;
    bool isWBufPtrEnd_ = false;
    bool isLastLBlockRow_ = false;
    std::array<bool, NumArrays> readCArrayEn_{};

    LFetchDone lFetchDone_ = []() { return true; };
    SumOfMac sumOfMac_ = [](const MatFSM &) { return 0; };
};

} // namespace mat_fsm
} // namespace gem5

#endif // __LEARNING_GEM5_PIC_MAT_FSM_MAT_FSM_HH__
