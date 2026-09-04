#include "mat_fsm.hh"

#include <stdexcept>

namespace gem5
{
namespace mat_fsm
{

// All member defaults above (in mat_fsm.hh) already match the RTL's
// RegInit values (accWidthReg_=AccWidth32Bit, arrayCacheModeReg_=true,
// isFirstSlice_=true, everything else 0/false/Mem) -- see
// REGISTER_TABLE.md's "Registers" table for the citations. Nothing left
// to do here.
MatFSM::MatFSM() = default;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

std::array<ArrayMode, NumArrays>
MatFSM::computeArrayMode(unsigned nBuf, unsigned nCal)
{
    // CONFIRMED against Controller.scala:248-252 -- ported verbatim from
    // mat_fsm.py's compute_array_mode(). NOT a "first N lanes = MAC,
    // rest IdleMac" pattern; each lane has its own formula:
    //
    //   workingArrayNum = 4 if (nBuf+nCal)==0 else nBuf+nCal   [wire,
    //       NOT an external input -- computed here, matching main_idle's
    //       own local computation in the RTL]
    //   lane 0: always Mem
    //   lane 1: Mem if nBuf>1 else Mac
    //   lane 2: Mem if nBuf>2, elif workingArrayNum<=2: IdleMac,
    //           else Mac
    //   lane 3: IdleMac if workingArrayNum<=3 else Mac
    const unsigned workingArrayNum = (nBuf + nCal) == 0 ? 4 : (nBuf + nCal);

    std::array<ArrayMode, NumArrays> modes = {
        ArrayMode::Mem, ArrayMode::Mem, ArrayMode::Mem, ArrayMode::Mem
    };
    modes[1] = (nBuf > 1) ? ArrayMode::Mem : ArrayMode::Mac;
    if (nBuf > 2) {
        modes[2] = ArrayMode::Mem;
    } else if (workingArrayNum <= 2) {
        modes[2] = ArrayMode::IdleMac;
    } else {
        modes[2] = ArrayMode::Mac;
    }
    modes[3] = (workingArrayNum <= 3) ? ArrayMode::IdleMac : ArrayMode::Mac;
    return modes;
}

// ---------------------------------------------------------------------
// Background logic. Runs every step() call regardless of state.
// ---------------------------------------------------------------------

void
MatFSM::tickBackground(const SetUpIO &setUpIo)
{
    // ---- RegNext: M-array read-data latch -----------------------------
    // spec: mArrayDoutValid_ is RegNext of readMArrayEnWire_.
    mArrayDoutValid_ = readMArrayEnWirePrev_;
    if (mArrayDoutValid_) {
        // Pure plumbing latch -- dataIn_from_M_array is datapath-side and
        // out of scope for this control-only model (mirrors
        // mat_fsm.py's IO.dataIn_from_M_array default of 0); no control
        // decision anywhere reads rBuf_.
        rBuf_ = 0;
    }

    // ---- RegNext: wBuf accumulate pipeline -----------------------------
    writeWBuf_ = writeWBufWirePrev_;
    if (writeWBuf_) {
        // CONTROL-ONLY (explicit instruction to separate FSM control
        // from the MAC/shift/signed datapath). Only the pointer movement
        // below is control-relevant; the stored value is an inert
        // placeholder from the black-box hook.
        const unsigned idx = wbufPtrReg_;
        if (idx < WBufNumSlots) {
            wBuf_[idx] = sumOfMac_(*this);
        }
        // CONFIRMED (Controller.scala:138,214): buf_ptr_inc is
        // accWidth-dependent (+2 packs two 16b halves into one 32b
        // accumulator per element; +1 for standalone 16b elements).
        const unsigned bufPtrInc = (accWidthReg_ == AccWidth32Bit) ? 2 : 1;
        // CONFIRMED (Controller.scala:132): wbufPtrReg_ is a FIXED-WIDTH
        // register (UInt(log2Ceil(WBufNumSlots).W)) -- it wraps on
        // overflow like any hardware adder.
        wbufPtrReg_ = (wbufPtrReg_ + bufPtrInc) % WBufNumSlots;
    }

    // ---- combinational wires, recomputed fresh every cycle -------------
    // CONFIRMED (Controller.scala:152): `wbuf_ptr_reg===(segNum_in_per_word-1)`
    // -- a fixed hardware-width constant (WBufNumSlots-1), NOT nCal-1.
    isWBufPtrEnd_ = (wbufPtrReg_ == WBufNumSlots - 1);

    // CONFIRMED (Controller.scala:151):
    // `val is_last_L_block_row=(_L_vec_ptr_cur===_L_block_row)` -- exact
    // equality, not >=.
    isLastLBlockRow_ = (lVecPtrCur_ == lBlockRow_);

    (void)setUpIo;  // not consumed directly by tickBackground itself
}

// ---------------------------------------------------------------------
// State handlers. Each mirrors mat_fsm.py's same-named function 1:1.
// ---------------------------------------------------------------------

MainState
MatFSM::mainIdle(const SetUpIO &setUpIo)
{
    // spec (main_idle.a): when exec=false, stay put, write nothing.
    if (!setUpIo.exec) {
        return MainState::MainIdle;
    }

    // spec (template, GIVEN): "_C_array_EndPtr <- _R_block_row"
    cArrayEndPtr_ = setUpIo.R_block_row;
    readCArrayAddrReg_ = 0;

    // CONFIRMED against Controller.scala:238-244. The RTL's three-way Mux
    // (i==nBuf / i>nBuf / else) collapses to these two cases (i==nBuf's
    // branch value equals the i>nBuf formula evaluated at i=nBuf):
    //   i <  nBuf : 0                        (Mem-mode buffer lane --
    //               RTL just zeros it, no "active" meaning)
    //   i >= nBuf : R_base_bit + (i - nBuf)  (this lane's R bit-slice
    //               position; i==nBuf is the FIRST Mac lane, at
    //               R_base_bit exactly)
    // No "inactive sentinel" concept exists in the RTL.
    for (unsigned i = 0; i < NumArrays; ++i) {
        if (i < setUpIo.nBuf) {
            bitIdR_[i] = 0;
        } else {
            bitIdR_[i] = static_cast<int>(setUpIo.R_base_bit + (i - setUpIo.nBuf));
        }
    }

    // CONFIRMED (Controller.scala:258): independent formula, NOT derived
    // from bitID_R -- do not latch this from bitIdR_[nBuf].
    lastBitRBidId_ = static_cast<int>(setUpIo.R_base_bit) +
                      static_cast<int>(setUpIo.nCal) - 1;

    // CONFIRMED: see computeArrayMode() above.
    arrayModeReg_ = computeArrayMode(setUpIo.nBuf, setUpIo.nCal);

    // GIVEN (direct copies of the one-time setup inputs):
    accWidthReg_ = setUpIo.accWidth;
    signedL_ = setUpIo.signed_L;
    signedRLastExist_ = setUpIo.signed_R_last_exist;
    lBlockRow_ = setUpIo.L_block_row;
    lPrecisionReg_ = setUpIo.L_precision;
    lVecAddr_ = setUpIo.L_vec_fetch_addr;

    // CONFIRMED (Controller.scala:276, "Disable cache mode"): the array's
    // physical SRAM port switches from external/cache addressing to the
    // Controller's own C-/M-array addressing for the duration of this
    // command. preCheck()'s (T,T) branch is the matching restore.
    arrayCacheModeReg_ = false;

    // GIVEN: fresh command starts at row 0, bit-slice 0, first slice.
    lVecPtrCur_ = 0;
    lBitSliceIdPtr_ = 0;
    isFirstSlice_ = true;
    // CONFIRMED (Controller.scala:264): accWidth-dependent, mirrors
    // postProcess()'s branch (2) reset.
    wbufPtrReg_ = (setUpIo.accWidth == AccWidth32Bit) ? 1 : 0;
    writeMArrayRowAdrReg_ = 0;
    readMArrayRowAdrReg_ = 0;

    // CONFIRMED: mainIdle does NOT write skipReadMArray_ in the RTL at
    // all (searched Controller.scala:231-280 -- absent). Provably a
    // no-op given the FSM's reachable paths: the only path back to
    // mainIdle is preCheck()'s (T,T) branch, which is only reachable
    // when isLastLBlockRow_ was True, which is only possible if the
    // prior mainWaitL() visit's skipReadMArray_-clear already ran, or
    // postProcess()'s branch (2) explicitly set it False -- branch (3),
    // the only branch that sets it True, requires isLastLBlockRow_
    // False, which routes to preCheck()'s (F,*)/(T,F) instead, never
    // (T,T). So skipReadMArray_ is always already False entering
    // mainIdle.

    return MainState::MainWaitL;
}

MainState
MatFSM::mainWaitL(const SetUpIO & /*setUpIo*/)
{
    // CONFIRMED against Controller.scala:384-411. This state is really a
    // 3-phase sub-FSM (load_vec_state: send_L_req -> wait_L_resp ->
    // start_next) around the request_vec/response_vec Decoupled
    // handshake to AutoLoadL. Per this class's own stated design
    // (lFetchDone_, "black box #1"), that handshake -- and the
    // load_vec_state register itself -- stays deliberately out of scope
    // here; lFetchDone_() stands in for "has AutoLoadL's response come
    // back", collapsing send_L_req+wait_L_resp into one gate. What IS in
    // scope (it's Controller's own state, not AutoLoadL's) is
    // start_next's three writes:
    //   - lVecPtrCur_ += 1
    //   - mainState := skipReadMArray_ ? Cal : PreReadMArray
    //   - skipReadMArray_ := false   (CONFIRMED :408 -- a real bug if
    //     missing: a stale true left by postProcess()'s branch (3) would
    //     keep routing every SUBSEQUENT mainWaitL() visit straight to
    //     Cal, skipping preReadMArray()'s M-array prefetch)
    // Also in scope (plumbing, not part of the AutoLoadL handshake being
    // stubbed): send_L_req's write, lVecAddr_ += 1 (CONFIRMED :394).
    if (!lFetchDone_()) {
        return MainState::MainWaitL;
    }

    lVecPtrCur_ += 1;
    lVecAddr_ += 1;

    const MainState next = skipReadMArray_ ? MainState::Cal : MainState::PreReadMArray;
    skipReadMArray_ = false;
    return next;
}

MainState
MatFSM::preReadMArray(const SetUpIO & /*setUpIo*/)
{
    // CONFIRMED against Controller.scala:283-291 -- next-state is
    // unconditionally Cal in both branches (the RTL's `mainState:=cal`
    // sits outside the `when(!is_first_slice)` block).
    if (isFirstSlice_) {
        // spec (a, GIVEN): no read issued, straight to cal.
        readMArrayEnWire_ = false;
        return MainState::Cal;
    }

    // spec (b, GIVEN): readMArrayEnWire_=true, readMArrayRowAdrReg_+1;
    // dout_valid becomes visible on the FOLLOWING tickBackground() call
    // (RegNext), not this one.
    readMArrayEnWire_ = true;
    readMArrayRowAdrReg_ += 1;
    return MainState::Cal;
}

MainState
MatFSM::cal(const SetUpIO & /*setUpIo*/)
{
    // cal() -- the module this class was specifically requested for.
    // One step() call = one loop iteration's worth of work: accumulate,
    // increment address, check if equals EndPtr.
    //
    // CONFIRMED against Controller.scala:292-319, with a Chisel-semantics
    // subtlety: readCArrayAddrReg_ is a Reg, and every read of a Reg
    // within the same always-executing block sees its value from BEFORE
    // this cycle's edge (its "old" value) -- including reads that come
    // AFTER an earlier `:=` to that same register in program order (that
    // `:=` only takes effect on the NEXT edge). Two consequences:
    //
    //   1. readCArrayEn/writeWBufWire (Controller.scala:295-297) are
    //      assigned BEFORE, and unconditionally with respect to, the
    //      `when(read_C_ArrayAddr_reg===_C_array_EndPtr)` termination
    //      check at :300 -- so they still fire on the terminating cycle.
    //      cArrayEndPtr_ is an INCLUSIVE bound (addresses 0..EndPtr,
    //      i.e. EndPtr+1 total accumulate cycles), not exclusive.
    //   2. The mid-loop-flush guard (Controller.scala:310) is
    //      `is_wBuf_ptr_end && read_C_ArrayAddr_reg=/=0.U` -- using the
    //      SAME old (pre-edge) value, i.e. "was the address already
    //      nonzero entering this cycle", which suppresses a flush on
    //      cal's own very first cycle of a fresh sweep (old value 0).
    const uint64_t oldAddr = readCArrayAddrReg_;

    // CONFIRMED: unconditional, regardless of termination (point 1 above).
    writeWBufWire_ = true;
    for (unsigned i = 0; i < NumArrays; ++i) {
        readCArrayEn_[i] = (arrayModeReg_[i] == ArrayMode::Mac);
    }

    // CONFIRMED (point 2 above): gated on oldAddr != 0 too.
    if (isWBufPtrEnd_ && oldAddr != 0) {
        writeMArrayEnWire_ = true;
        if (isFirstSlice_) {
            readMArrayEnWire_ = false;
        } else {
            readMArrayEnWire_ = true;
            readMArrayRowAdrReg_ += 1;
        }
    } else {
        writeMArrayEnWire_ = false;
        readMArrayEnWire_ = false;
    }

    if (oldAddr == cArrayEndPtr_) {
        // CONFIRMED: termination -- reset and hand off to postProcess.
        readCArrayAddrReg_ = 0;
        return MainState::PostProcess;
    }

    readCArrayAddrReg_ = oldAddr + 1;
    return MainState::Cal;
}

MainState
MatFSM::postProcess(const SetUpIO & /*setUpIo*/)
{
    // CONFIRMED against Controller.scala:328-346. Branch SELECTION
    // (isWBufPtrEnd_ / elif isLastLBlockRow_ / else) and which fields
    // each branch touches are exactly right:
    //
    //   1. Branch (isWBufPtrEnd_): the RTL's body is
    //      `write_M_array_En_wire:=true.B` and NOTHING else -- wbufPtrReg_
    //      is NOT reset here (it's otherwise an ever-incrementing
    //      counter, driven unconditionally by tickBackground()'s
    //      accumulate path every time writeWBuf_ fires, wrapping on its
    //      own; isWBufPtrEnd_ is an exact-equality test against its max
    //      valid index, relying on the counter to move past that value
    //      on its own next cycle -- not on an explicit reset here).
    //   2. Branch (isLastLBlockRow_): wbufPtrReg_'s reset is
    //      accWidth-dependent (Controller.scala:340,
    //      `Mux(accWidth_reg===ACC_32BIT,1.U,0.U)`), mirroring
    //      mainIdle()'s own reset.
    if (isWBufPtrEnd_) {
        // branch (1): writeMArrayEnWire_ only. wbufPtrReg_ deliberately
        // left untouched.
        writeMArrayEnWire_ = true;

    } else if (isLastLBlockRow_) {
        // branch (2): buffer not full, but no more rows remain in this
        // bit-slice's sweep -- flush the partial contents.
        writeMArrayEnWire_ = true;
        wbufPtrReg_ = (accWidthReg_ == AccWidth32Bit) ? 1 : 0;
        skipReadMArray_ = false;

    } else {
        // branch (3): common inner case -- neither full nor last row;
        // carry the partial accumulation forward to the next row without
        // flushing. wbufPtrReg_ NOT reset (only "reset" is spec'd for
        // the other two branches, so absence here means "carries over").
        writeMArrayEnWire_ = false;
        skipReadMArray_ = true;
    }

    return MainState::PreCheck;
}

MainState
MatFSM::preCheck(const SetUpIO & /*setUpIo*/)
{
    // CONFIRMED against Controller.scala:347-383. The (T,T)/(T,F)/(F,*)
    // next-state mapping -- MainIdle / MainWaitL / MainWaitL -- is
    // exactly right, including the outer-loop-is-bit-slice /
    // inner-loop-is-L-block-row structure. Register writes accompanying
    // each transition:
    //
    //   - lVecPtrCur_ := 0 fires in BOTH (T,T) and (T,F) -- it sits
    //     OUTSIDE the bitSliceDone if/else in the RTL, at the top of the
    //     `when(is_last_L_block_row)` block.
    //   - arrayCacheModeReg_ := true fires in (T,T) ("Restore cache
    //     state" in the RTL comment) -- pairs with mainIdle()'s
    //     `:= false` ("Disable cache mode").
    //   - (T,T) does NOT touch lBitSliceIdPtr_ or isFirstSlice_ (only
    //     readCArrayAddrReg_ / readMArrayRowAdrReg_ /
    //     writeMArrayRowAdrReg_ / arrayCacheModeReg_) -- both are
    //     harmless to leave stale here since mainIdle() unconditionally
    //     re-inits them on the next exec, but writing them here would
    //     not be faithful to the RTL.
    //   - readCArrayAddrReg_ := 0 ALSO fires in (T,F), not just (T,T).
    //     Functionally redundant in practice (postProcess()/preCheck()
    //     are only ever reached via cal()'s OWN termination branch,
    //     which already zeroes readCArrayAddrReg_ right there) but
    //     included for literal RTL fidelity.
    const bool bitSliceDone = (lBitSliceIdPtr_ == lPrecisionReg_);

    if (isLastLBlockRow_) {
        // RTL: this reset sits OUTSIDE the bitSliceDone branch below --
        // it fires for BOTH (T,T) and (T,F).
        lVecPtrCur_ = 0;

        if (bitSliceDone) {
            // (T,T): fully done -> MainIdle.
            readCArrayAddrReg_ = 0;
            readMArrayRowAdrReg_ = 0;
            writeMArrayRowAdrReg_ = 0;
            // "Restore cache state" (Controller.scala:359); pairs with
            // mainIdle()'s `:= false` ("Disable cache mode").
            arrayCacheModeReg_ = true;
            return MainState::MainIdle;
        }

        // (T,F): more bit-slices remain -> MainWaitL. isFirstSlice_
        // becomes false from here on (this is now a continuation, not a
        // fresh command).
        lBitSliceIdPtr_ += 1;
        isFirstSlice_ = false;
        readCArrayAddrReg_ = 0;
        readMArrayRowAdrReg_ = 0;
        writeMArrayRowAdrReg_ = 0;
        return MainState::MainWaitL;
    }

    // (F,*): more L-block-rows remain in this bit-slice's sweep -- bare
    // transition, no register writes (RTL's `.otherwise` here is just
    // `mainState := main_wait_L`, nothing else).
    return MainState::MainWaitL;
}

// ---------------------------------------------------------------------
// The interpreter.
// ---------------------------------------------------------------------

MainState
MatFSM::dispatchCurrentState(const SetUpIO &setUpIo)
{
    switch (state_) {
      case MainState::MainIdle:
        return mainIdle(setUpIo);
      case MainState::MainWaitL:
        return mainWaitL(setUpIo);
      case MainState::PreReadMArray:
        return preReadMArray(setUpIo);
      case MainState::Cal:
        return cal(setUpIo);
      case MainState::PostProcess:
        return postProcess(setUpIo);
      case MainState::PreCheck:
        return preCheck(setUpIo);
    }
    return state_;
}

MainState
MatFSM::step(const SetUpIO &setUpIo)
{
    // Process last cycle's RegNext-delayed effects using the wires the
    // state handler computed on the PREVIOUS call, before this cycle's
    // handler overwrites them.
    tickBackground(setUpIo);

    const MainState next = dispatchCurrentState(setUpIo);

    // Snapshot this cycle's freshly-computed wires so the NEXT step()'s
    // tickBackground() sees them as "last cycle's" values.
    readMArrayEnWirePrev_ = readMArrayEnWire_;
    writeWBufWirePrev_ = writeWBufWire_;

    state_ = next;
    return state_;
}

MainState
MatFSM::runStateHandlerOnlyForTest(const SetUpIO &setUpIo)
{
    state_ = dispatchCurrentState(setUpIo);
    return state_;
}

std::vector<MainState>
MatFSM::runUntil(const SetUpIO &setUpIo, MainState target, unsigned maxSteps)
{
    std::vector<MainState> trace{state_};
    for (unsigned i = 0; i < maxSteps; ++i) {
        if (state_ == target) {
            return trace;
        }
        step(setUpIo);
        trace.push_back(state_);
    }
    throw std::runtime_error(
        "MatFSM::runUntil: did not reach target state within maxSteps");
}

} // namespace mat_fsm
} // namespace gem5
