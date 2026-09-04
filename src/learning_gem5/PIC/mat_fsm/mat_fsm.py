"""
Functional (dataflow-only, timing-ignored) model of the mat_fsm "cal()"
control state machine.

Purpose: verify that each state's *control* dataflow -- what gets
read/written/compared, and which state comes next -- matches spec,
independent of (a) how many real hardware cycles a state would take, and
(b) the actual MAC/shift/signed-arithmetic datapath. Per explicit
instruction, this file models CONTROL ONLY: the states, transition
conditions, and the control registers/counters those transitions depend
on. The datapath (sum_of_mac, left_shift_bias(i), signed(i)) is a
swappable black box that this model never inspects for any control
decision -- see IO.sum_of_mac below.

Every state handler has the signature (regs, io) -> next_state_name and
is a pure function, so it stays trivially unit-testable and, later,
trivially portable into a gem5 SimObject (same signature, different
"regs"/"io" wiring -- see file docstring in the eventual .hh/.cc port).

step() executes ONE state's entire logic per call -- NOT one hardware
cycle's worth of that logic -- except `cal`, which self-loops once per
call by design (the loop-body-per-call constraint is explicit in the
plan this implements: "accumulate, increment address, check if equals
EndPtr" -- one iteration per step() call, not the whole loop at once).

ASSUMPTION markers below flag control logic inferred from the test-plan
description rather than given directly by a written spec. See
REGISTER_TABLE.md in this directory for the full reasoning behind each
one and a place to record corrections. Everything NOT marked ASSUMPTION
is either stated directly in the plan or a direct mechanical consequence
of something that is (e.g. "reset to 0" literally says reset to 0).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# GIVEN: spec lists "bitID_R[4]" and "arrayMode_reg[4]" explicitly.
NUM_ARRAYS = 4

# CONFIRMED (Controller.scala:122,132,152 / PolymorPIC_Kernal_Config's
# segNum_in_per_word=(bitlineNums/16).toInt, default bitlineNums=64 -> 4):
# wBuf's slot count is a fixed HARDWARE-WIDTH constant (segNum_in_per_word),
# completely unrelated to nCal (a runtime ISA field) -- REGISTER_TABLE.md's
# assumption #5 guessed wbuf_ptr_reg's bound was nCal-1; it's actually this,
# always. wbuf_ptr_reg itself is declared UInt(log2Ceil(segNum_in_per_word).W)
# -- a FIXED-WIDTH register that wraps on overflow like any hardware adder.
WBUF_NUM_SLOTS = 4

# CONFIRMED (CalInfo.scala: ACC_32BIT=true.B/ACC_16BIT=false.B). This model
# represents accWidth as an int (SetUpIO.accWidth: int = 16, matching a
# real bit width) rather than the RTL's raw Bool -- these are the two
# values that convention uses, and every accWidth-dependent comparison in
# this file (Controller.scala's `accWidth_reg===ACC_32BIT`) is written
# against them.
ACC_32BIT = 32
ACC_16BIT = 16


class ArrayMode(Enum):
    """Per-lane array mode (arrayMode_reg[i]). CONFIRMED against
    Controller.scala:249-252 / ModeInfo.scala -- THREE distinct RTL
    modes, not two: Mem (buffer/partial-sum storage lane), Mac (actively
    reading C-array + accumulating), IdleMac (participates in neither --
    a lane left unused when fewer than 4 arrays are configured to work
    this pass). cal()'s read_C_ArrayEn gates on isPICMode_Mac(...)
    specifically (Controller.scala:296), which is False for BOTH Mem and
    IdleMac, so the model's previous 2-state {IDLE, MAC} enum happened to
    get read_C_ArrayEn right by accident -- but arrayMode_reg's own
    per-lane VALUES were wrong regardless (see compute_array_mode)."""
    MEM = auto()
    MAC = auto()
    IDLE_MAC = auto()


# ---------------------------------------------------------------------------
# External inputs (environment) -- Step 1 "External inputs" row.
# ---------------------------------------------------------------------------

@dataclass
class SetUpIO:
    """io.set_up_io.* -- the one-time command setup fields consumed by
    main_idle. Field set matches CAL_Payload/ExeParams in
    learning_gem5/PIC/scheduler.hh and pic_protocol.hh (this module's
    eventual gem5-side callers), plus the extra fields main_idle's
    template names that aren't in those structs yet (nBuf, nCal,
    _L_precision, _L_vec_fetch_addr, _R_block_row). CONFIRMED: this
    dataclass previously also had a `working_array_num` field -- removed,
    since it isn't a real external input in the RTL (nor in CAL_Payload):
    it's a wire computed from nBuf+nCal inside main_idle itself
    (Controller.scala:248), now done in compute_array_mode()."""
    exec: bool = False
    nBuf: int = 0
    nCal: int = 0
    accWidth: int = 16
    _R_base_bit: int = 0
    _R_block_row: int = 0
    _L_block_row: int = 0
    _L_precision: int = 0
    _L_vec_fetch_addr: int = 0
    signed_L: bool = False
    signed_R_last_exist: bool = False


@dataclass
class IO:
    set_up_io: SetUpIO = field(default_factory=SetUpIO)

    # Wire carrying whatever the M-array read returns this cycle. Only
    # ever latched into rBuf (pure plumbing) -- no control decision reads
    # its value, only whether a read was issued at all.
    dataIn_from_M_array: int = 0

    # ---- Replaceable black box #1 (plan Step 6 / "reserve the interface")
    # AutoLoadL completion handshake. Defaults to "always immediately
    # done" per the plan's stub instruction.
    l_fetch_done: Callable[[], bool] = field(default=lambda: True)

    # ---- Replaceable black box #2 (plan Step 6, tightened per explicit
    # follow-up instruction: FSM control is verified independently of the
    # MAC/shift/signed datapath). This is called from tick_background()
    # purely to keep the wiring point alive for later -- its return value
    # is stored into wBuf[wbuf_ptr_reg] and NEVER read by any control
    # decision (is_wBuf_ptr_end etc. are pointer/counter-driven only).
    # Default: an inert placeholder, deliberately not "a simple sum" --
    # using a real number here would invite someone to accidentally start
    # depending on it for control, which is exactly what we're avoiding.
    sum_of_mac: Callable[[dict, "IO"], object] = field(
        default=lambda regs, io: None
    )


# ---------------------------------------------------------------------------
# Register file (Step 1 "Registers" + "Wires" rows)
# ---------------------------------------------------------------------------

def initial_regs() -> Dict:
    """All regs/wires from the inventory, at their power-on-reset values.
    main_idle overwrites most of these on the first `exec`; values here
    mostly matter for asserting main_idle's (a) "when exec=false ... no
    registers written" test case, and for state before the first command."""
    return {
        # ---- persisted registers -------------------------------------
        "_C_array_EndPtr": 0,
        "read_C_ArrayAddr_reg": 0,
        "bitID_R": [0] * NUM_ARRAYS,
        "arrayMode_reg": [ArrayMode.MEM] * NUM_ARRAYS,
        # CONFIRMED: RegInit(ACC_32BIT) (Controller.scala:137) -- was 0,
        # which matches neither ACC_32BIT nor ACC_16BIT and silently
        # behaved like 16-bit mode in every accWidth-dependent comparison
        # (0 != ACC_32BIT). RTL's true reset default is 32-bit mode.
        "accWidth_reg": ACC_32BIT,
        "lastBitR_bidID": 0,
        "signed_L": False,
        "signed_R_last_exist": False,
        # CONFIRMED: WBUF_NUM_SLOTS (segNum_in_per_word), not a placeholder
        # -- was [None]*4096, an arbitrary oversized upper bound.
        "wBuf": [None] * WBUF_NUM_SLOTS,
        "wbuf_ptr_reg": 0,
        "_L_vec_ptr_cur": 0,
        "_L_block_row": 0,
        "_L_precision_reg": 0,
        "_L_bitSlice_ID_ptr": 0,
        "_L_vec_addr": 0,
        "is_first_slice": True,
        # CONFIRMED: RegInit(true.B) (Controller.scala:135) -- normal
        # cache mode is the reset default. main_idle drives it False on
        # exec ("Disable cache mode"); pre_check's (T,T) branch restores
        # it True on command completion ("Restore cache state").
        "arrayCacheMode_reg": True,
        "skip_read_M_array": False,
        "write_M_Array_RowAdr_reg": 0,
        "read_M_Array_RowAdr_reg": 0,
        "rBuf": None,
        "write_wBuf": False,          # RegNext of write_wBuf_wire

        # ---- one-cycle-delay shadow copies (explicit RegNext plumbing,
        # per the plan's tick_background() sketch using *_prev names) ----
        "_read_M_array_En_wire_prev": False,
        "write_wBuf_wire_prev": False,

        # ---- wires (combinational; recomputed every step()) -----------
        "write_wBuf_wire": False,
        "read_M_array_En_wire": False,
        "write_M_array_En_wire": False,
        "_M_array_dout_valid": False,
        "is_wBuf_ptr_end": False,
        "is_last_L_block_row": False,
        "read_C_ArrayEn": [False] * NUM_ARRAYS,
    }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def compute_array_mode(nBuf: int, nCal: int) -> List[ArrayMode]:
    """arrayMode_reg's four lanes. CONFIRMED against Controller.scala:
    248-252 -- this is NOT a "first N lanes = MAC, rest IDLE" pattern
    (the model's previous placeholder); each lane has its own formula:

        working_array_num = 4 if (nBuf+nCal)==0 else nBuf+nCal   [wire,
            NOT an external input -- see SetUpIO's docstring]
        lane 0: always Mem
        lane 1: Mem if nBuf>1 else Mac
        lane 2: Mem if nBuf>2, elif working_array_num<=2: IdleMac,
                else Mac
        lane 3: IdleMac if working_array_num<=3 else Mac
    """
    working_array_num = 4 if (nBuf + nCal) == 0 else (nBuf + nCal)

    modes = [ArrayMode.MEM, ArrayMode.MEM, ArrayMode.MEM, ArrayMode.MEM]
    modes[1] = ArrayMode.MEM if nBuf > 1 else ArrayMode.MAC
    if nBuf > 2:
        modes[2] = ArrayMode.MEM
    elif working_array_num <= 2:
        modes[2] = ArrayMode.IDLE_MAC
    else:
        modes[2] = ArrayMode.MAC
    modes[3] = ArrayMode.IDLE_MAC if working_array_num <= 3 else ArrayMode.MAC
    return modes


# ---------------------------------------------------------------------------
# Background logic -- Step 3. Runs every step() call regardless of state.
# ---------------------------------------------------------------------------

def tick_background(regs: Dict, io: IO) -> None:
    """The effect of "combinational logic + RegNext" for one cycle.
    Must run BEFORE the state handler (see step() below) so that this
    cycle only sees last cycle's wire values for the RegNext-gated
    effects, and the state handler's freshly-computed wires only become
    visible to tick_background on the NEXT step() call."""

    # ---- RegNext: M-array read-data latch ----------------------------
    # spec: "_M_array_dout_valid" is RegNext of read_M_array_En_wire; the
    # following tick's tick_background shows dout_valid=true, per
    # pre_read_M_array's test focus (b).
    regs["_M_array_dout_valid"] = regs["_read_M_array_En_wire_prev"]
    if regs["_M_array_dout_valid"]:
        # pure plumbing latch -- not a computed value, no control decision
        # reads rBuf.
        regs["rBuf"] = io.dataIn_from_M_array

    # ---- RegNext: wBuf accumulate pipeline ----------------------------
    regs["write_wBuf"] = regs["write_wBuf_wire_prev"]
    if regs["write_wBuf"]:
        # spec: "do_wbuf_accumulate" -- CONTROL-ONLY here (explicit
        # instruction to separate FSM control from the MAC/shift/signed
        # datapath). Only the pointer movement is control-relevant;
        # the stored value is an inert placeholder from the black-box
        # hook (see IO.sum_of_mac docstring).
        idx = regs["wbuf_ptr_reg"]
        if 0 <= idx < len(regs["wBuf"]):
            regs["wBuf"][idx] = io.sum_of_mac(regs, io)
        # CONFIRMED (Controller.scala:138,214): buf_ptr_inc is
        # accWidth-dependent (+2 packs two 16b halves into one 32b
        # accumulator per element; +1 for standalone 16b elements) -- was
        # unconditionally +1 here.
        buf_ptr_inc = 2 if regs["accWidth_reg"] == ACC_32BIT else 1
        # CONFIRMED (Controller.scala:132): wbuf_ptr_reg is a
        # FIXED-WIDTH register (UInt(log2Ceil(WBUF_NUM_SLOTS).W)) -- it
        # wraps on overflow like any hardware adder. Was an unbounded
        # Python int here, growing forever across sweeps/commands.
        regs["wbuf_ptr_reg"] = (regs["wbuf_ptr_reg"] + buf_ptr_inc) % WBUF_NUM_SLOTS

    # ---- combinational wires, recomputed fresh every cycle ------------
    # CONFIRMED (Controller.scala:152): `wbuf_ptr_reg===(segNum_in_per_word-1)`
    # -- a fixed hardware-width constant (WBUF_NUM_SLOTS-1), NOT nCal-1.
    # Previously used nCal as a stand-in for wBuf's capacity (assumption
    # #5) and, combined with the unbounded-growth bug above, this made
    # is_wBuf_ptr_end latch True forever once wbuf_ptr_reg passed
    # nCal-1 -- it never came back down because nothing ever wrapped.
    regs["is_wBuf_ptr_end"] = regs["wbuf_ptr_reg"] == WBUF_NUM_SLOTS - 1

    # CONFIRMED against Controller.scala:151 --
    # `val is_last_L_block_row=(_L_vec_ptr_cur===_L_block_row)` -- an
    # exact equality compare, not >=. Kept as `==` here for literal
    # fidelity, even though the two are behaviorally equivalent under
    # this model's own invariants (_L_vec_ptr_cur only ever increments by
    # 1 and is reset to 0 in lockstep with _L_block_row, so it can never
    # overshoot).
    regs["is_last_L_block_row"] = regs["_L_vec_ptr_cur"] == regs["_L_block_row"]


# ---------------------------------------------------------------------------
# State handlers -- Step 2. Each is a pure (regs, io) -> next_state function.
# ---------------------------------------------------------------------------

def main_idle(regs: Dict, io: IO) -> str:
    setup_io = io.set_up_io

    # spec (main_idle.a): when exec=false, stay put, write nothing.
    if not setup_io.exec:
        return "main_idle"

    # spec (template, GIVEN): "_C_array_EndPtr <- _R_block_row"
    regs["_C_array_EndPtr"] = setup_io._R_block_row
    regs["read_C_ArrayAddr_reg"] = 0

    # CONFIRMED against Controller.scala:238-244. The RTL's three-way Mux
    # (i==nBuf / i>nBuf / else) collapses to these two cases (i==nBuf's
    # branch value equals the i>nBuf formula evaluated at i=nBuf):
    #   i <  nBuf : 0                        (Mem-mode buffer lane --
    #               RTL just zeros it, no "active" meaning)
    #   i >= nBuf : _R_base_bit + (i - nBuf) (this lane's R bit-slice
    #               position; i==nBuf is the FIRST Mac lane, at
    #               _R_base_bit exactly)
    # No "inactive sentinel" concept exists in the RTL -- every lane gets
    # a real UInt value (INACTIVE_BIT_ID has been removed).
    for i in range(NUM_ARRAYS):
        if i < setup_io.nBuf:
            regs["bitID_R"][i] = 0
        else:
            regs["bitID_R"][i] = setup_io._R_base_bit + (i - setup_io.nBuf)

    # CONFIRMED (Controller.scala:258): independent formula, NOT derived
    # from bitID_R -- was previously (wrongly) latched from bitID_R[nBuf].
    regs["lastBitR_bidID"] = setup_io._R_base_bit + setup_io.nCal - 1

    # CONFIRMED: see compute_array_mode() docstring.
    regs["arrayMode_reg"] = compute_array_mode(setup_io.nBuf, setup_io.nCal)

    # GIVEN (direct copies of the one-time setup inputs):
    regs["accWidth_reg"] = setup_io.accWidth
    regs["signed_L"] = setup_io.signed_L
    regs["signed_R_last_exist"] = setup_io.signed_R_last_exist
    regs["_L_block_row"] = setup_io._L_block_row
    regs["_L_precision_reg"] = setup_io._L_precision
    regs["_L_vec_addr"] = setup_io._L_vec_fetch_addr

    # CONFIRMED (Controller.scala:276, "Disable cache mode"): the array's
    # physical SRAM port switches from external/cache addressing to the
    # Controller's own C-/M-array addressing for the duration of this
    # command. pre_check's (T,T) branch is the matching restore.
    regs["arrayCacheMode_reg"] = False

    # GIVEN: fresh command starts at row 0, bit-slice 0, first slice.
    regs["_L_vec_ptr_cur"] = 0
    regs["_L_bitSlice_ID_ptr"] = 0
    regs["is_first_slice"] = True
    # CONFIRMED (Controller.scala:264): accWidth-dependent, mirrors
    # post_process branch (2)'s reset -- was a bare 0 here (only correct
    # for 16-bit mode; 32-bit mode needs 1).
    regs["wbuf_ptr_reg"] = 1 if setup_io.accWidth == ACC_32BIT else 0
    regs["write_M_Array_RowAdr_reg"] = 0
    regs["read_M_Array_RowAdr_reg"] = 0

    # CONFIRMED: main_idle does NOT write skip_read_M_array in the RTL at
    # all (searched Controller.scala:231-280 -- it's absent). Previously
    # this file explicitly reset it False here; removed for literal
    # fidelity. Provably a no-op either way under normal operation: the
    # only path back to main_idle is pre_check's (T,T) branch, which is
    # only reachable when is_last_L_block_row is True, which is only
    # possible if the prior main_wait_L visit's skip_read_M_array-clear
    # (see main_wait_L) already ran, or post_process's branch (2)
    # (is_last_L_block_row) explicitly set it False -- branch (3), the
    # only branch that sets it True, requires is_last_L_block_row False,
    # which routes to pre_check's (F,*)/(T,F) instead, never (T,T). So
    # skip_read_M_array is always already False entering main_idle.
    return "main_wait_L"


def main_wait_L(regs: Dict, io: IO) -> str:
    """CONFIRMED against Controller.scala:384-411. This state is really a
    3-phase sub-FSM (load_vec_state: send_L_req -> wait_L_resp ->
    start_next) around the request_vec/response_vec Decoupled handshake
    to AutoLoadL. Per this file's own stated design (IO.l_fetch_done
    docstring, "Replaceable black box #1"), that handshake -- and the
    load_vec_state register itself -- stays deliberately out of scope
    here; io.l_fetch_done() stands in for "has AutoLoadL's response come
    back", collapsing send_L_req+wait_L_resp into one gate. What IS in
    scope (it's Controller's own state, not AutoLoadL's) is start_next's
    three writes, which this file already had two of:
      - _L_vec_ptr_cur += 1                          (already correct)
      - mainState := skip_read_M_array ? cal : pre_read_M_array
                                                       (already correct)
      - skip_read_M_array := false.B                 (was MISSING -- a
        real bug: without this clear, a True left by post_process's
        branch (3) would keep routing every SUBSEQUENT main_wait_L visit
        straight to cal, skipping pre_read_M_array's M-array prefetch,
        even on visits where nothing re-armed skip_read_M_array).
    Also added (send_L_req's write, in scope since it's plumbing
    -- not part of the AutoLoadL handshake being stubbed):
      - _L_vec_addr += 1
    """
    if not io.l_fetch_done():
        return "main_wait_L"

    regs["_L_vec_ptr_cur"] += 1
    # CONFIRMED (Controller.scala:394, send_L_req): the request payload
    # counter advances once per row requested. Not consumed by any
    # control decision in this file (datapath-side / AutoLoadL-facing),
    # tracked here for register-write fidelity.
    regs["_L_vec_addr"] += 1

    next_state = "cal" if regs["skip_read_M_array"] else "pre_read_M_array"
    # CONFIRMED (Controller.scala:408): cleared unconditionally after use.
    regs["skip_read_M_array"] = False
    return next_state


def pre_read_M_array(regs: Dict, io: IO) -> str:
    """CONFIRMED against Controller.scala:283-291 -- exactly right as
    written, no mismatches found (next-state is unconditionally "cal" in
    both branches; the RTL's `mainState:=cal` sits outside the
    `when(!is_first_slice)` block, matching this function's structure)."""
    if regs["is_first_slice"]:
        # spec (pre_read_M_array.a, GIVEN): no read issued, straight to cal.
        regs["read_M_array_En_wire"] = False
        return "cal"

    # spec (pre_read_M_array.b, GIVEN): read_M_array_En_wire=true,
    # read_M_Array_RowAdr_reg+1; dout_valid becomes visible on the
    # FOLLOWING tick_background() call (RegNext), not this one.
    regs["read_M_array_En_wire"] = True
    regs["read_M_Array_RowAdr_reg"] += 1
    return "cal"


def cal(regs: Dict, io: IO) -> str:
    """cal() -- the module this file was specifically requested for.
    One step() call = one loop iteration's worth of work: accumulate,
    increment address, check if equals EndPtr (explicit constraint from
    the plan -- do NOT run the whole loop in one call).

    CONFIRMED against Controller.scala:292-319, with a Chisel-semantics
    subtlety this file's earlier version got wrong: read_C_ArrayAddr_reg
    is a Reg, and EVERY read of a Reg within the same always-executing
    block sees its value from BEFORE this cycle's edge (its "old" value)
    -- including reads that come AFTER an earlier `:=` to that same
    register in program order (that `:=` only takes effect on the NEXT
    edge). Two control-flow bugs followed from missing this:

      1. `read_C_ArrayEn`/`write_wBuf_wire` (Controller.scala:295-297)
         are assigned BEFORE, and unconditionally with respect to, the
         `when(read_C_ArrayAddr_reg===_C_array_EndPtr)` termination check
         at :300 -- so they still fire on the terminating cycle.
         _C_array_EndPtr is an INCLUSIVE bound (addresses 0..EndPtr,
         i.e. EndPtr+1 total accumulate cycles), not exclusive. The old
         code's early-return on termination skipped that last cycle's
         accumulate/read_C_ArrayEn entirely.
      2. The mid-loop-flush guard (Controller.scala:310) is
         `is_wBuf_ptr_end && read_C_ArrayAddr_reg=/=0.U` -- using the
         SAME old (pre-edge) value, i.e. "was the address already
         nonzero entering this cycle", which suppresses a flush on cal's
         own very first cycle of a fresh sweep (old value 0). The
         `=/=0.U` half was missing entirely.
    """
    old_addr = regs["read_C_ArrayAddr_reg"]

    # CONFIRMED: unconditional, regardless of termination (see docstring
    # point 1).
    regs["write_wBuf_wire"] = True
    regs["read_C_ArrayEn"] = [
        regs["arrayMode_reg"][i] == ArrayMode.MAC for i in range(NUM_ARRAYS)
    ]

    # CONFIRMED (docstring point 2): gated on old_addr != 0 too.
    if regs["is_wBuf_ptr_end"] and old_addr != 0:
        regs["write_M_array_En_wire"] = True
        if regs["is_first_slice"]:
            regs["read_M_array_En_wire"] = False
        else:
            regs["read_M_array_En_wire"] = True
            regs["read_M_Array_RowAdr_reg"] += 1
    else:
        regs["write_M_array_En_wire"] = False
        regs["read_M_array_En_wire"] = False

    if old_addr == regs["_C_array_EndPtr"]:
        regs["read_C_ArrayAddr_reg"] = 0
        return "post_process"

    regs["read_C_ArrayAddr_reg"] = old_addr + 1
    return "cal"


def post_process(regs: Dict, io: IO) -> str:
    """CONFIRMED against Controller.scala:328-346. Branch SELECTION
    (is_wBuf_ptr_end / elif is_last_L_block_row / else) and which fields
    each branch touches were already right. Two VALUE mismatches found
    and fixed:

      1. Branch (1) (is_wBuf_ptr_end): the RTL's `when(is_wBuf_ptr_end)`
         body is `write_M_array_En_wire:=true.B` and NOTHING else --
         wbuf_ptr_reg is NOT reset here. This file previously reset it
         to 0 anyway. That's a real behavioral difference in general
         (wbuf_ptr_reg is otherwise an ever-incrementing counter, driven
         unconditionally by tick_background's accumulate path every time
         write_wBuf fires, and is_wBuf_ptr_end is an EXACT equality test
         against its max valid index -- relying on the counter to move
         past that value on its own, not on an explicit reset here).
      2. Branch (2) (is_last_L_block_row): wbuf_ptr_reg's reset is
         accWidth-dependent (Controller.scala:340,
         `Mux(accWidth_reg===ACC_32BIT,1.U,0.U)`), mirroring main_idle's
         own reset -- was a bare 0 here (only correct for 16-bit mode).
    """

    if regs["is_wBuf_ptr_end"]:
        # branch (1): write_M_array_En_wire only. wbuf_ptr_reg
        # deliberately left untouched (see docstring point 1).
        regs["write_M_array_En_wire"] = True

    elif regs["is_last_L_block_row"]:
        # branch (2): buffer not full, but no more rows remain in this
        # bit-slice's sweep -- flush the partial contents.
        regs["write_M_array_En_wire"] = True
        regs["wbuf_ptr_reg"] = 1 if regs["accWidth_reg"] == ACC_32BIT else 0
        regs["skip_read_M_array"] = False

    else:
        # branch (3): common inner case -- neither full nor last row;
        # carry the partial accumulation forward to the next row without
        # flushing. wbuf_ptr_reg NOT reset (only "wbuf_ptr_reg reset" is
        # spec'd for the other two branches, so absence here is read as
        # "carries over").
        regs["write_M_array_En_wire"] = False
        regs["skip_read_M_array"] = True

    return "pre_check"


def pre_check(regs: Dict, io: IO) -> str:
    """CONFIRMED against Controller.scala:347-383. The (T,T)/(T,F)/(F,*)
    next-state mapping this file already had -- main_idle / main_wait_L /
    main_wait_L -- is exactly right, including the outer-loop-is-bit-slice
    / inner-loop-is-L-block-row structure inferred from Scenarios A/B.
    Four register-write mismatches against the RTL were found and fixed
    here:

      1. `_L_vec_ptr_cur := 0` fires in BOTH (T,T) and (T,F) -- it sits
         OUTSIDE the bit_slice_done if/else in the RTL, at the top of the
         `when(is_last_L_block_row)` block. Previously only (T,F) reset
         it.
      2. `arrayCacheMode_reg := true` fires in (T,T) ("Restore cache
         state" in the RTL comment) -- previously missing entirely; see
         the matching `:= false` in main_idle ("Disable cache mode").
      3. (T,T) previously ALSO reset `_L_bitSlice_ID_ptr` and
         `is_first_slice` -- the RTL's (T,T) branch does not touch
         either register (only read_C_ArrayAddr_reg /
         read_M_Array_RowAdr_reg / write_M_Array_RowAdr_reg /
         arrayCacheMode_reg). Removed: both are harmless to leave stale
         here since main_idle unconditionally re-inits them on the next
         `exec`, but writing them here was not faithful to the RTL and
         would show up as a false diff against a cycle-by-cycle
         RTL/waveform cross-check.
      4. `read_C_ArrayAddr_reg := 0` ALSO fires in (T,F), not just
         (T,T) -- previously MISSING from (T,F). NOTE: verified this is
         functionally redundant, not a live bug -- `post_process` (and
         hence `pre_check`) is only ever reached via `cal`'s OWN
         termination branch (`cal`'s `read_C_ArrayAddr_reg==_C_array_EndPtr`
         case already sets it to 0 right there, Controller.scala:300-304,
         mirrored in this file's `cal()`), so the register is always
         already 0 by the time (T,F) runs. Fixed anyway for literal RTL
         fidelity (the hardware really does write it again here) and
         because relying on "some other state already did it" is a
         fragile invariant to leave undocumented.
    """

    bit_slice_done = regs["_L_bitSlice_ID_ptr"] == regs["_L_precision_reg"]

    if regs["is_last_L_block_row"]:
        # RTL: this reset sits OUTSIDE the bit_slice_done branch below --
        # it fires for BOTH (T,T) and (T,F).
        regs["_L_vec_ptr_cur"] = 0

        if bit_slice_done:
            # (T,T): fully done -> main_idle.
            regs["read_C_ArrayAddr_reg"] = 0
            regs["read_M_Array_RowAdr_reg"] = 0
            regs["write_M_Array_RowAdr_reg"] = 0
            # "Restore cache state" (Controller.scala:359); pairs with
            # main_idle's `:= false` ("Disable cache mode").
            regs["arrayCacheMode_reg"] = True
            return "main_idle"

        # (T,F): more bit-slices remain -> main_wait_L. is_first_slice
        # becomes False from here on (this is now a continuation, not a
        # fresh command).
        regs["_L_bitSlice_ID_ptr"] += 1
        regs["is_first_slice"] = False
        # C-/M-array pointers all reset to 0 for the next bit-slice's
        # sweep -- read_C_ArrayAddr_reg included (see docstring point 4;
        # redundant with cal()'s own reset in practice, kept for literal
        # RTL fidelity).
        regs["read_C_ArrayAddr_reg"] = 0
        regs["read_M_Array_RowAdr_reg"] = 0
        regs["write_M_Array_RowAdr_reg"] = 0
        return "main_wait_L"

    # (F,*): more L-block-rows remain in this bit-slice's sweep -- bare
    # transition, no register writes (RTL's `.otherwise` here is just
    # `mainState := main_wait_L`, nothing else).
    return "main_wait_L"


STATE_HANDLERS: Dict[str, Callable[[Dict, IO], str]] = {
    "main_idle": main_idle,
    "main_wait_L": main_wait_L,
    "pre_read_M_array": pre_read_M_array,
    "cal": cal,
    "post_process": post_process,
    "pre_check": pre_check,
}


# ---------------------------------------------------------------------------
# The interpreter -- Overall Architecture.
# ---------------------------------------------------------------------------

class MatModel:
    def __init__(self, io: Optional[IO] = None):
        self.io = io if io is not None else IO()
        self.regs: Dict = initial_regs()
        self.state: str = "main_idle"

    def step(self) -> str:
        # Step 3: process last cycle's RegNext-delayed effects using the
        # wires the state handler computed on the PREVIOUS call, before
        # this cycle's handler overwrites them.
        tick_background(self.regs, self.io)

        next_state = STATE_HANDLERS[self.state](self.regs, self.io)

        # Snapshot this cycle's freshly-computed wires so the NEXT
        # step()'s tick_background() sees them as "last cycle's" values.
        self.regs["_read_M_array_En_wire_prev"] = self.regs["read_M_array_En_wire"]
        self.regs["write_wBuf_wire_prev"] = self.regs["write_wBuf_wire"]

        self.state = next_state
        return self.state

    def run_until(self, target_state: str, max_steps: int = 10_000) -> List[str]:
        """Test/debug helper for integration scenarios: step until
        `state == target_state`, returning the full state-sequence trace
        (target_state included as the last entry)."""
        trace = [self.state]
        for _ in range(max_steps):
            if self.state == target_state:
                return trace
            self.step()
            trace.append(self.state)
        raise RuntimeError(
            f"did not reach {target_state!r} within {max_steps} steps; "
            f"trace={trace}"
        )
