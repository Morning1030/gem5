"""
Unit + integration tests for mat_fsm.py (Steps 4-5 of the plan).

Runnable both under pytest (`pytest test_mat_fsm.py -v`) and standalone
(`python3 test_mat_fsm.py`) with no dependencies beyond the stdlib, so it
doesn't need a pytest install to sanity-check the model.

Each test's docstring/comment cites which state + sub-case of the plan's
"Step 4" table it covers, so a failing test points straight back to the
requirement it's checking.
"""
import sys

from mat_fsm import (
    ACC_16BIT,
    ACC_32BIT,
    ArrayMode,
    IO,
    MatModel,
    SetUpIO,
    initial_regs,
    main_idle,
    main_wait_L,
    pre_read_M_array,
    cal,
    post_process,
    pre_check,
    tick_background,
)


# ---------------------------------------------------------------------------
# main_idle
# ---------------------------------------------------------------------------

def test_main_idle_exec_false_stays_put_and_writes_nothing():
    """main_idle (a): exec=false -> stays put, writes no registers."""
    regs = initial_regs()
    snapshot = dict(regs)  # shallow copy is enough: no state handler
                           # mutates nested lists when exec=false
    io = IO(set_up_io=SetUpIO(exec=False))

    next_state = main_idle(regs, io)

    assert next_state == "main_idle"
    assert regs == snapshot


def test_main_idle_exec_true_one_time_setup():
    """main_idle (b): exec=true -> verify the one-time setup fields."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(
        exec=True, nBuf=1, nCal=4, accWidth=16,
        _R_base_bit=3, _R_block_row=10, _L_block_row=5, _L_precision=7,
        _L_vec_fetch_addr=0x1000, signed_L=True, signed_R_last_exist=False,
    ))

    next_state = main_idle(regs, io)

    assert next_state == "main_wait_L"
    assert regs["_C_array_EndPtr"] == 10
    assert regs["read_C_ArrayAddr_reg"] == 0
    assert regs["accWidth_reg"] == 16
    assert regs["signed_L"] is True
    assert regs["signed_R_last_exist"] is False
    assert regs["_L_block_row"] == 5
    assert regs["_L_precision_reg"] == 7
    assert regs["_L_vec_addr"] == 0x1000
    assert regs["_L_vec_ptr_cur"] == 0
    assert regs["_L_bitSlice_ID_ptr"] == 0
    assert regs["is_first_slice"] is True
    assert regs["wbuf_ptr_reg"] == 0
    assert regs["write_M_Array_RowAdr_reg"] == 0
    assert regs["read_M_Array_RowAdr_reg"] == 0
    # CONFIRMED (Controller.scala:276, "Disable cache mode"): exec firing
    # switches the array SRAM ports off external/cache addressing for the
    # duration of this command; pre_check's (T,T) branch is the restore.
    assert regs["arrayCacheMode_reg"] is False


def test_main_idle_wbuf_ptr_reg_reset_is_accWidth_dependent():
    """main_idle (b): CONFIRMED (Controller.scala:264) -- wbuf_ptr_reg's
    reset is Mux(accWidth===ACC_32BIT, 1, 0), not a bare 0. The other
    test above already covers the 16-bit case (0); this covers 32-bit."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=1, nCal=1, accWidth=ACC_32BIT))
    main_idle(regs, io)
    assert regs["wbuf_ptr_reg"] == 1


def test_main_idle_array_mode_branch_nBuf_gt_2():
    """main_idle (b): CONFIRMED against Controller.scala:248-252. NOT a
    "first N lanes = MAC" pattern (the old placeholder) -- nBuf>2 forces
    lanes 0-2 to Mem regardless of working_array_num; lane 3 depends only
    on working_array_num (here 3+0=3 -> IdleMac, since 3<=3)."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=3, nCal=0))
    main_idle(regs, io)
    assert regs["arrayMode_reg"] == [
        ArrayMode.MEM, ArrayMode.MEM, ArrayMode.MEM, ArrayMode.IDLE_MAC
    ]


def test_main_idle_array_mode_branch_working_array_num_le_2():
    """main_idle (b): nBuf<=2, working_array_num=nBuf+nCal=2 (<=2)."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=0, nCal=2))
    main_idle(regs, io)
    assert regs["arrayMode_reg"] == [
        ArrayMode.MEM, ArrayMode.MAC, ArrayMode.IDLE_MAC, ArrayMode.IDLE_MAC
    ]


def test_main_idle_array_mode_branch_working_array_num_le_3():
    """main_idle (b): nBuf<=2, working_array_num=nBuf+nCal=3 (<=3, not <=2)."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=0, nCal=3))
    main_idle(regs, io)
    assert regs["arrayMode_reg"] == [
        ArrayMode.MEM, ArrayMode.MAC, ArrayMode.MAC, ArrayMode.IDLE_MAC
    ]


def test_main_idle_array_mode_branch_else():
    """main_idle (b): nBuf<=2, working_array_num=nBuf+nCal=4 (>3)."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=0, nCal=4))
    main_idle(regs, io)
    assert regs["arrayMode_reg"] == [
        ArrayMode.MEM, ArrayMode.MAC, ArrayMode.MAC, ArrayMode.MAC
    ]


def test_main_idle_array_mode_working_array_num_zero_case_defaults_to_4():
    """main_idle (b): CONFIRMED (Controller.scala:248) -- the (nBuf+nCal)
    special case: working_array_num defaults to 4 when nBuf+nCal==0,
    rather than 0. Distinct code path from branch_else above (which
    reaches working_array_num=4 via 0+4, not via the ==0 special case)."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=0, nCal=0))
    main_idle(regs, io)
    assert regs["arrayMode_reg"] == [
        ArrayMode.MEM, ArrayMode.MAC, ArrayMode.MAC, ArrayMode.MAC
    ]


def test_main_idle_bitID_R_and_lastBitR_bidID():
    """main_idle (c): CONFIRMED against Controller.scala:238-244,258.
    bitID_R: i<nBuf -> 0 (Mem-mode buffer lane, RTL just zeros it -- no
    "active id" or "inactive sentinel" concept exists for these lanes);
    i>=nBuf -> _R_base_bit+(i-nBuf) (this lane's R bit-slice position,
    i==nBuf being the FIRST Mac lane). lastBitR_bidID is an INDEPENDENT
    formula (_R_base_bit+nCal-1) -- previously (wrongly) latched from
    bitID_R[nBuf] instead."""
    regs = initial_regs()
    io = IO(set_up_io=SetUpIO(exec=True, nBuf=2, nCal=2, _R_base_bit=10))
    main_idle(regs, io)

    # i < nBuf (0, 1): Mem-mode buffer lanes -- zeroed, not "active".
    assert regs["bitID_R"][0] == 0
    assert regs["bitID_R"][1] == 0
    # i >= nBuf (2, 3): Mac lanes, consecutive R bit positions.
    assert regs["bitID_R"][2] == 10
    assert regs["bitID_R"][3] == 11
    # Independent formula: _R_base_bit + nCal - 1 = 10 + 2 - 1.
    assert regs["lastBitR_bidID"] == 11


# ---------------------------------------------------------------------------
# main_wait_L
# ---------------------------------------------------------------------------

def test_main_wait_L_stub_not_done_stays_put():
    regs = initial_regs()
    regs["_L_vec_ptr_cur"] = 0
    io = IO(l_fetch_done=lambda: False)
    next_state = main_wait_L(regs, io)
    assert next_state == "main_wait_L"
    assert regs["_L_vec_ptr_cur"] == 0  # not incremented while waiting


def test_main_wait_L_increments_ptr_and_routes_skip_true_to_cal():
    regs = initial_regs()
    regs["_L_vec_ptr_cur"] = 3
    regs["_L_vec_addr"] = 100
    regs["skip_read_M_array"] = True
    io = IO(l_fetch_done=lambda: True)
    next_state = main_wait_L(regs, io)
    assert regs["_L_vec_ptr_cur"] == 4
    # CONFIRMED (Controller.scala:394): the request payload counter
    # advances once per row requested -- previously not tracked at all.
    assert regs["_L_vec_addr"] == 101
    assert next_state == "cal"
    # CONFIRMED (Controller.scala:408): cleared unconditionally after use
    # -- previously MISSING. Regression-relevant: without this clear, a
    # stale True would keep routing every SUBSEQUENT main_wait_L visit
    # straight to cal, even ones nothing re-armed it for.
    assert regs["skip_read_M_array"] is False


def test_main_wait_L_routes_skip_false_to_pre_read_M_array():
    regs = initial_regs()
    regs["skip_read_M_array"] = False
    io = IO(l_fetch_done=lambda: True)
    next_state = main_wait_L(regs, io)
    assert next_state == "pre_read_M_array"
    assert regs["skip_read_M_array"] is False


# ---------------------------------------------------------------------------
# pre_read_M_array
# ---------------------------------------------------------------------------

def test_pre_read_M_array_first_slice_no_read():
    """pre_read_M_array (a): is_first_slice=True -> no read, straight to cal."""
    regs = initial_regs()
    regs["is_first_slice"] = True
    regs["read_M_Array_RowAdr_reg"] = 5
    next_state = pre_read_M_array(regs, IO())
    assert regs["read_M_array_En_wire"] is False
    assert regs["read_M_Array_RowAdr_reg"] == 5  # unchanged
    assert next_state == "cal"


def test_pre_read_M_array_not_first_slice_issues_read():
    """pre_read_M_array (b): is_first_slice=False -> read_M_array_En_wire
    true, row-addr +1; next tick_background() should show dout_valid=True."""
    regs = initial_regs()
    regs["is_first_slice"] = False
    regs["read_M_Array_RowAdr_reg"] = 5
    next_state = pre_read_M_array(regs, IO())
    assert regs["read_M_array_En_wire"] is True
    assert regs["read_M_Array_RowAdr_reg"] == 6
    assert next_state == "cal"

    # Simulate step()'s prev-snapshot + the following tick_background().
    regs["_read_M_array_En_wire_prev"] = regs["read_M_array_En_wire"]
    tick_background(regs, IO())
    assert regs["_M_array_dout_valid"] is True


# ---------------------------------------------------------------------------
# cal -- the module this file was specifically requested for.
# ---------------------------------------------------------------------------

def _cal_regs(end_ptr, array_mode=None):
    regs = initial_regs()
    regs["_C_array_EndPtr"] = end_ptr
    regs["read_C_ArrayAddr_reg"] = 0
    regs["arrayMode_reg"] = array_mode or [
        ArrayMode.MAC, ArrayMode.MEM, ArrayMode.MAC, ArrayMode.MEM
    ]
    regs["is_wBuf_ptr_end"] = False
    return regs


def test_cal_normal_loop_write_wBuf_and_read_C_ArrayEn():
    """cal (a): normal loop -- write_wBuf_wire true every cycle,
    read_C_ArrayEn(i) true only where arrayMode==Mac."""
    regs = _cal_regs(end_ptr=4)
    io = IO()

    seen_addrs = []
    for _ in range(4):
        seen_addrs.append(regs["read_C_ArrayAddr_reg"])
        next_state = cal(regs, io)
        assert regs["write_wBuf_wire"] is True
        assert regs["read_C_ArrayEn"] == [True, False, True, False]
        assert next_state == "cal"

    assert seen_addrs == [0, 1, 2, 3]
    assert regs["read_C_ArrayAddr_reg"] == 4


def test_cal_terminating_cycle_still_accumulates():
    """cal (c), CONFIRMED against Controller.scala:295-300: read_C_ArrayEn
    and write_wBuf_wire are asserted UNCONDITIONALLY, before (and
    regardless of) the termination check -- _C_array_EndPtr is an
    INCLUSIVE bound, so the terminating cycle (old address == EndPtr)
    still does its accumulate/read_C_ArrayEn work before handing off.
    Previously an early-return on termination skipped both entirely on
    that last cycle."""
    regs = _cal_regs(end_ptr=2)
    io = IO()

    cal(regs, io)  # addr 0 -> 1
    cal(regs, io)  # addr 1 -> 2
    assert regs["read_C_ArrayAddr_reg"] == 2

    # These are wires (freshly computed every real hardware cycle); since
    # `regs` is a plain dict that persists across calls in this model,
    # clear them to a distinguishing value first so the assertions below
    # actually prove the terminating call sets them, rather than passing
    # on stale leftovers from the previous (non-terminating) call.
    regs["write_wBuf_wire"] = False
    regs["read_C_ArrayEn"] = [False] * 4

    next_state = cal(regs, io)  # addr == EndPtr -> terminates...
    assert next_state == "post_process"
    assert regs["read_C_ArrayAddr_reg"] == 0
    # ...but still accumulates for address 2 (EndPtr itself) on the way.
    assert regs["write_wBuf_wire"] is True
    assert regs["read_C_ArrayEn"] == [True, False, True, False]


def test_cal_mid_loop_flush_suppressed_on_first_cycle_of_sweep():
    """cal (b), CONFIRMED against Controller.scala:310: the mid-loop-flush
    guard is `is_wBuf_ptr_end && read_C_ArrayAddr_reg=/=0.U` -- using the
    OLD (pre-cycle) address, so it's suppressed on cal's own very first
    cycle of a fresh sweep (old address 0) even if is_wBuf_ptr_end is
    already (spuriously) true. Previously the `!= 0` half was missing
    entirely, so a stale is_wBuf_ptr_end=True would incorrectly fire the
    flush on cycle 0."""
    regs = _cal_regs(end_ptr=10)
    regs["is_wBuf_ptr_end"] = True  # already true entering cycle 0
    io = IO()

    cal(regs, io)  # old address was 0 -> flush must be suppressed
    assert regs["write_M_array_En_wire"] is False
    assert regs["read_M_array_En_wire"] is False

    # Second call: old address is now 1 (!= 0) -> flush fires normally.
    cal(regs, io)
    assert regs["write_M_array_En_wire"] is True


def test_cal_mid_loop_flush_not_first_slice():
    """cal (b): is_wBuf_ptr_end becomes true partway (not cycle 0) ->
    write_M_array_En_wire fires, and read_M_array_En_wire fires (with
    row-addr +1) when is_first_slice=False."""
    regs = _cal_regs(end_ptr=10)
    regs["is_first_slice"] = False
    regs["read_M_Array_RowAdr_reg"] = 0
    io = IO()

    cal(regs, io)  # cycle 0: old address 0 -> flush suppressed regardless
    assert regs["write_M_array_En_wire"] is False

    regs["is_wBuf_ptr_end"] = True  # flush fires on cycle 1, not cycle 0
    cal(regs, io)
    assert regs["write_M_array_En_wire"] is True
    assert regs["read_M_array_En_wire"] is True
    assert regs["read_M_Array_RowAdr_reg"] == 1


def test_cal_mid_loop_flush_first_slice():
    """cal (b), is_first_slice=True branch: flush fires but no read is
    issued (mirrors pre_read_M_array's own is_first_slice gating). Uses
    old address 1 (not 0) so the != 0 guard doesn't suppress the flush."""
    regs = _cal_regs(end_ptr=10)
    regs["read_C_ArrayAddr_reg"] = 1
    regs["is_first_slice"] = True
    regs["is_wBuf_ptr_end"] = True
    io = IO()

    cal(regs, io)
    assert regs["write_M_array_En_wire"] is True
    assert regs["read_M_array_En_wire"] is False


def test_cal_termination_resets_addr_and_transitions():
    """cal (c): the cycle where read_C_ArrayAddr_reg == EndPtr transitions
    to post_process and resets read_C_ArrayAddr_reg to 0."""
    regs = _cal_regs(end_ptr=2)
    io = IO()

    assert cal(regs, io) == "cal"          # addr 0 -> 1
    assert cal(regs, io) == "cal"          # addr 1 -> 2
    assert regs["read_C_ArrayAddr_reg"] == 2

    next_state = cal(regs, io)             # addr == EndPtr -> terminate
    assert next_state == "post_process"
    assert regs["read_C_ArrayAddr_reg"] == 0


# ---------------------------------------------------------------------------
# post_process -- three mutually exclusive branches
# ---------------------------------------------------------------------------

def test_post_process_branch1_wBuf_ended():
    """branch (1): CONFIRMED against Controller.scala:331-334 -- the RTL's
    when(is_wBuf_ptr_end) body is write_M_array_En_wire:=true.B and
    NOTHING else. wbuf_ptr_reg is deliberately left UNTOUCHED here
    (previously this file incorrectly reset it to 0) -- it's otherwise an
    ever-incrementing counter (tick_background's accumulate path, driven
    unconditionally whenever write_wBuf fires) and is_wBuf_ptr_end is an
    exact-equality test against its max valid index, relying on the
    counter moving past that value on its own rather than an explicit
    reset here."""
    regs = initial_regs()
    regs["is_wBuf_ptr_end"] = True
    regs["is_last_L_block_row"] = False  # irrelevant to branch (1)
    regs["skip_read_M_array"] = "SENTINEL"  # branch (1) must not touch this
    regs["wbuf_ptr_reg"] = 3  # distinguishing value -- must survive untouched

    next_state = post_process(regs, IO())

    assert next_state == "pre_check"
    assert regs["write_M_array_En_wire"] is True
    assert regs["wbuf_ptr_reg"] == 3, "wbuf_ptr_reg must NOT be reset in branch (1)"
    assert regs["skip_read_M_array"] == "SENTINEL"  # untouched


def test_post_process_branch2_last_row_not_full_16bit():
    """branch (2), CONFIRMED against Controller.scala:337-341: the reset
    value is Mux(accWidth_reg===ACC_32BIT, 1, 0) -- accWidth-dependent,
    not a bare 0. This is the 16-bit case (0); see the _32bit variant
    below for the case this file previously got wrong."""
    regs = initial_regs()
    regs["is_wBuf_ptr_end"] = False
    regs["is_last_L_block_row"] = True
    regs["accWidth_reg"] = ACC_16BIT

    next_state = post_process(regs, IO())

    assert next_state == "pre_check"
    assert regs["write_M_array_En_wire"] is True
    assert regs["wbuf_ptr_reg"] == 0
    assert regs["skip_read_M_array"] is False


def test_post_process_branch2_last_row_not_full_32bit():
    """branch (2), 32-bit case: wbuf_ptr_reg resets to 1, not 0 --
    previously this file used a bare 0 regardless of accWidth, which was
    only correct for the 16-bit case above."""
    regs = initial_regs()
    regs["is_wBuf_ptr_end"] = False
    regs["is_last_L_block_row"] = True
    regs["accWidth_reg"] = ACC_32BIT

    next_state = post_process(regs, IO())

    assert next_state == "pre_check"
    assert regs["write_M_array_En_wire"] is True
    assert regs["wbuf_ptr_reg"] == 1
    assert regs["skip_read_M_array"] is False


def test_post_process_branch3_common_inner_case():
    regs = initial_regs()
    regs["is_wBuf_ptr_end"] = False
    regs["is_last_L_block_row"] = False
    regs["wbuf_ptr_reg"] = 2  # must NOT be reset in this branch (in-range:
                              # wbuf_ptr_reg is a 2-bit reg, max value
                              # WBUF_NUM_SLOTS-1=3)

    next_state = post_process(regs, IO())

    assert next_state == "pre_check"
    assert regs["write_M_array_En_wire"] is False
    assert regs["wbuf_ptr_reg"] == 2
    assert regs["skip_read_M_array"] is True


# ---------------------------------------------------------------------------
# pre_check -- (T,T) / (T,F) / (F,*)
# ---------------------------------------------------------------------------

def test_pre_check_TT_fully_done_to_main_idle():
    """(T,T): CONFIRMED against Controller.scala:347-359. Also asserts
    what pre_check does NOT do here (contrast with the old, incorrect
    behavior this test used to check): _L_bitSlice_ID_ptr and
    is_first_slice are left untouched by (T,T) in the RTL -- they're
    reset by main_idle on the next exec instead, not here."""
    regs = initial_regs()
    regs["is_last_L_block_row"] = True
    regs["_L_bitSlice_ID_ptr"] = 7
    regs["_L_precision_reg"] = 7  # bit_slice_done
    regs["read_C_ArrayAddr_reg"] = 3
    regs["read_M_Array_RowAdr_reg"] = 3
    regs["write_M_Array_RowAdr_reg"] = 3
    regs["_L_vec_ptr_cur"] = 5
    regs["arrayCacheMode_reg"] = False  # set False by main_idle at exec
    # Distinguishing value: by the time (T,T) is reached mid-command,
    # is_first_slice has normally already flipped to False (via an
    # earlier (T,F) pass) -- set it explicitly so "pre_check leaves it
    # alone" is actually exercised, not true only by coincidence.
    regs["is_first_slice"] = False

    next_state = pre_check(regs, IO())

    assert next_state == "main_idle"
    assert regs["read_C_ArrayAddr_reg"] == 0
    assert regs["read_M_Array_RowAdr_reg"] == 0
    assert regs["write_M_Array_RowAdr_reg"] == 0
    # _L_vec_ptr_cur := 0 fires for (T,T) too (RTL: outside the
    # bit_slice_done branch, at the top of when(is_last_L_block_row)).
    assert regs["_L_vec_ptr_cur"] == 0
    # "Restore cache state" (Controller.scala:359).
    assert regs["arrayCacheMode_reg"] is True
    # NOT touched by (T,T) in the RTL -- left exactly as they were.
    assert regs["_L_bitSlice_ID_ptr"] == 7
    assert regs["is_first_slice"] is False


def test_pre_check_TF_more_bit_slices_to_main_wait_L():
    regs = initial_regs()
    regs["is_last_L_block_row"] = True
    regs["_L_bitSlice_ID_ptr"] = 0
    regs["_L_precision_reg"] = 7  # not done
    regs["is_first_slice"] = True
    regs["_L_vec_ptr_cur"] = 5
    regs["read_C_ArrayAddr_reg"] = 3
    regs["read_M_Array_RowAdr_reg"] = 3
    regs["write_M_Array_RowAdr_reg"] = 3

    next_state = pre_check(regs, IO())

    assert next_state == "main_wait_L"
    assert regs["_L_bitSlice_ID_ptr"] == 1
    assert regs["_L_vec_ptr_cur"] == 0
    assert regs["is_first_slice"] is False
    assert regs["read_M_Array_RowAdr_reg"] == 0
    assert regs["write_M_Array_RowAdr_reg"] == 0


def test_pre_check_TF_more_bit_slices_resets_read_C_ArrayAddr():
    """Unit-level fidelity check: RTL resets read_C_ArrayAddr_reg in
    (T,F), not just (T,T) -- previously missing here. In isolation (this
    test calls pre_check() directly, bypassing cal()) that reset matters;
    at the full-FSM level it's actually redundant, because pre_check is
    only ever reached via cal()'s OWN termination branch, which already
    zeroed this register (see test_scenario_D_... and the docstring note
    in pre_check() point 4 for the full reasoning) -- so this test exists
    for literal register-write fidelity against the RTL, not because a
    real bit-slice-skipping bug was observed at the integration level."""
    regs = initial_regs()
    regs["is_last_L_block_row"] = True
    regs["_L_bitSlice_ID_ptr"] = 0
    regs["_L_precision_reg"] = 7  # not done -> (T,F)
    regs["_C_array_EndPtr"] = 9
    regs["read_C_ArrayAddr_reg"] = 9  # deliberately nonzero, to isolate this write

    next_state = pre_check(regs, IO())

    assert next_state == "main_wait_L"
    assert regs["read_C_ArrayAddr_reg"] == 0, (
        "pre_check's (T,F) branch must write read_C_ArrayAddr_reg := 0, "
        "matching the RTL, even though cal()'s own termination logic "
        "already guarantees this in the full FSM"
    )


def test_pre_check_F_star_more_rows_to_main_wait_L():
    regs = initial_regs()
    regs["is_last_L_block_row"] = False
    for bit_slice_done in (True, False):
        regs["_L_bitSlice_ID_ptr"] = 7 if bit_slice_done else 0
        regs["_L_precision_reg"] = 7
        assert pre_check(regs, IO()) == "main_wait_L"


# ---------------------------------------------------------------------------
# Integration scenarios (Step 5)
# ---------------------------------------------------------------------------

def test_scenario_A_minimal_single_pass():
    """Scenario A: single bit-slice, _L_block_row=1 -- full path
    main_idle -> wait_L -> pre_read_M -> cal(N) -> post_process ->
    pre_check -> main_idle."""
    model = MatModel(io=IO(set_up_io=SetUpIO(
        exec=True, nBuf=1, nCal=4, accWidth=16,
        _R_block_row=3, _L_block_row=1, _L_precision=0,
    )))
    trace = [model.state]
    for _ in range(60):
        model.step()
        trace.append(model.state)
        if model.state == "main_idle" and len(trace) > 1:
            break

    print("Scenario A trace:", trace)
    assert trace[0] == "main_idle"
    assert "main_wait_L" in trace
    assert "pre_read_M_array" in trace
    assert "cal" in trace
    assert "post_process" in trace
    assert "pre_check" in trace
    assert trace[-1] == "main_idle"
    # exec was consumed by the first main_idle call; model should now be
    # idling (exec still True in this io stub, but is_first_slice/etc.
    # were reset -- re-running main_idle would just redo setup, which is
    # fine/idempotent, not asserted further here).


def test_scenario_B_multiple_bit_slices_loops_to_wait_L():
    """Scenario B: multiple bit-slices -- pre_check's "more bit-slices
    remain" branch loops back to main_wait_L instead of main_idle, and
    is_first_slice flips True -> False across that transition."""
    model = MatModel(io=IO(set_up_io=SetUpIO(
        exec=True, nBuf=1, nCal=2, accWidth=32,
        _R_block_row=2, _L_block_row=1, _L_precision=1,  # 2 bit-slices
    )))
    model.step()  # consume exec -> main_wait_L
    assert model.regs["is_first_slice"] is True

    trace = ["main_wait_L"]
    saw_first_slice_flip = False
    for _ in range(60):
        model.step()
        trace.append(model.state)
        if trace[-1] == "main_wait_L" and model.regs["is_first_slice"] is False:
            saw_first_slice_flip = True
            break

    print("Scenario B trace:", trace)
    assert saw_first_slice_flip, "expected is_first_slice True->False on the loop-back to main_wait_L"


def test_scenario_C_wBuf_fills_exactly_mid_cal():
    """Scenario C: wBuf fills mid-cal, wraps (WBUF_NUM_SLOTS=4, a fixed
    hardware width -- unrelated to nCal=2 here), and post_process is
    entered AFTER it has already wrapped back past the flush point.
    CONFIRMED (traced with a debug script against the fixed model): with
    _R_block_row=5, wbuf_ptr_reg walks 0,0,0,0,1,2,3,[wrap]0,1 across the
    cal sweep -- is_wBuf_ptr_end pulses True for exactly the one cycle
    where wbuf_ptr_reg==3 (firing cal's mid-loop flush), then wraps back
    to 0 before the sweep even ends. So by the time post_process runs,
    is_wBuf_ptr_end is back to False and it's actually BRANCH (2)
    (is_last_L_block_row, since _L_block_row=1 is a single-row block)
    that fires and resets wbuf_ptr_reg -- not branch (1), which this
    test's docstring used to (incorrectly) imply. This is exactly the
    scenario that caught the original is_wBuf_ptr_end/wbuf_ptr_reg
    fixed-width bug: before that fix, wbuf_ptr_reg grew unbounded and
    is_wBuf_ptr_end latched True forever, so this test was (wrongly)
    exercising branch (1) throughout."""
    model = MatModel(io=IO(set_up_io=SetUpIO(
        exec=True, nBuf=1, nCal=2, accWidth=16,
        _R_block_row=5, _L_block_row=1, _L_precision=0,
    )))
    model.step()  # main_idle -> main_wait_L
    model.step()  # main_wait_L -> pre_read_M_array (skip_read_M_array=False default)
    model.step()  # pre_read_M_array -> cal (is_first_slice=True path)

    trace = ["cal"]
    hit_flush = False
    while model.state != "post_process":
        model.step()
        trace.append(model.state)
        if model.regs.get("write_M_array_En_wire"):
            hit_flush = True

    print("Scenario C trace:", trace)
    assert hit_flush, "expected the mid-loop flush to fire before post_process (nCal=2 < _R_block_row=5)"
    # Confirms the wrap-back-past-the-flush-point behavior above, not
    # just an assumption about it.
    assert model.regs["is_wBuf_ptr_end"] is False, (
        "is_wBuf_ptr_end should have wrapped back to False by the time "
        "post_process is entered -- branch (2) should fire, not (1)"
    )

    model.step()  # run post_process
    assert model.regs["wbuf_ptr_reg"] == 0, "wbuf_ptr_reg must be RESET, not merely incremented"


def test_scenario_D_second_bit_slice_cal_sweep_not_skipped():
    """Scenario D: full-FSM check that bit-slice 2's cal() sweep runs its
    full length, not just bit-slice 1's. With _R_block_row=3 (EndPtr=3),
    each bit-slice's cal sweep should visit the "cal" state 4 times (addr
    0,1,2,3) before handing off to post_process.

    NOTE: this does NOT regression-test pre_check's (T,F)
    read_C_ArrayAddr_reg reset specifically -- verified (by temporarily
    reverting that line) that this scenario passes either way, because
    cal()'s OWN termination branch already zeroes read_C_ArrayAddr_reg
    before pre_check ever runs (post_process/pre_check are only reached
    via that branch). See test_pre_check_TF_more_bit_slices_resets_read_C_ArrayAddr
    for the unit-level check of that specific write.
    """
    model = MatModel(io=IO(set_up_io=SetUpIO(
        exec=True, nBuf=1, nCal=2, accWidth=32,
        _R_block_row=3, _L_block_row=1, _L_precision=1,  # 2 bit-slices
    )))

    # Run to the (T,F) loop-back into bit-slice 2 (same detection as
    # Scenario B: is_first_slice flips True -> False at main_wait_L).
    trace = [model.state]
    saw_first_slice_flip = False
    for _ in range(60):
        model.step()
        trace.append(model.state)
        if model.state == "main_wait_L" and model.regs["is_first_slice"] is False:
            saw_first_slice_flip = True
            break
    assert saw_first_slice_flip, f"never reached bit-slice 2; trace={trace}"
    assert model.regs["read_C_ArrayAddr_reg"] == 0, (
        "read_C_ArrayAddr_reg must already be 0 entering bit-slice 2's "
        f"main_wait_L (got {model.regs['read_C_ArrayAddr_reg']}), or its "
        "cal sweep will terminate on the first call"
    )

    # Run bit-slice 2's cal sweep to completion, counting "cal" visits.
    cal_visits = 0
    for _ in range(60):
        model.step()
        if model.state == "cal":
            cal_visits += 1
        if model.state == "post_process":
            break

    assert cal_visits == 4, (
        f"expected bit-slice 2's cal sweep to run all 4 iterations "
        f"(_R_block_row=3 -> EndPtr=3 -> addr 0,1,2,3), got {cal_visits}"
    )


# ---------------------------------------------------------------------------
# Standalone runner (no pytest dependency)
# ---------------------------------------------------------------------------

def _all_tests():
    g = globals()
    return [g[name] for name in sorted(g) if name.startswith("test_")]


if __name__ == "__main__":
    failures = []
    for fn in _all_tests():
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failures.append(fn.__name__)
            print(f"FAIL  {fn.__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            failures.append(fn.__name__)
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")

    print(f"\n{len(_all_tests()) - len(failures)}/{len(_all_tests())} passed")
    if failures:
        print("Failed:", ", ".join(failures))
        sys.exit(1)
