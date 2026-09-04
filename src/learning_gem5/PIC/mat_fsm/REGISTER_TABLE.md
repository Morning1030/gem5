# mat_fsm register/wire/input inventory (Step 1)

Source of truth for [mat_fsm.py](mat_fsm.py). Every `ASSUMPTION` here has a
matching comment at its point of use in the code — this table exists to
make them easy to review as one pass, and to record *why* each guess was
made, not just what it is. If you have the real spec, the fastest way to
use this doc is: skim the "Assumption" column, fix the wrong ones (in
both this table and the matching code comment), leave the rest.

**Status: all six `mainState` handlers now traced against
`Controller.scala` and CONFIRMED (see item 7 below for the second,
larger pass — `main_idle`, `main_wait_L`, `pre_read_M_array`, `cal`,
`post_process` — following item 6's `pre_check` pass). Every fix below
was verified by reverting it in isolation and confirming the regression
suite catches it (details in item 7's sub-points), the same discipline
used for `pre_check`'s point 4 in item 6.**

## Registers (persisted across steps)

| name | type | init | written by | read by |
|---|---|---|---|---|
| `_C_array_EndPtr` | int | 0 | `main_idle` (`= _R_block_row`) | `cal` (loop bound, inclusive) |
| `read_C_ArrayAddr_reg` | int | 0 | `main_idle` (reset), `cal` (old+1, or reset to 0 on termination), `pre_check` (reset in **both** (T,T) and (T,F)) | `cal` (loop cond — CONFIRMED: reads the OLD/pre-cycle value even after `cal`'s own `:=`, a Chisel Reg-read subtlety; see item 7) |
| `bitID_R[4]` | int[4] | 0s | `main_idle` — **CONFIRMED** (Controller.scala:238-244): `i<nBuf → 0` (Mem-mode buffer lane, RTL just zeros it), `i>=nBuf → _R_base_bit+(i-nBuf)` (Mac lane's R bit position). No "inactive sentinel" concept exists in the RTL; previous model had this inverted (see item 7) | — (not consumed by any control decision in this file; datapath-side) |
| `lastBitR_bidID` | int | 0 | `main_idle` — **CONFIRMED** (Controller.scala:258): **independent** formula `_R_base_bit + nCal - 1`, NOT derived from `bitID_R[nBuf]` (previous model wrongly latched it from there — see item 7) | — (datapath-side) |
| `arrayMode_reg[4]` | enum[4] | **Mem** (CONFIRMED: `RegInit(VecInit(Seq.fill(4)(0.U(...))))`, and `PICMode_Mem="b00"` — was `IDLE`, an enum value that no longer exists) | `main_idle` — **CONFIRMED** (Controller.scala:249-252): 4 independently-formulated lanes, NOT "first N lanes = MAC" (see item 7 and `compute_array_mode()`) | `cal` (gates `read_C_ArrayEn(i)` on exact-`Mac`, not `IdleMac`) |
| `accWidth_reg` | int | **ACC_32BIT=32** (CONFIRMED: `RegInit(ACC_32BIT)`, Controller.scala:137 — was `0`, which matched neither `ACC_32BIT` nor `ACC_16BIT` and silently behaved like 16-bit mode) | `main_idle` (`= accWidth`) | `tick_background` (`buf_ptr_inc`), `post_process` branch ② (reset value) — CONFIRMED both are accWidth-dependent, see item 7 |
| `signed_L`, `signed_R_last_exist` | bool | False | `main_idle` | — (datapath-side) |
| `wBuf[]` | list | `[None]*WBUF_NUM_SLOTS` (CONFIRMED: was `[None]*4096`, an arbitrary placeholder — see item 7) | `tick_background` (accumulate) | — (values inert; only `wbuf_ptr_reg` is control-relevant) |
| `wbuf_ptr_reg` | int, **fixed-width, wraps mod WBUF_NUM_SLOTS** | 0 | `main_idle` (accWidth-dependent reset, CONFIRMED), `tick_background` (`+= buf_ptr_inc`, wrapping — CONFIRMED, was unbounded), `post_process` (branch ② only, accWidth-dependent — branch ① does **NOT** touch it, CONFIRMED; previous model wrongly reset it to 0 in both ①② — see item 7) | `tick_background` (`is_wBuf_ptr_end`) |
| `_L_vec_ptr_cur` | int | 0 | `main_idle` (reset), `main_wait_L` (+1), `pre_check` (reset in **both** (T,T) and (T,F)) | `tick_background` (`is_last_L_block_row`) |
| `_L_block_row` | int | 0 | `main_idle` (`= _L_block_row` input) | `tick_background` (`is_last_L_block_row`) |
| `_L_precision_reg` | int | 0 | `main_idle` (`= _L_precision` input) | `pre_check` (`bit_slice_done`) |
| `_L_bitSlice_ID_ptr` | int | 0 | `main_idle` (reset), `pre_check` (+1 in (T,F) **only**) | `pre_check` (`bit_slice_done`) |
| `_L_vec_addr` | int | 0 | `main_idle` (`= _L_vec_fetch_addr` input), `main_wait_L` (+1 per row requested — CONFIRMED, previously not tracked at all, see item 7) | — (datapath-side / AutoLoadL) |
| `is_first_slice` | bool | True | `main_idle` (True), `pre_check` (False in (T,F) **only**) | `pre_read_M_array`, `cal` (mid-loop flush read gating) |
| `arrayCacheMode_reg` | bool | **True** (`RegInit(true.B)`, Controller.scala:135) | `main_idle` (`:= False`, "Disable cache mode", :276), `pre_check` (T,T) (`:= True`, "Restore cache state", :359) | — (gates the array SRAM port's external-cache-vs-Controller-internal mux in `Mat.scala`) |
| `skip_read_M_array` | bool | False | `main_idle` — **CONFIRMED not written here at all** (searched Controller.scala:231-280, absent; previous model explicitly reset it False, removed — provably a no-op given the FSM's reachable paths, see `pre_check`'s docstring). `post_process` (branches ②③). `main_wait_L` (**CONFIRMED `:= False` after use, Controller.scala:408 — previously MISSING entirely**, a real bug: a stale `True` would keep routing every subsequent visit straight to `cal`; see item 7) | `main_wait_L` (routes to `cal` vs `pre_read_M_array`) |
| `write_M_Array_RowAdr_reg` | int | 0 | `main_idle` (reset), `pre_check` (reset in (T,T)/(T,F)) | — (datapath-side) |
| `read_M_Array_RowAdr_reg` | int | 0 | `main_idle` (reset), `pre_read_M_array` (+1), `cal` (+1 mid-loop flush), `pre_check` (reset) | — (datapath-side) |
| `rBuf` | any | None | `tick_background` (latch on `_M_array_dout_valid`) | — (datapath-side) |
| `write_wBuf` | bool | False | `tick_background` (RegNext of `write_wBuf_wire`) | `tick_background` (gates accumulate) |

## Wires (combinational, recomputed every `step()`)

| name | written by | read by | formula source |
|---|---|---|---|
| `write_wBuf_wire` | `cal` — **CONFIRMED unconditional**, including the terminating cycle (Controller.scala:295-300 execute BEFORE, and regardless of, the termination check — see item 7) | `tick_background` (next cycle, via `_prev`) | true every `cal` cycle, terminating cycle included |
| `read_C_ArrayEn[4]` | `cal` — same unconditional-including-terminating-cycle fix as `write_wBuf_wire` above | — (datapath-side) | true iff `arrayMode_reg[i] == Mac` |
| `read_M_array_En_wire` | `pre_read_M_array`, `cal` (mid-loop flush) | `tick_background` (next cycle → `_M_array_dout_valid`) | CONFIRMED (both call sites) |
| `write_M_array_En_wire` | `cal` (mid-loop flush, **CONFIRMED also gated on `read_C_ArrayAddr_reg` OLD-value `!= 0`, Controller.scala:310 — previously missing, see item 7**), `post_process` (branches ①②, CONFIRMED exact values — see item 7) | — | CONFIRMED |
| `_M_array_dout_valid` | `tick_background` | — | RegNext of `read_M_array_En_wire` |
| `is_wBuf_ptr_end` | `tick_background` | `cal`, `post_process` | **CONFIRMED** (Controller.scala:152): `wbuf_ptr_reg === WBUF_NUM_SLOTS-1` — a fixed hardware-width constant, **unrelated to `nCal`**. Previous formula (`wbuf_ptr_reg >= nCal-1`) combined with `wbuf_ptr_reg` growing unbounded meant this latched `True` forever once crossed — see item 7 |
| `is_last_L_block_row` | `tick_background` | `post_process`, `pre_check` | **CONFIRMED** (Controller.scala:151): `_L_vec_ptr_cur == _L_block_row` (exact equality) |
| `left_shift_bias(i)`, `signed(i)`, `sum_of_mac` | — | — | **OUT OF SCOPE by instruction** — datapath black box, see `IO.sum_of_mac` |

## External inputs (`io.set_up_io.*` and environment)

| name | source |
|---|---|
| `exec`, `nBuf`, `nCal`, `accWidth`, `_R_base_bit`, `_R_block_row`, `_L_block_row`, `_L_precision`, `_L_vec_fetch_addr`, `signed_L`, `signed_R_last_exist` | GIVEN — named directly in the plan |
| ~~`working_array_num`~~ | **REMOVED — CONFIRMED not a real external input.** It's a wire computed from `nBuf+nCal` inside `main_idle` (Controller.scala:248, `working_array_num = 4 if (nBuf+nCal)==0 else nBuf+nCal`), now done in `compute_array_mode()`. Previously modeled as a directly-settable `SetUpIO` field, which doesn't exist in `CAL_Payload`/`ISA_EXE` either |
| `dataIn_from_M_array` | GIVEN name; plumbing-only in this control-only model |
| C-array contents | OUT OF SCOPE — datapath black box |
| AutoLoadL completion | GIVEN — modeled as `io.l_fetch_done: Callable[[], bool]`, stub always `True`. **CONFIRMED scope boundary** (item 7): the request/response Decoupled handshake and `load_vec_state`'s own 3-phase sub-FSM stay deliberately out of scope (that's AutoLoadL-facing machinery), but `main_wait_L`'s OWN register writes around that handshake (`_L_vec_addr += 1`, `skip_read_M_array := False`) are in scope and are tracked |

## New hardware constant introduced this pass

`WBUF_NUM_SLOTS = 4` — CONFIRMED (Controller.scala:122,132,152 /
`PolymorPIC_Kernal_Config.segNum_in_per_word = bitlineNums/16`, default
`bitlineNums=64` → 4). A **fixed hardware-width constant**, unrelated to
the runtime `nCal` field. `wbuf_ptr_reg` is declared
`UInt(log2Ceil(segNum_in_per_word).W)` in the RTL — a genuinely
fixed-width register that **wraps on overflow** like any hardware adder;
the model previously treated it as an unbounded Python int.

## Assumptions, ranked by how much they'd change if wrong

1. ~~**`arrayMode_reg`'s four branches**~~ — **CONFIRMED**, see item 7.
2. ~~**`post_process`'s three branches' exact register values**~~ — **CONFIRMED**, see item 7.
3. ~~**`skip_read_M_array`'s source formula at `main_idle`**~~ — **CONFIRMED**: not written there at all (see the register row above).
4. ~~**`bitID_R[i]`'s active-lane value**~~ — **CONFIRMED**, see item 7.
5. ~~**`is_wBuf_ptr_end` / `is_last_L_block_row` source expressions**~~ — **CONFIRMED**, see item 7 (`is_wBuf_ptr_end` was the bigger surprise: not `nCal`-based at all).

6. ~~**`pre_check`'s next-state mapping and the "outer loop is bit-slice,
   inner loop is L-block-row" structure**~~ — **CONFIRMED against
   Controller.scala:347-383.** The (T,T)/(T,F)/(F,*) → main_idle /
   main_wait_L / main_wait_L mapping, and the outer/inner loop structure
   it implies, are exactly right. What *was* wrong were four of the
   register writes accompanying those transitions — now fixed in
   `pre_check()`:
     - `_L_vec_ptr_cur := 0` was missing from (T,T) (RTL fires it for
       both (T,T) and (T,F)).
     - `arrayCacheMode_reg := True` was missing from (T,T) entirely
       (and its `main_idle` counterpart `:= False`, and its `RegInit`
       default, were also missing/wrong — all three fixed together).
     - (T,T) incorrectly also reset `_L_bitSlice_ID_ptr`/`is_first_slice`
       (RTL doesn't touch either there); removed.
     - `read_C_ArrayAddr_reg := 0` was missing from (T,F). Initially
       flagged this as functionally significant (would skip the next
       bit-slice's whole C-array sweep) but **verified that's wrong**:
       `post_process`/`pre_check` are only ever reached via `cal()`'s own
       termination branch, which already zeroes `read_C_ArrayAddr_reg`
       right there — confirmed empirically by reverting just this one
       line and re-running the full suite. Fixed anyway for literal RTL
       fidelity; covered at the unit level by
       `test_pre_check_TF_more_bit_slices_resets_read_C_ArrayAddr`.

7. **`main_idle`, `main_wait_L`, `pre_read_M_array`, `cal`,
   `post_process` — second verification pass, all against
   `Controller.scala`.** `pre_read_M_array` had no mismatches. The other
   four did; all fixes below were confirmed by reverting them in
   isolation and checking the suite catches each one (see git history /
   conversation for the individual revert runs):

   - **`main_idle`**:
     - `bitID_R`'s formula was inverted (see register row above) — the
       model's `i<nBuf` computed a fake "active" value and `i>nBuf` used
       an `INACTIVE_BIT_ID=-1` sentinel; the RTL has no such sentinel
       (removed) and the roles were backwards.
     - `lastBitR_bidID` was derived from `bitID_R[nBuf]`; it's an
       independent formula.
     - `arrayMode_reg` used a "first N lanes = MAC" placeholder with a
       2-state enum; the RTL has 4 independently-formulated lanes and 3
       modes (Mem/Mac/IdleMac) — `cal`'s `read_C_ArrayEn` happened to
       still come out right for the 2-state version by coincidence
       (both non-Mac RTL modes map to "not Mac"), but the register's own
       written VALUES were wrong regardless.
     - `working_array_num` was modeled as an external `SetUpIO` field;
       it's a wire computed from `nBuf+nCal` (with a 0→4 special case).
     - `wbuf_ptr_reg`'s reset was a bare 0; it's
       `Mux(accWidth===ACC_32BIT,1,0)`, mirroring `post_process`
       branch ②'s reset.
     - `skip_read_M_array := False` was written here; the RTL doesn't
       touch it in `main_idle` at all (see register row — provably a
       no-op given the FSM's reachable paths, removed for fidelity).

   - **`main_wait_L`**: missing `_L_vec_addr += 1` (tracked for
     write-fidelity, not consumed by any control decision) and missing
     `skip_read_M_array := False` after consuming it (Controller.scala:408)
     — the latter is a real bug: a stale `True` left by `post_process`
     branch ③ would keep routing every SUBSEQUENT `main_wait_L` visit
     straight to `cal`, skipping `pre_read_M_array`'s M-array prefetch,
     even on visits nothing re-armed it for. `load_vec_state`'s own
     3-phase sub-FSM (send_L_req/wait_L_resp/start_next) and the
     request/response Decoupled handshake stay deliberately out of
     scope, per this file's own stated design for `io.l_fetch_done`.

   - **`cal`**: a Chisel Reg-read subtlety this file's earlier version
     missed — every read of a Reg within one always-executing block sees
     its PRE-EDGE value, even after an earlier `:=` to that same
     register in program order. Two consequences:
       1. `write_wBuf_wire`/`read_C_ArrayEn` (Controller.scala:295-297)
          are assigned BEFORE, and unconditionally with respect to, the
          termination check at :300 — so they still fire on the
          terminating cycle. `_C_array_EndPtr` is an INCLUSIVE bound
          (`EndPtr+1` total accumulate cycles), not exclusive. The old
          early-return-on-termination skipped the last cycle's
          accumulate/`read_C_ArrayEn` entirely.
       2. The mid-loop-flush guard (:310) is
          `is_wBuf_ptr_end && read_C_ArrayAddr_reg=/=0.U` — same
          pre-edge value, suppressing a flush on `cal`'s own first cycle
          of a fresh sweep. The `!= 0` half was missing entirely.

   - **`post_process`**: branch selection was already right (confirmed
     unchanged). Two VALUE mismatches: branch ① (`is_wBuf_ptr_end`)
     does NOT touch `wbuf_ptr_reg` in the RTL at all — the model
     incorrectly reset it to 0. Branch ②'s reset is accWidth-dependent
     (`Mux(accWidth===ACC_32BIT,1,0)`, mirroring `main_idle`'s own
     reset), not a bare 0.

   - **`tick_background`** (feeds every state, so listed here rather
     than under any one handler): `wbuf_ptr_reg`'s accumulate increment
     is accWidth-dependent (`buf_ptr_inc`, +2/+1) — was unconditionally
     +1. Separately, and more consequentially: `is_wBuf_ptr_end`'s
     formula was never actually `nCal`-based in the RTL at all (see
     `WBUF_NUM_SLOTS` above) — combined with `wbuf_ptr_reg` not wrapping
     at its real fixed hardware width, this made `is_wBuf_ptr_end`
     latch `True` forever once it crossed the (wrong) threshold, which
     is what `test_scenario_C_wBuf_fills_exactly_mid_cal` caught (it was
     asserting a `post_process` branch ① reset that, with both bugs
     fixed, turns out to actually be branch ②'s accWidth-aware reset —
     see that test's docstring for the full trace).

Everything NOT listed above (state names, the six-state topology's
non-branching edges, RegNext timing for `_M_array_dout_valid`/`write_wBuf`)
is either stated directly in the plan or a direct mechanical consequence
of something that is.
