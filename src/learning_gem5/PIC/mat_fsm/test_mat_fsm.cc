// Unit + integration tests for mat_fsm.hh/.cc -- a C++ mirror of
// test_mat_fsm.py's coverage, run against the ported MatFSM class to
// catch any translation bugs the Python model doesn't have.
//
// No gtest/scons dependency -- compiles and runs standalone:
//   g++ -std=c++17 -Wall -Wextra -o test_mat_fsm test_mat_fsm.cc mat_fsm.cc
//   ./test_mat_fsm

#include "mat_fsm.hh"

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

using namespace gem5::mat_fsm;

namespace
{

int g_failures = 0;
int g_total = 0;

#define CHECK(cond) \
    do { \
        ++g_total; \
        if (!(cond)) { \
            ++g_failures; \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

void
runTest(const std::string &name, const std::function<void()> &fn)
{
    const int before = g_failures;
    fn();
    std::printf("%s  %s\n", (g_failures == before) ? "PASS" : "FAIL", name.c_str());
}

const char *
stateName(MainState s)
{
    switch (s) {
      case MainState::MainIdle: return "main_idle";
      case MainState::MainWaitL: return "main_wait_L";
      case MainState::PreReadMArray: return "pre_read_M_array";
      case MainState::Cal: return "cal";
      case MainState::PostProcess: return "post_process";
      case MainState::PreCheck: return "pre_check";
    }
    return "?";
}

void
printTrace(const std::string &label, const std::vector<MainState> &trace)
{
    std::printf("%s trace: [", label.c_str());
    for (size_t i = 0; i < trace.size(); ++i) {
        std::printf("%s%s", (i ? ", " : ""), stateName(trace[i]));
    }
    std::printf("]\n");
}

// -----------------------------------------------------------------------
// mainIdle
// -----------------------------------------------------------------------

void test_main_idle_exec_true_one_time_setup()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 1; io.nCal = 4; io.accWidth = AccWidth16Bit;
    io.R_base_bit = 3; io.R_block_row = 10; io.L_block_row = 5;
    io.L_precision = 7; io.L_vec_fetch_addr = 0x1000;
    io.signed_L = true; io.signed_R_last_exist = false;

    const MainState next = fsm.step(io);

    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.cArrayEndPtr() == 10);
    CHECK(fsm.readCArrayAddrReg() == 0);
    CHECK(fsm.accWidthReg() == 16);
    CHECK(fsm.signedL() == true);
    CHECK(fsm.signedRLastExist() == false);
    CHECK(fsm.lBlockRow() == 5);
    CHECK(fsm.lPrecisionReg() == 7);
    CHECK(fsm.lVecAddr() == 0x1000);
    CHECK(fsm.lVecPtrCur() == 0);
    CHECK(fsm.lBitSliceIdPtr() == 0);
    CHECK(fsm.isFirstSlice() == true);
    CHECK(fsm.wbufPtrReg() == 0);           // 16-bit reset
    CHECK(fsm.writeMArrayRowAdrReg() == 0);
    CHECK(fsm.readMArrayRowAdrReg() == 0);
    CHECK(fsm.arrayCacheModeReg() == false); // "Disable cache mode"
}

void test_main_idle_wbuf_ptr_reg_reset_is_accWidth_dependent()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 1; io.nCal = 1; io.accWidth = AccWidth32Bit;
    fsm.step(io);
    CHECK(fsm.wbufPtrReg() == 1);
}

void test_main_idle_array_mode_branch_nBuf_gt_2()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 3; io.nCal = 0;
    fsm.step(io);
    const auto &m = fsm.arrayModeReg();
    CHECK(m[0] == ArrayMode::Mem);
    CHECK(m[1] == ArrayMode::Mem);
    CHECK(m[2] == ArrayMode::Mem);
    CHECK(m[3] == ArrayMode::IdleMac);
}

void test_main_idle_array_mode_branch_working_array_num_le_2()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 0; io.nCal = 2;
    fsm.step(io);
    const auto &m = fsm.arrayModeReg();
    CHECK(m[0] == ArrayMode::Mem);
    CHECK(m[1] == ArrayMode::Mac);
    CHECK(m[2] == ArrayMode::IdleMac);
    CHECK(m[3] == ArrayMode::IdleMac);
}

void test_main_idle_array_mode_branch_working_array_num_le_3()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 0; io.nCal = 3;
    fsm.step(io);
    const auto &m = fsm.arrayModeReg();
    CHECK(m[0] == ArrayMode::Mem);
    CHECK(m[1] == ArrayMode::Mac);
    CHECK(m[2] == ArrayMode::Mac);
    CHECK(m[3] == ArrayMode::IdleMac);
}

void test_main_idle_array_mode_branch_else()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 0; io.nCal = 4;
    fsm.step(io);
    const auto &m = fsm.arrayModeReg();
    CHECK(m[0] == ArrayMode::Mem);
    CHECK(m[1] == ArrayMode::Mac);
    CHECK(m[2] == ArrayMode::Mac);
    CHECK(m[3] == ArrayMode::Mac);
}

void test_main_idle_array_mode_working_array_num_zero_case_defaults_to_4()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 0; io.nCal = 0;   // nBuf+nCal==0 -> 4
    fsm.step(io);
    const auto &m = fsm.arrayModeReg();
    CHECK(m[0] == ArrayMode::Mem);
    CHECK(m[1] == ArrayMode::Mac);
    CHECK(m[2] == ArrayMode::Mac);
    CHECK(m[3] == ArrayMode::Mac);
}

void test_main_idle_bitID_R_and_lastBitR_bidID()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 2; io.nCal = 2; io.R_base_bit = 10;
    fsm.step(io);

    const auto &b = fsm.bitIdR();
    CHECK(b[0] == 0);
    CHECK(b[1] == 0);
    CHECK(b[2] == 10);
    CHECK(b[3] == 11);
    CHECK(fsm.lastBitRBidId() == 11);  // R_base_bit + nCal - 1 = 10+2-1
}

// -----------------------------------------------------------------------
// mainWaitL / preReadMArray -- exercised directly via setStateForTest,
// mirroring the Python tests calling the state functions in isolation.
// -----------------------------------------------------------------------

void test_main_wait_L_stub_not_done_stays_put()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLFetchDone([]() { return false; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.lVecPtrCur() == 0);
}

void test_main_wait_L_increments_ptr_and_routes_skip_true_to_cal()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLVecPtrCurForTest(3);
    fsm.setLVecAddrForTest(100);
    fsm.setSkipReadMArrayForTest(true);
    fsm.setLFetchDone([]() { return true; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(fsm.lVecPtrCur() == 4);
    CHECK(fsm.lVecAddr() == 101);
    CHECK(next == MainState::Cal);
    CHECK(fsm.skipReadMArray() == false);  // cleared after use
}

void test_main_wait_L_routes_skip_false_to_pre_read_M_array()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setSkipReadMArrayForTest(false);
    fsm.setLFetchDone([]() { return true; });
    SetUpIO io;
    CHECK(fsm.step(io) == MainState::PreReadMArray);
}

void test_pre_read_M_array_first_slice_no_read()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreReadMArray);
    fsm.setIsFirstSliceForTest(true);
    fsm.setReadMArrayRowAdrRegForTest(5);
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(fsm.readMArrayEnWire() == false);
    CHECK(fsm.readMArrayRowAdrReg() == 5);
    CHECK(next == MainState::Cal);
}

void test_pre_read_M_array_not_first_slice_issues_read()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreReadMArray);
    fsm.setIsFirstSliceForTest(false);
    fsm.setReadMArrayRowAdrRegForTest(5);
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(fsm.readMArrayEnWire() == true);
    CHECK(fsm.readMArrayRowAdrReg() == 6);
    CHECK(next == MainState::Cal);

    // Following tick_background() should show dout_valid via RegNext.
    fsm.step(io);  // runs `cal`, but tickBackground() at its top reads
                    // the readMArrayEnWire_ this call just left behind
    CHECK(fsm.mArrayDoutValid() == true);
}

// -----------------------------------------------------------------------
// cal -- the module this port was specifically requested for.
// -----------------------------------------------------------------------

void primeCal(MatFSM &fsm, uint64_t endPtr)
{
    fsm.setStateForTest(MainState::Cal);
    fsm.setCArrayEndPtrForTest(endPtr);
    fsm.setReadCArrayAddrRegForTest(0);
    std::array<ArrayMode, NumArrays> modes = {
        ArrayMode::Mac, ArrayMode::IdleMac, ArrayMode::Mac, ArrayMode::IdleMac
    };
    fsm.setArrayModeRegForTest(modes);
    fsm.setIsWBufPtrEndForTest(false);
}

void test_cal_normal_loop_write_wBuf_and_read_C_ArrayEn()
{
    MatFSM fsm;
    primeCal(fsm, 4);
    SetUpIO io;

    std::vector<uint64_t> seenAddrs;
    for (int i = 0; i < 4; ++i) {
        seenAddrs.push_back(fsm.readCArrayAddrReg());
        const MainState next = fsm.step(io);
        CHECK(fsm.writeWBufWire() == true);
        const auto &en = fsm.readCArrayEn();
        CHECK(en[0] == true && en[1] == false && en[2] == true && en[3] == false);
        CHECK(next == MainState::Cal);
    }
    CHECK((seenAddrs == std::vector<uint64_t>{0, 1, 2, 3}));
    CHECK(fsm.readCArrayAddrReg() == 4);
}

void test_cal_mid_loop_flush_not_first_slice()
{
    // Uses runStateHandlerOnlyForTest (not step()) so poking
    // isWBufPtrEnd_ directly sticks -- step() would run tickBackground()
    // first and recompute it from wbufPtrReg_, clobbering the injected
    // value (this mirrors test_mat_fsm.py calling cal(regs, io) directly,
    // bypassing tick_background()).
    MatFSM fsm;
    primeCal(fsm, 10);
    fsm.setIsFirstSliceForTest(false);
    fsm.setReadMArrayRowAdrRegForTest(0);
    fsm.setReadCArrayAddrRegForTest(1);  // old_addr must be != 0 to flush
    fsm.setIsWBufPtrEndForTest(true);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);
    CHECK(fsm.writeMArrayEnWire() == true);
    CHECK(fsm.readMArrayEnWire() == true);
    CHECK(fsm.readMArrayRowAdrReg() == 1);
}

void test_cal_mid_loop_flush_first_slice()
{
    MatFSM fsm;
    primeCal(fsm, 10);
    fsm.setIsFirstSliceForTest(true);
    fsm.setReadCArrayAddrRegForTest(1);  // old_addr must be != 0
    fsm.setIsWBufPtrEndForTest(true);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);
    CHECK(fsm.writeMArrayEnWire() == true);
    CHECK(fsm.readMArrayEnWire() == false);
}

void test_cal_mid_loop_flush_suppressed_on_cycle_zero()
{
    // CONFIRMED-against-RTL subtlety (Controller.scala:310): the flush
    // guard reads read_C_ArrayAddr_reg's OLD value, so even if
    // is_wBuf_ptr_end is already true, cal's very first cycle of a fresh
    // sweep (old_addr==0) must NOT flush.
    MatFSM fsm;
    primeCal(fsm, 10);
    fsm.setIsWBufPtrEndForTest(true);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);
    CHECK(fsm.writeMArrayEnWire() == false);
    CHECK(fsm.readMArrayEnWire() == false);
}

void test_cal_termination_is_inclusive_and_resets()
{
    // CONFIRMED-against-RTL subtlety: _C_array_EndPtr is an INCLUSIVE
    // bound -- the terminating cycle still fires write_wBuf_wire/
    // read_C_ArrayEn before transitioning.
    MatFSM fsm;
    primeCal(fsm, 2);
    SetUpIO io;

    CHECK(fsm.step(io) == MainState::Cal);   // addr 0 -> 1
    CHECK(fsm.step(io) == MainState::Cal);   // addr 1 -> 2
    CHECK(fsm.readCArrayAddrReg() == 2);

    const MainState next = fsm.step(io);     // addr == EndPtr -> terminate
    CHECK(fsm.writeWBufWire() == true);      // fired even on this cycle
    CHECK(next == MainState::PostProcess);
    CHECK(fsm.readCArrayAddrReg() == 0);
}

// -----------------------------------------------------------------------
// postProcess -- three mutually exclusive branches
// -----------------------------------------------------------------------

void test_post_process_branch1_wBuf_ended_does_not_reset_ptr()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PostProcess);
    fsm.setIsWBufPtrEndForTest(true);
    fsm.setIsLastLBlockRowForTest(false);
    fsm.setWbufPtrRegForTest(2);
    fsm.setSkipReadMArrayForTest(true);  // must NOT be touched by branch 1
    SetUpIO io;

    const MainState next = fsm.runStateHandlerOnlyForTest(io);
    CHECK(next == MainState::PreCheck);
    CHECK(fsm.writeMArrayEnWire() == true);
    CHECK(fsm.wbufPtrReg() == 2);        // untouched, NOT reset to 0
    CHECK(fsm.skipReadMArray() == true); // untouched
}

void test_post_process_branch2_last_row_not_full_accWidth16()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PostProcess);
    fsm.setIsWBufPtrEndForTest(false);
    fsm.setIsLastLBlockRowForTest(true);
    fsm.setAccWidthRegForTest(AccWidth16Bit);
    SetUpIO io;

    fsm.step(io);
    CHECK(fsm.writeMArrayEnWire() == true);
    CHECK(fsm.wbufPtrReg() == 0);
    CHECK(fsm.skipReadMArray() == false);
}

void test_post_process_branch2_accWidth32_resets_to_one()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PostProcess);
    fsm.setIsWBufPtrEndForTest(false);
    fsm.setIsLastLBlockRowForTest(true);
    fsm.setAccWidthRegForTest(AccWidth32Bit);
    SetUpIO io;

    fsm.step(io);
    CHECK(fsm.wbufPtrReg() == 1);
}

void test_post_process_branch3_common_inner_case()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PostProcess);
    fsm.setIsWBufPtrEndForTest(false);
    fsm.setIsLastLBlockRowForTest(false);
    fsm.setWbufPtrRegForTest(3);
    SetUpIO io;

    const MainState next = fsm.runStateHandlerOnlyForTest(io);
    CHECK(next == MainState::PreCheck);
    CHECK(fsm.writeMArrayEnWire() == false);
    CHECK(fsm.wbufPtrReg() == 3);   // not reset
    CHECK(fsm.skipReadMArray() == true);
}

// -----------------------------------------------------------------------
// preCheck -- (T,T) / (T,F) / (F,*)
// -----------------------------------------------------------------------

void test_pre_check_TT_fully_done_to_main_idle()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreCheck);
    fsm.setIsLastLBlockRowForTest(true);
    fsm.setLBitSliceIdPtrForTest(7);
    fsm.setLPrecisionRegForTest(7);
    fsm.setReadCArrayAddrRegForTest(3);
    fsm.setReadMArrayRowAdrRegForTest(3);
    SetUpIO io;

    const MainState next = fsm.runStateHandlerOnlyForTest(io);
    CHECK(next == MainState::MainIdle);
    CHECK(fsm.readCArrayAddrReg() == 0);
    CHECK(fsm.readMArrayRowAdrReg() == 0);
    CHECK(fsm.arrayCacheModeReg() == true);   // "Restore cache state"
}

void test_pre_check_TF_more_bit_slices_to_main_wait_L()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreCheck);
    fsm.setIsLastLBlockRowForTest(true);
    fsm.setLBitSliceIdPtrForTest(0);
    fsm.setLPrecisionRegForTest(7);
    fsm.setIsFirstSliceForTest(true);
    fsm.setLVecPtrCurForTest(5);
    fsm.setReadCArrayAddrRegForTest(9);  // must be reset (RTL fidelity)
    SetUpIO io;

    const MainState next = fsm.runStateHandlerOnlyForTest(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.lBitSliceIdPtr() == 1);
    CHECK(fsm.lVecPtrCur() == 0);
    CHECK(fsm.isFirstSlice() == false);
    CHECK(fsm.readCArrayAddrReg() == 0);
}

void test_pre_check_F_star_more_rows_to_main_wait_L()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreCheck);
    fsm.setIsLastLBlockRowForTest(false);
    SetUpIO io;

    for (bool bitSliceDone : {true, false}) {
        fsm.setLBitSliceIdPtrForTest(bitSliceDone ? 7 : 0);
        fsm.setLPrecisionRegForTest(7);
        fsm.setStateForTest(MainState::PreCheck);
        CHECK(fsm.runStateHandlerOnlyForTest(io) == MainState::MainWaitL);
    }
}

// -----------------------------------------------------------------------
// Integration scenarios
// -----------------------------------------------------------------------

void test_scenario_A_minimal_single_pass()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 1; io.nCal = 4; io.accWidth = AccWidth16Bit;
    io.R_block_row = 3; io.L_block_row = 1; io.L_precision = 0;

    std::vector<MainState> trace{fsm.state()};
    bool seenAll[6] = {false, false, false, false, false, false};
    for (int i = 0; i < 60; ++i) {
        fsm.step(io);
        trace.push_back(fsm.state());
        seenAll[static_cast<int>(fsm.state())] = true;
        if (fsm.state() == MainState::MainIdle && trace.size() > 1) break;
    }
    printTrace("Scenario A", trace);

    CHECK(trace.front() == MainState::MainIdle);
    CHECK(trace.back() == MainState::MainIdle);
    for (bool seen : seenAll) CHECK(seen);
}

void test_scenario_B_multiple_bit_slices_loops_to_wait_L()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 1; io.nCal = 2; io.accWidth = AccWidth32Bit;
    io.R_block_row = 2; io.L_block_row = 1; io.L_precision = 1;  // 2 bit-slices

    fsm.step(io);  // consume exec -> main_wait_L
    CHECK(fsm.isFirstSlice() == true);

    std::vector<MainState> trace{MainState::MainWaitL};
    bool sawFlip = false;
    for (int i = 0; i < 60; ++i) {
        fsm.step(io);
        trace.push_back(fsm.state());
        if (fsm.state() == MainState::MainWaitL && !fsm.isFirstSlice()) {
            sawFlip = true;
            break;
        }
    }
    printTrace("Scenario B", trace);
    CHECK(sawFlip);
}

void test_scenario_C_wBuf_fills_exactly_mid_cal()
{
    MatFSM fsm;
    SetUpIO io;
    io.exec = true; io.nBuf = 1; io.nCal = 2; io.accWidth = AccWidth16Bit;
    io.R_block_row = 8; io.L_block_row = 1; io.L_precision = 0;

    fsm.step(io);  // main_idle -> main_wait_L
    fsm.step(io);  // main_wait_L -> pre_read_M_array (skip=false default)
    fsm.step(io);  // pre_read_M_array -> cal (is_first_slice=true path)

    std::vector<MainState> trace{MainState::Cal};
    bool hitFlush = false;
    while (fsm.state() != MainState::PostProcess) {
        fsm.step(io);
        trace.push_back(fsm.state());
        if (fsm.writeMArrayEnWire()) hitFlush = true;
    }
    printTrace("Scenario C", trace);
    CHECK(hitFlush);

    fsm.step(io);  // run post_process
    // Whether wbuf_ptr_reg is reset here depends on WHICH branch fired
    // (isWBufPtrEnd_ -> untouched; isLastLBlockRow_ -> accWidth-aware
    // reset) -- both are legitimate RTL-confirmed outcomes depending on
    // exactly when is_wBuf_ptr_end last became true relative to cal's
    // termination, so this scenario only asserts the flush happened
    // somewhere along the way (checked above), matching the Python
    // model's own scenario docstring note about this ambiguity.
}

} // namespace

int
main()
{
    std::vector<std::pair<std::string, std::function<void()>>> tests = {
        {"test_main_idle_exec_true_one_time_setup", test_main_idle_exec_true_one_time_setup},
        {"test_main_idle_wbuf_ptr_reg_reset_is_accWidth_dependent", test_main_idle_wbuf_ptr_reg_reset_is_accWidth_dependent},
        {"test_main_idle_array_mode_branch_nBuf_gt_2", test_main_idle_array_mode_branch_nBuf_gt_2},
        {"test_main_idle_array_mode_branch_working_array_num_le_2", test_main_idle_array_mode_branch_working_array_num_le_2},
        {"test_main_idle_array_mode_branch_working_array_num_le_3", test_main_idle_array_mode_branch_working_array_num_le_3},
        {"test_main_idle_array_mode_branch_else", test_main_idle_array_mode_branch_else},
        {"test_main_idle_array_mode_working_array_num_zero_case_defaults_to_4", test_main_idle_array_mode_working_array_num_zero_case_defaults_to_4},
        {"test_main_idle_bitID_R_and_lastBitR_bidID", test_main_idle_bitID_R_and_lastBitR_bidID},
        {"test_main_wait_L_stub_not_done_stays_put", test_main_wait_L_stub_not_done_stays_put},
        {"test_main_wait_L_increments_ptr_and_routes_skip_true_to_cal", test_main_wait_L_increments_ptr_and_routes_skip_true_to_cal},
        {"test_main_wait_L_routes_skip_false_to_pre_read_M_array", test_main_wait_L_routes_skip_false_to_pre_read_M_array},
        {"test_pre_read_M_array_first_slice_no_read", test_pre_read_M_array_first_slice_no_read},
        {"test_pre_read_M_array_not_first_slice_issues_read", test_pre_read_M_array_not_first_slice_issues_read},
        {"test_cal_normal_loop_write_wBuf_and_read_C_ArrayEn", test_cal_normal_loop_write_wBuf_and_read_C_ArrayEn},
        {"test_cal_mid_loop_flush_not_first_slice", test_cal_mid_loop_flush_not_first_slice},
        {"test_cal_mid_loop_flush_first_slice", test_cal_mid_loop_flush_first_slice},
        {"test_cal_mid_loop_flush_suppressed_on_cycle_zero", test_cal_mid_loop_flush_suppressed_on_cycle_zero},
        {"test_cal_termination_is_inclusive_and_resets", test_cal_termination_is_inclusive_and_resets},
        {"test_post_process_branch1_wBuf_ended_does_not_reset_ptr", test_post_process_branch1_wBuf_ended_does_not_reset_ptr},
        {"test_post_process_branch2_last_row_not_full_accWidth16", test_post_process_branch2_last_row_not_full_accWidth16},
        {"test_post_process_branch2_accWidth32_resets_to_one", test_post_process_branch2_accWidth32_resets_to_one},
        {"test_post_process_branch3_common_inner_case", test_post_process_branch3_common_inner_case},
        {"test_pre_check_TT_fully_done_to_main_idle", test_pre_check_TT_fully_done_to_main_idle},
        {"test_pre_check_TF_more_bit_slices_to_main_wait_L", test_pre_check_TF_more_bit_slices_to_main_wait_L},
        {"test_pre_check_F_star_more_rows_to_main_wait_L", test_pre_check_F_star_more_rows_to_main_wait_L},
        {"test_scenario_A_minimal_single_pass", test_scenario_A_minimal_single_pass},
        {"test_scenario_B_multiple_bit_slices_loops_to_wait_L", test_scenario_B_multiple_bit_slices_loops_to_wait_L},
        {"test_scenario_C_wBuf_fills_exactly_mid_cal", test_scenario_C_wBuf_fills_exactly_mid_cal},
    };

    for (auto &[name, fn] : tests) {
        runTest(name, fn);
    }

    std::printf("\n%d/%d passed\n", g_total - g_failures, g_total);
    return g_failures == 0 ? 0 : 1;
}
