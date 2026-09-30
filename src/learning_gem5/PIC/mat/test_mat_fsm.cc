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

void test_load_vec_state_send_l_req_stalls_without_arbiter_grant()
{
    // SendLReq (a): requestVecFire refuses -> stalls, no writes.
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::SendLReq);
    fsm.setLVecAddrForTest(100);
    fsm.setRequestVecFire([]() { return false; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.loadVecState() == LoadVecState::SendLReq);
    CHECK(fsm.lVecAddr() == 100);
}

void test_load_vec_state_send_l_req_fires_advances_to_wait_l_resp()
{
    // SendLReq (b): fires -> lVecAddr_+=1, -> WaitLResp. mainState stays
    // MainWaitL (only StartNext leaves).
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::SendLReq);
    fsm.setLVecAddrForTest(100);
    fsm.setRequestVecFire([]() { return true; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.loadVecState() == LoadVecState::WaitLResp);
    CHECK(fsm.lVecAddr() == 101);
}

void test_load_vec_state_wait_l_resp_stalls_without_response()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::WaitLResp);
    fsm.setResponseVecFire([]() { return false; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.loadVecState() == LoadVecState::WaitLResp);
}

void test_load_vec_state_wait_l_resp_fires_advances_to_start_next()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::WaitLResp);
    fsm.setResponseVecFire([]() { return true; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::MainWaitL);
    CHECK(fsm.loadVecState() == LoadVecState::StartNext);
}

void test_load_vec_state_start_next_always_advances_regardless_of_gates()
{
    // StartNext: no gating condition -- fires even if both handshake
    // gates would refuse, since it doesn't consult them at all.
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::StartNext);
    fsm.setSkipReadMArrayForTest(true);
    fsm.setRequestVecFire([]() { return false; });
    fsm.setResponseVecFire([]() { return false; });
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(next == MainState::Cal);  // left MainWaitL despite both gates refusing
    CHECK(fsm.loadVecState() == LoadVecState::SendLReq);  // reset for next time
}

void test_load_vec_state_start_next_increments_ptr_and_routes_skip_true_to_cal()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::StartNext);
    fsm.setLVecPtrCurForTest(3);
    fsm.setSkipReadMArrayForTest(true);
    SetUpIO io;
    const MainState next = fsm.step(io);
    CHECK(fsm.lVecPtrCur() == 4);
    CHECK(next == MainState::Cal);
    CHECK(fsm.skipReadMArray() == false);  // cleared after use
}

void test_load_vec_state_start_next_routes_skip_false_to_pre_read_M_array()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::StartNext);
    fsm.setSkipReadMArrayForTest(false);
    SetUpIO io;
    CHECK(fsm.step(io) == MainState::PreReadMArray);
}

void test_main_wait_L_full_round_trip_no_stall_takes_exactly_three_cycles()
{
    // Integration-level check: with both gates firing immediately, a
    // fresh MainWaitL entry (loadVecState_ defaults to SendLReq) takes
    // exactly 3 step() calls to leave -- one per sub-state, per the
    // always-one-state-per-call convention used throughout this class.
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLVecPtrCurForTest(0);
    fsm.setLVecAddrForTest(100);
    SetUpIO io;  // both gates default to always-fire

    CHECK(fsm.loadVecState() == LoadVecState::SendLReq);
    CHECK(fsm.step(io) == MainState::MainWaitL);       // cycle 1: SendLReq fires
    CHECK(fsm.loadVecState() == LoadVecState::WaitLResp);
    CHECK(fsm.lVecAddr() == 101);

    CHECK(fsm.step(io) == MainState::MainWaitL);       // cycle 2: WaitLResp fires
    CHECK(fsm.loadVecState() == LoadVecState::StartNext);

    const MainState next = fsm.step(io);               // cycle 3: StartNext
    CHECK(next == MainState::PreReadMArray);
    CHECK(fsm.loadVecState() == LoadVecState::SendLReq);  // reset for next visit
    CHECK(fsm.lVecPtrCur() == 1);
}

void test_main_wait_L_arbiter_stall_extends_send_l_req()
{
    // Arbiter-contention case: requestVecFire refuses for a few cycles
    // before granting -- MainWaitL (and loadVecState_) must stay parked
    // at SendLReq for exactly that long, not advance early.
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    const int callsBeforeGrant = 3;
    int callCount = 0;
    fsm.setRequestVecFire([&callCount]() {
        ++callCount;
        return callCount > callsBeforeGrant;
    });
    SetUpIO io;

    for (int i = 0; i < callsBeforeGrant; ++i) {
        CHECK(fsm.step(io) == MainState::MainWaitL);
        CHECK(fsm.loadVecState() == LoadVecState::SendLReq);
    }
    CHECK(fsm.step(io) == MainState::MainWaitL);  // the grant cycle
    CHECK(fsm.loadVecState() == LoadVecState::WaitLResp);
}

void test_main_wait_L_response_stall_extends_wait_l_resp()
{
    // Fetch-latency case: responseVecFire refuses for a few cycles
    // before AutoLoadL finishes -- mirrors the arbiter-stall test above
    // for the other handshake gate.
    MatFSM fsm;
    fsm.setStateForTest(MainState::MainWaitL);
    fsm.setLoadVecStateForTest(LoadVecState::WaitLResp);
    const int callsBeforeResponse = 2;
    int callCount = 0;
    fsm.setResponseVecFire([&callCount]() {
        ++callCount;
        return callCount > callsBeforeResponse;
    });
    SetUpIO io;

    for (int i = 0; i < callsBeforeResponse; ++i) {
        CHECK(fsm.step(io) == MainState::MainWaitL);
        CHECK(fsm.loadVecState() == LoadVecState::WaitLResp);
    }
    CHECK(fsm.step(io) == MainState::MainWaitL);  // the response-arrives cycle
    CHECK(fsm.loadVecState() == LoadVecState::StartNext);
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

// ---------------------------------------------------------------------
// cal() Datapath Spec -- C-array SRAM / vecBuf / rBuf / wBuf / M-array
// ---------------------------------------------------------------------

void test_cal_reads_rVec_from_cArraySram_on_mac_lanes()
{
    MatFSM fsm;
    primeCal(fsm, 0);
    fsm.datapath().setCArrayWordForTest(0, 0, 0xAAAA);
    fsm.datapath().setCArrayWordForTest(2, 0, 0xBBBB);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);

    const auto &rVec = fsm.datapath().rVec();
    CHECK(rVec[0] == 0xAAAA);
    CHECK(rVec[1] == 0);
    CHECK(rVec[2] == 0xBBBB);
    CHECK(rVec[3] == 0);
}

void test_pre_read_M_array_sets_addr_wire_to_old_row()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PreReadMArray);
    fsm.setIsFirstSliceForTest(false);
    fsm.setReadMArrayRowAdrRegForTest(5);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);

    CHECK(fsm.readMArrayRowAdrReg() == 6);
    // step() would latch mArrayReadAddrWire_ into rBuf next cycle; verify
    // that round trip end to end via a real M-array read below.
}

void test_marray_read_unpacks_into_rBuf()
{
    MatFSM fsm;
    fsm.datapath().setMArrayWordForTest(5, 0x4444333322221111ULL);
    fsm.setStateForTest(MainState::PreReadMArray);
    fsm.setIsFirstSliceForTest(false);
    fsm.setReadMArrayRowAdrRegForTest(5);
    SetUpIO io;

    fsm.step(io);  // issues the read, addressed at row 5
    fsm.step(io);  // RegNext: dout_valid, rBuf unpacked

    CHECK(fsm.mArrayDoutValid() == true);
    const auto &rBuf = fsm.datapath().rBuf();
    CHECK(rBuf[0] == 0x1111);
    CHECK(rBuf[1] == 0x2222);
    CHECK(rBuf[2] == 0x3333);
    CHECK(rBuf[3] == 0x4444);
}

void test_accumulate_adder_16bit_mode()
{
    // MatDatapath::accumulate() is a plain function of its arguments now
    // (macResult is just passed in) -- test it directly, no FSM needed.
    MatDatapath dp;
    std::array<int16_t, WBufNumSlots> rBuf{};
    rBuf[2] = 100;
    dp.setRBufForTest(rBuf);

    dp.accumulate(/*idx=*/2, AccWidth16Bit, /*isFirstSlice=*/false,
                  /*freshMArrayValid=*/false, /*macResult=*/5);

    CHECK(dp.wBuf()[2] == 105);
}

void test_accumulate_adder_32bit_mode_combines_two_lanes()
{
    MatDatapath dp;
    std::array<int16_t, WBufNumSlots> rBuf{};
    rBuf[3] = 1;
    rBuf[2] = 2;
    dp.setRBufForTest(rBuf);

    dp.accumulate(/*idx=*/3, AccWidth32Bit, /*isFirstSlice=*/false,
                  /*freshMArrayValid=*/false, /*macResult=*/3);

    CHECK(dp.wBuf()[3] == 1);
    CHECK(dp.wBuf()[2] == 5);
}

void test_accumulate_uses_zero_previous_on_first_slice()
{
    // Rule 8/16: first-slice override -- previous partial sum is 0
    // regardless of whatever stale rBuf_/rWire_ contents remain from an
    // earlier command.
    MatDatapath dp;
    std::array<int16_t, WBufNumSlots> rBuf{};
    rBuf[1] = 999;  // stale, must NOT be used
    dp.setRBufForTest(rBuf);

    dp.accumulate(/*idx=*/1, AccWidth16Bit, /*isFirstSlice=*/true,
                  /*freshMArrayValid=*/false, /*macResult=*/5);

    CHECK(dp.wBuf()[1] == 5);
}

void test_accumulate_prefers_fresh_rWire_over_rBuf()
{
    // Rule 8/15: when fresh M-array data landed THIS cycle, use rWire_,
    // not the older latched rBuf_.
    MatDatapath dp;
    std::array<int16_t, WBufNumSlots> rWire{};
    rWire[0] = 10;
    dp.setRWireForTest(rWire);
    std::array<int16_t, WBufNumSlots> rBuf{};
    rBuf[0] = 999;  // stale, must NOT be used
    dp.setRBufForTest(rBuf);

    dp.accumulate(/*idx=*/0, AccWidth16Bit, /*isFirstSlice=*/false,
                  /*freshMArrayValid=*/true, /*macResult=*/1);

    CHECK(dp.wBuf()[0] == 11);
}

void test_latch_marray_row_updates_both_rWire_and_rBuf()
{
    MatDatapath dp;
    dp.setMArrayWordForTest(2, 0x4444333322221111ULL);

    dp.latchMArrayRow(2);

    CHECK(dp.rWire()[0] == 0x1111);
    CHECK(dp.rBuf()[0] == 0x1111);
    CHECK(dp.rWire()[3] == 0x4444);
    CHECK(dp.rBuf()[3] == 0x4444);
}

void test_load_vec_writes_vecBuf_only_when_enabled()
{
    MatDatapath dp;
    dp.loadVec(false, 0xDEAD);
    CHECK(dp.vecBuf() == 0);

    dp.loadVec(true, 0xBEEF);
    CHECK(dp.vecBuf() == 0xBEEF);
}

// ---------------------------------------------------------------------
// MatMac (cal() Datapath / MAC Spec)
// ---------------------------------------------------------------------

void test_mac_shift_combines_bitIdR_and_lBitSliceId()
{
    // Rule 1: shift = bitIdR[i] + lBitSliceId.
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0x1);  // read into rVec via readCArray below
    dp.readCArray(0, {true, false, false, false});
    dp.loadVec(true, 0x1);  // AND = 0x1, popcount = 1

    MacControl ctrl;
    ctrl.macEnable = {true, false, false, false};
    ctrl.bitIdR[0] = 3;
    ctrl.lBitSliceId = 2;
    // shift = 3+2 = 5 -> 1 << 5 = 32

    CHECK(MatMac::sumOfMac(dp, ctrl) == 32);
}

void test_mac_rSign_negates_when_lane_holds_R_sign_bit()
{
    // Rule 2: rSign only when signedRLastExist && bitIdR[i]==lastBitRBitId
    // (and the lane is MAC-enabled).
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0x1);
    dp.readCArray(0, {true, false, false, false});
    dp.loadVec(true, 0x1);

    MacControl ctrl;
    ctrl.macEnable = {true, false, false, false};
    ctrl.signedRLastExist = true;
    ctrl.bitIdR[0] = 0;      // no shift, isolates the sign effect
    ctrl.lastBitRBitId = 0;  // this lane holds R's sign bit

    CHECK(MatMac::sumOfMac(dp, ctrl) == -1);
}

void test_mac_lSign_negates_when_L_reaches_its_last_bit()
{
    // Rule 3: lSign uses ==, not ==-1 -- lPrecisionReg_ IS the final index.
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0x1);
    dp.readCArray(0, {true, false, false, false});
    dp.loadVec(true, 0x1);

    MacControl ctrl;
    ctrl.macEnable = {true, false, false, false};
    ctrl.signedL = true;
    ctrl.lBitSliceId = 0;    // no shift, isolates the sign effect
    ctrl.lPrecisionReg = 0;  // ==, so lSign true here

    CHECK(MatMac::sumOfMac(dp, ctrl) == -1);
}

void test_mac_both_signs_cancel_to_positive()
{
    // Rule 4: negate = rSign XOR lSign -- both signed -> positive result.
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0x1);
    dp.readCArray(0, {true, false, false, false});
    dp.loadVec(true, 0x1);

    MacControl ctrl;
    ctrl.macEnable = {true, false, false, false};
    ctrl.signedRLastExist = true;
    ctrl.bitIdR[0] = 0;
    ctrl.lastBitRBitId = 0;
    ctrl.signedL = true;
    ctrl.lBitSliceId = 0;    // no shift, isolates the sign effect
    ctrl.lPrecisionReg = 0;  // ==, so lSign true too

    CHECK(MatMac::sumOfMac(dp, ctrl) == 1);
}

void test_mac_non_mac_lane_output_zero_even_with_nonzero_sram()
{
    // Rule 6/8: a non-MAC-mode lane's SRAM may hold real (nonzero) data
    // (M-array traffic reusing that PolyArray's port) -- the MAC OUTPUT
    // must still be forced to 0, not derived from the SRAM being zero.
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0xFFFFFFFFFFFFFFFFULL);
    dp.readCArray(0, {true, false, false, false});  // lane 0 reads, but...
    dp.loadVec(true, 0xFFFFFFFFFFFFFFFFULL);

    MacControl ctrl;
    ctrl.macEnable = {false, false, false, false};  // ...is NOT MAC-mode

    CHECK(MatMac::sumOfMac(dp, ctrl) == 0);
}

void test_mac_reduces_all_four_lanes()
{
    // Adder #1: four independently shifted/signed lanes, summed.
    MatDatapath dp;
    dp.setCArrayWordForTest(0, 0, 0x1);
    dp.setCArrayWordForTest(1, 0, 0x1);
    dp.setCArrayWordForTest(2, 0, 0x1);
    dp.setCArrayWordForTest(3, 0, 0x1);
    dp.readCArray(0, {true, true, true, true});
    dp.loadVec(true, 0x1);

    MacControl ctrl;
    ctrl.macEnable = {true, true, true, true};
    // no shift/sign -- each lane contributes popcount(1)=1
    CHECK(MatMac::sumOfMac(dp, ctrl) == 4);
}

void test_full_pipeline_cal_step_computes_real_mac_into_wbuf()
{
    // End-to-end through fsm.step()'s real tickBackground -> MacControl ->
    // MatMac -> accumulate wiring (not manually poking MatDatapath), to
    // catch wiring bugs the isolated unit tests above wouldn't.
    MatFSM fsm;
    primeCal(fsm, 0);  // arrayModeReg = [Mac, IdleMac, Mac, IdleMac]
    fsm.setAccWidthRegForTest(AccWidth16Bit);
    fsm.setWbufPtrRegForTest(0);
    fsm.datapath().setCArrayWordForTest(0, 0, 0x3);
    fsm.datapath().setCArrayWordForTest(2, 0, 0x5);
    fsm.datapath().loadVec(true, 0x7);
    SetUpIO io;

    fsm.step(io);  // cal fires writeWBufWire_, reads rVec via lanes 0/2
    fsm.step(io);  // RegNext: MatMac + accumulate

    // popcount(0x3&0x7)=2, popcount(0x5&0x7)=2, no shift/sign -> sum=4;
    // first slice -> previous=0 -> wBuf[0] = 0+4 = 4.
    CHECK(fsm.datapath().wBuf()[0] == 4);
}

void test_post_process_commits_wbuf_to_marray()
{
    MatFSM fsm;
    fsm.setStateForTest(MainState::PostProcess);
    fsm.setIsWBufPtrEndForTest(true);
    std::array<int16_t, WBufNumSlots> wBuf = {0x1111, 0x2222, 0x3333, 0x4444};
    fsm.datapath().setWBufForTest(wBuf);
    SetUpIO io;

    fsm.runStateHandlerOnlyForTest(io);

    CHECK(fsm.datapath().mArrayWord(0) == 0x4444333322221111ULL);
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
        {"test_load_vec_state_send_l_req_stalls_without_arbiter_grant", test_load_vec_state_send_l_req_stalls_without_arbiter_grant},
        {"test_load_vec_state_send_l_req_fires_advances_to_wait_l_resp", test_load_vec_state_send_l_req_fires_advances_to_wait_l_resp},
        {"test_load_vec_state_wait_l_resp_stalls_without_response", test_load_vec_state_wait_l_resp_stalls_without_response},
        {"test_load_vec_state_wait_l_resp_fires_advances_to_start_next", test_load_vec_state_wait_l_resp_fires_advances_to_start_next},
        {"test_load_vec_state_start_next_always_advances_regardless_of_gates", test_load_vec_state_start_next_always_advances_regardless_of_gates},
        {"test_load_vec_state_start_next_increments_ptr_and_routes_skip_true_to_cal", test_load_vec_state_start_next_increments_ptr_and_routes_skip_true_to_cal},
        {"test_load_vec_state_start_next_routes_skip_false_to_pre_read_M_array", test_load_vec_state_start_next_routes_skip_false_to_pre_read_M_array},
        {"test_main_wait_L_full_round_trip_no_stall_takes_exactly_three_cycles", test_main_wait_L_full_round_trip_no_stall_takes_exactly_three_cycles},
        {"test_main_wait_L_arbiter_stall_extends_send_l_req", test_main_wait_L_arbiter_stall_extends_send_l_req},
        {"test_main_wait_L_response_stall_extends_wait_l_resp", test_main_wait_L_response_stall_extends_wait_l_resp},
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
        {"test_cal_reads_rVec_from_cArraySram_on_mac_lanes", test_cal_reads_rVec_from_cArraySram_on_mac_lanes},
        {"test_pre_read_M_array_sets_addr_wire_to_old_row", test_pre_read_M_array_sets_addr_wire_to_old_row},
        {"test_marray_read_unpacks_into_rBuf", test_marray_read_unpacks_into_rBuf},
        {"test_accumulate_adder_16bit_mode", test_accumulate_adder_16bit_mode},
        {"test_accumulate_adder_32bit_mode_combines_two_lanes", test_accumulate_adder_32bit_mode_combines_two_lanes},
        {"test_accumulate_uses_zero_previous_on_first_slice", test_accumulate_uses_zero_previous_on_first_slice},
        {"test_accumulate_prefers_fresh_rWire_over_rBuf", test_accumulate_prefers_fresh_rWire_over_rBuf},
        {"test_latch_marray_row_updates_both_rWire_and_rBuf", test_latch_marray_row_updates_both_rWire_and_rBuf},
        {"test_load_vec_writes_vecBuf_only_when_enabled", test_load_vec_writes_vecBuf_only_when_enabled},
        {"test_mac_shift_combines_bitIdR_and_lBitSliceId", test_mac_shift_combines_bitIdR_and_lBitSliceId},
        {"test_mac_rSign_negates_when_lane_holds_R_sign_bit", test_mac_rSign_negates_when_lane_holds_R_sign_bit},
        {"test_mac_lSign_negates_when_L_reaches_its_last_bit", test_mac_lSign_negates_when_L_reaches_its_last_bit},
        {"test_mac_both_signs_cancel_to_positive", test_mac_both_signs_cancel_to_positive},
        {"test_mac_non_mac_lane_output_zero_even_with_nonzero_sram", test_mac_non_mac_lane_output_zero_even_with_nonzero_sram},
        {"test_mac_reduces_all_four_lanes", test_mac_reduces_all_four_lanes},
        {"test_full_pipeline_cal_step_computes_real_mac_into_wbuf", test_full_pipeline_cal_step_computes_real_mac_into_wbuf},
        {"test_post_process_commits_wbuf_to_marray", test_post_process_commits_wbuf_to_marray},
    };

    for (auto &[name, fn] : tests) {
        runTest(name, fn);
    }

    std::printf("\n%d/%d passed\n", g_total - g_failures, g_total);
    return g_failures == 0 ? 0 : 1;
}
