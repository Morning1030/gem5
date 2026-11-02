# Same cal() job and checks as cal_mat_e2e.py, except the Mat is started
# the way the RTL starts it: by an EXE command through the frontend.
#
#   PicTestFrontend (EXE: SET_SRC, SET_DST, SET_PARAM) -> PicMmioTransport
#     -> Scheduler (decode, cb_port) -> MatSetUpRouter -> the Mat under test
#
# The Mats get no cal() parameters of their own (wait_for_setup=True), so
# the job only computes the right M if every EXE field makes it through the
# frontend, the Scheduler's decode and the router. Data in (C, L, junk M)
# and read back still go through ArbTestClients on AccessBankArb, and the L
# fetch through the real AutoLoadL, exactly as in cal_mat_e2e.py.
#
# Run from the gem5 root (pick a case from cal_test_data.CASES: u16,
# u16_multi, s32, big; default u16):
#   build/RISCV/gem5.opt --debug-flags=MatBank,Scheduler \
#       --outdir=m5out/cal_fe_u16 \
#       configs/learning_gem5/PIC/cal_frontend_e2e.py u16 > cal_fe_u16.log 2>&1
#   python3 configs/learning_gem5/PIC/check_cal_mat.py u16 cal_fe_u16.log
#
# All four cases:
#   for c in u16 u16_multi s32 big; do
#     build/RISCV/gem5.opt --debug-flags=MatBank,Scheduler \
#         --outdir=m5out/cal_fe_$c \
#         configs/learning_gem5/PIC/cal_frontend_e2e.py $c > cal_fe_$c.log 2>&1
#     python3 configs/learning_gem5/PIC/check_cal_mat.py $c cal_fe_$c.log
#   done
#
# MatBank is needed: without it the Mat does not print "dropped" for an
# access that came while it was busy, and check_cal_mat.py cannot catch
# one. 2>&1 is needed too: the reader prints the values it reads back with
# inform(), which goes to stderr. Scheduler is only there to read the log.
#
# To see that the EXE came through the frontend:
#   grep -E "EXE -> matID|EXE set-up" cal_fe_u16.log
# The fields the Mat got must match the EXE line in cal_test_input.txt.
#
# Files in the output directory:
#   cal_test_input.txt    -- the EXE, parameters, L and R, where the data was
#   cal_test_expected.csv -- every M word: address, value, lane -> (l, c)

import os
import sys

import m5
from m5.objects import *

# cal_test_data sits next to this config; gem5 does not always put the
# config's own directory on sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cal_test_data as ctd

case = ctd.Case(sys.argv[1] if len(sys.argv) > 1 else 'u16')

prefill = case.prefill()
expected = case.m_expected()
guards = case.guards()

# The frontend starts sending the EXE once every word is written; the Mat
# starts a few dozen cycles later, which the read-back margin covers.
COMPUTE_START = 10 * len(prefill) + 1000
READ_START = COMPUTE_START + 4 * case.compute_cycles() + 1000

# RTL SysConfig.get_bankID / get_in_bank_matID: matID = way * banks + bank.
TARGET_MAT_ID = ctd.MAT[0] * ctd.NUM_BANKS + ctd.MAT[1]

# The EXE, in ISA.c EXE() order (see PicTestFrontend.cal_cmds). R_Valid_nRols
# is the last C row (Controller: _C_array_EndPtr := _R_block_row).
exe_cmd = (f"{ctd.L_BASE:#x} {TARGET_MAT_ID} {case.r_block_row} "
           f"{case.n_buf} {case.n_cal} {case.r_base_bit} {case.l_precision} "
           f"{case.l_block_row} {int(case.signed_l)} "
           f"{int(case.signed_r_last_exist)} {int(case.acc_width == 32)} 1")

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

system.bank_arb = AccessBankArb(num_banks=ctd.NUM_BANKS)

# All PIC Mats, in AccessBankArb.mat_side order (same in-bank way, bank
# varies fastest). None runs a job until an EXE reaches it.
mats = []
for w in range(ctd.FIRST_PIC_WAY, ctd.NUM_WAYS):
    for b in range(ctd.NUM_BANKS):
        if (w, b) == ctd.MAT:
            mat = Mat(bank_index=b, way=w, wait_for_setup=True,
                      max_cycles=10 * case.compute_cycles())
            mat_under_test = mat
        else:
            mat = Mat(bank_index=b, way=w, wait_for_setup=True)
        mats.append(mat)
system.mats = mats
for mat in system.mats:
    system.bank_arb.mat_side = mat.array_side

# L path, as in the RTL: Mat -> AutoLoadL -> AccessBankArb (client 3).
system.autoload_l = AutoLoadL(system=system)
system.autoload_l.cb_port = system.bank_arb.autoload_side
system.autoload_l.inst_port = mat_under_test.load_l_port

# Command path: the frontend writes the EXE's SET registers like a CPU
# would; the Scheduler decodes them and sends the set-up out cb_port.
system.setup_router = MatSetUpRouter(mats=system.mats,
                                     num_banks=ctd.NUM_BANKS)
system.pic_transport = PicMmioTransport(request_gap=1,
                                        protocol_retry_delay=50)
system.test_frontend = PicTestFrontend(
    transport=system.pic_transport,
    trace="cal",
    cal_cmds=[exe_cmd],
    start_delay=COMPUTE_START,
    # The frontend is done once every SET is sent, not when the job is;
    # the read back below is what ends the run.
    exit_on_finish=False,
)
system.scheduler = Scheduler()
system.cmd_state_helper = CmdStateHelper()
system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.scheduler.cb_port = system.setup_router.cmd_side

# Data in before the job.
system.filler = ArbTestClient(
    kind='p2s', ops=[f"W {a:#x} {d:#x}" for a, d in prefill.items()])
system.filler.port = system.bank_arb.p2s_side[0]
system.idle_p2s = [ArbTestClient(kind='p2s', ops=[]) for _ in range(2)]
system.bank_arb.p2s_side = system.idle_p2s[0].port
system.bank_arb.p2s_side = system.idle_p2s[1].port

# Read back after the job. Every value is printed and checked by
# check_cal_mat.py, so one wrong word does not hide the others.
reads = list(expected) + [a for a in guards if a not in expected]
system.reader = ArbTestClient(
    kind='autoload', ops=[f"R {a:#x} *" for a in reads],
    start_delay=READ_START, exit_when_done=True)
system.reader.port = system.bank_arb.acc_side

# --------------------------------------------------------------------------
# Input / expected files
# --------------------------------------------------------------------------

outdir = m5.options.outdir
with open(os.path.join(outdir, 'cal_test_input.txt'), 'w') as f:
    f.write(f"case {case.name}: " + ", ".join(
        f"{k}={v}" for k, v in case.mat_params().items()) + "\n")
    f.write(f"EXE (ISA.c EXE() order): {exe_cmd}\n")
    f.write(f"Mat under test: way {ctd.MAT[0]} bank {ctd.MAT[1]} "
            f"(matID {TARGET_MAT_ID}, base {ctd.MAT_BASE:#x}); L words in "
            f"way {ctd.L_MAT[0]} bank {ctd.L_MAT[1]} (base {ctd.L_BASE:#x})\n")
    f.write(f"{case.rows} L rows x {case.cols} C rows = "
            f"{case.rows * case.cols} results in {case.m_words_needed()} "
            f"M words\n\n")
    f.write("L (raw bits, row l, element k = 0..63):\n")
    for l, row in enumerate(case.L):
        f.write(f"  L[{l}] = {row}\n")
    f.write("R (raw bytes, column c, element k = 0..63):\n")
    for c in range(case.cols):
        f.write(f"  R[:,{c}] = {[case.R[k][c] for k in range(64)]}\n")
    f.write("\nGolden M[l][c] (integer matrix product, before wrapping):\n")
    for l, row in enumerate(case.golden()):
        f.write(f"  M[{l}] = {row}\n")

with open(os.path.join(outdir, 'cal_test_expected.csv'), 'w') as f:
    f.write("array_addr,mat,bank,array,wordline,value,lanes\n")
    for a, lanes in case.m_layout():
        desc = " ".join(f"{j}:({l},{c})" if l is not None else f"{j}:stale"
                        for j, l, c in lanes)
        f.write(f"{a:#x},{','.join(map(str, ctd.split_addr(a)))},"
                f"{expected[a]:#x},{desc}\n")
    for a, v in guards.items():
        if a not in expected:
            f.write(f"{a:#x},{','.join(map(str, ctd.split_addr(a)))},"
                    f"{v:#x},guard\n")

# --------------------------------------------------------------------------
# Run
# --------------------------------------------------------------------------

m5.instantiate()
print(f"case {case.name}: {len(prefill)} words written before the job, "
      f"EXE sent @ cycle {COMPUTE_START}, read back @ cycle {READ_START}")
while True:
    exit_event = m5.simulate()
    cause = exit_event.getCause()
    print(f"Exiting @ tick {m5.curTick()} because {cause}")
    if cause != 'Mat finished':
        break
