# One Mat runs one cal() job; its M results are checked against an integer
# matrix product (cal_test_data.py has the golden model and the data
# layout). Everything goes through AccessBankArb, as in the RTL:
#   - before the job, the C data (Mat under test), the L words (another
#     Mat) and junk M words are written through p2s_side(filler,arb_test_client)
#   - during the job, the Mat fetches L words through the real AutoLoadL ->
#     AccessBankArb.autoload_side -> the Mat that holds them
#   - after the job, every M word and every guard word (C data, the M word
#     after the last result) is read back through acc_side
# The Mat is started by its SimObject parameters (start_cycles), not by the
# Scheduler.
#
# Run (pick a case from cal_test_data.CASES; default u16):
#   build/RISCV/gem5.opt --debug-flags=MatBank --outdir=m5out/cal_u16 \
#       configs/learning_gem5/PIC/cal_mat_e2e.py u16 > cal_u16.log 2>&1
#   python3 configs/learning_gem5/PIC/check_cal_mat.py u16 cal_u16.log
#
# Files in the output directory:
#   cal_test_input.txt    -- parameters, L and R, where the data was put
#   cal_test_expected.csv -- every M word: address, value, lane -> (l, c)

import os
import sys

import m5
from m5.objects import *

# gem5 does not always put the config's own directory on sys.path.
# adding this so that we can import ctd successfully
###################################################################
#_file_: relative path to this config
# os.path.abspath(__file__): turn relative path to absolute path
# os.path.dirname(): leave the folder(path) not the file(path)
# insert(addr, element) 
###################################################################
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cal_test_data as ctd

# sys.argv == ['configs/learning_gem5/PIC/cal_mat_e2e.py', 'u16']
# decide the whole parameter set (acc_width, signedness, n_buf...) according to the case name (u16(default), u16_multi, s32, big)
case = ctd.Case(sys.argv[1] if len(sys.argv) > 1 else 'u16')

# 1. decide the addr read back (L94)
# 2. produce cal_test_expected.cvs
# 3. (check_cal_mat.py will use the same case to cal m_expected and guards )
prefill = case.prefill() # C data, L words, junk M words
expected = case.m_expected() #golden answer of M words after cal
guards = case.guards() # make sure Cdata, and M words after the last result is not modified

# Each write/read through the arbiter takes a few cycles.
COMPUTE_START = 10 * len(prefill) + 1000 #U16: 1350 cycles
READ_START = COMPUTE_START + 4 * case.compute_cycles() + 1000 # 3350 cycles

# SE mode setting
system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

# Arbiter setting
system.bank_arb = AccessBankArb(num_banks=ctd.NUM_BANKS)

# Mat setting (build object by the type of Mat.py )
# All PIC Mats, in AccessBankArb.mat_side order (same in-bank way, bank varies fastest). 
#Only the Mat under test starts a job.
mats = []
for w in range(ctd.FIRST_PIC_WAY, ctd.NUM_WAYS):
    for b in range(ctd.NUM_BANKS):
        if (w, b) == ctd.MAT:
            mat = Mat(bank_index=b, way=w, start_cycles=COMPUTE_START,
                      max_cycles=10 * case.compute_cycles(),
                      **case.mat_params())
            mat_under_test = mat
        else:
            mat = Mat(bank_index=b, way=w, start_cycles=10**9)
        mats.append(mat)
system.mats = mats
for mat in system.mats:
    system.bank_arb.mat_side = mat.array_side

# L path, as in the RTL: Mat -> AutoLoadL -> AccessBankArb (client 3).
system.autoload_l = AutoLoadL(system=system)
system.autoload_l.cb_port = system.bank_arb.autoload_side
system.autoload_l.inst_port = mat_under_test.load_l_port

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
    f.write(f"Mat under test: way {ctd.MAT[0]} bank {ctd.MAT[1]} "
            f"(base {ctd.MAT_BASE:#x}); L words in way {ctd.L_MAT[0]} "
            f"bank {ctd.L_MAT[1]} (base {ctd.L_BASE:#x})\n")
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
      f"compute @ cycle {COMPUTE_START}, read back @ cycle {READ_START}")
while True:
    exit_event = m5.simulate()
    cause = exit_event.getCause()
    print(f"Exiting @ tick {m5.curTick()} because {cause}")
    if cause != 'Mat finished':
        break
