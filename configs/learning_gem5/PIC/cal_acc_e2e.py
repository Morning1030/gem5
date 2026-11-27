# CAL on four Mats, then ACC over their results, sent as one program that
# never waits for a command to finish (only a fixed 'wait' before ACC).
#
#   PicTestFrontend (trace 'program') -> PicMmioTransport -> Scheduler
#     cal x4 -> cb_port  -> MatSetUpRouter -> Mat (way 1, bank b), b = 0..3
#     acc    -> acc_port -> Accumulator -> AccessBankArb (acc_side)
#
# The four Mats each compute one K slice of a bigger product: Mat b holds
# its own R slice (C data), fetches its own L slice (way 2, bank b) through
# the real AutoLoadL, and leaves its partial M in its sub-arrays
# 0..nBuf-1. ACC then adds the four partial M's lane by lane: ACC source s
# is SRC + s * 2048 (ISA.c ACC(): "the offset between src is 4 array"),
# which is one bank over, so source s is exactly Mat (way 1, bank s)'s M.
# The sums go to way 3, bank 0.
#
# The four EXEs go out back to back. MatSetUpRouter answers an EXE as soon
# as the Mat has its set-up, and the Scheduler then takes the next command,
# so all four Mats compute at the same time and their L fetches share
# AutoLoadL and the arbiter.
#
# Before ACC there is a 'wait'. It stands in for the waitEnqCmdFree() an
# RTL program calls there (ACC_test/main.c): ACC must not read a Mat that
# is still computing, because a busy Mat drops every array access
# (Mat.scala arrayCacheMode). gem5 cannot do the QUERY yet: nothing marks
# an EXE finished in CmdStateHelper.
#
# Run from the gem5 root (case from cal_test_data.CASES: u16, u16_multi,
# s32, big; default u16):
#   build/RISCV/gem5.opt --outdir=m5out/cal_acc_u16 \
#       configs/learning_gem5/PIC/cal_acc_e2e.py u16 > cal_acc_u16.log 2>&1
#
# All four cases:
#   for c in u16 u16_multi s32 big; do
#     build/RISCV/gem5.opt --outdir=m5out/cal_acc_$c \
#         configs/learning_gem5/PIC/cal_acc_e2e.py $c > cal_acc_$c.log 2>&1
#     tail -1 cal_acc_$c.log
#   done
#
# A second argument sets the wait in cycles; 0 drops it, so ACC runs
# while the Mats still compute and the check must fail:
#   build/RISCV/gem5.opt --debug-flags=MatBank --outdir=m5out/cal_acc_nowait \
#       configs/learning_gem5/PIC/cal_acc_e2e.py big 0
#
# Pass = the last line is "PASS ...". The reader checks every Mat's partial
# M, the C words and ACC's sums, and panics at the first wrong word
# (address, value read, value expected). Useful debug flags:
#   MatBank      per-Mat activity; "dropped" = an access hit a busy Mat
#   Scheduler    when each command is decoded and dispatched
#   Accumulator  ACC reads/writes
#   PICBankArb   arbiter queue ("queue N, max M")
#
# Files in the output directory:
#   cal_acc_input.txt     -- the program, where every Mat's data is
#   cal_acc_expected.csv  -- every word the reader checks

import os
import sys

import m5
from m5.objects import *

# The data modules sit next to this config; gem5 does not always put the
# config's own directory on sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import acc_test_data as atd
import cal_test_data as ctd

CASE = sys.argv[1] if len(sys.argv) > 1 else 'u16'

N_MATS = 4                  # ACC srcNum: Mats (CAL_WAY, bank 0..N_MATS-1)
CAL_WAY = 1
L_WAY = 2                   # Mat b's L words are in (L_WAY, bank b)
ACC_DST = ctd.array_addr(3, 0, 0, 0)
ACC_SOURCE_STRIDE = 512 * 4  # Accumulator wordline_nums * arrays_per_mat

# Same shape in every Mat (so the M words line up for ACC), own data each.
base_seed = ctd.CASES[CASE]['seed']
cases = [ctd.Case(CASE, mat=(CAL_WAY, b), l_mat=(L_WAY, b),
                  seed=base_seed * 16 + b)
         for b in range(N_MATS)]
c0 = cases[0]
acc_rows = c0.m_words_needed()
acc_src = c0.mat_base
for s, c in enumerate(cases):
    assert c.mat_base == acc_src + s * ACC_SOURCE_STRIDE, \
        f"ACC source {s} is not Mat {c.mat}'s M"

# ---- data in, expected out --------------------------------------------------

prefill = {}
for c in cases:
    words = c.prefill()
    assert not (words.keys() & prefill.keys()), "two Mats' data overlap"
    prefill.update(words)
acc_dsts = {ACC_DST + r for r in range(acc_rows + 1)}
assert not (acc_dsts & prefill.keys()), "ACC dst overlaps the CAL data"

# ACC row r = lane-wise sum (wrapping per lane) of word r of every M.
acc_expected = {}
for r in range(acc_rows):
    total = 0
    for c in cases:
        total = atd.lane_add(total, c.m_expected()[c.mat_base + r],
                             c.acc_width)
    acc_expected[ACC_DST + r] = total
acc_guard = {ACC_DST + acc_rows: 0}     # never written

# The reader checks each Mat's own M (a CAL bug shows up there first),
# its untouched words, then ACC's sums.
checks = []
for c in cases:
    checks += list(c.m_expected().items())
    checks += [(a, v) for a, v in c.guards().items()
               if a not in c.m_expected()]
checks += list(acc_expected.items()) + list(acc_guard.items())

# ---- timing (cycles) --------------------------------------------------------

# Every word is written before the first EXE (a busy Mat drops writes).
FRONTEND_START = 10 * len(prefill) + 1000
# One SET per cycle: EXE = SET_SRC, SET_DST, SET_PARAM.
EXE_SETS = 3 * N_MATS
# Long enough for every Mat, even with four Mats sharing AutoLoadL. A
# second argument overrides it (0 = no wait: ACC reads busy Mats and the
# check fails, which shows why the RTL program waits there).
WAIT = (int(sys.argv[2]) if len(sys.argv) > 2
        else 4 * c0.compute_cycles() + 500)
ACC_SEND = FRONTEND_START + EXE_SETS + WAIT     # ACC's SET_SRC leaves here
ACC_CYCLES = acc_rows * (N_MATS + 8) + 500
READ_START = ACC_SEND + ACC_CYCLES + 500

TICKS_PER_CYCLE = 1000      # 1GHz

# ---- program ----------------------------------------------------------------


def exe_fields(c):
    """ISA.c EXE() arguments without cmdID (see PicTestFrontend.cal_cmds)."""
    mat_id = c.mat[0] * ctd.NUM_BANKS + c.mat[1]
    return (f"{c.l_base:#x} {mat_id} {c.r_block_row} {c.n_buf} {c.n_cal} "
            f"{c.r_base_bit} {c.l_precision} {c.l_block_row} "
            f"{int(c.signed_l)} {int(c.signed_r_last_exist)} "
            f"{int(c.acc_width == 32)} 1")


program = [f"cal {exe_fields(c)}" for c in cases]
if WAIT:
    program.append(f"wait {WAIT}")
program.append(f"acc {acc_src:#x} {ACC_DST:#x} {N_MATS} {acc_rows} "
               f"{int(c0.acc_width == 32)}")

# ---- system -----------------------------------------------------------------

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

system.bank_arb = AccessBankArb(num_banks=ctd.NUM_BANKS)

# All PIC Mats, in AccessBankArb.mat_side order (way by way, bank fastest).
mats = []
cal_mats = []
for w in range(ctd.FIRST_PIC_WAY, ctd.NUM_WAYS):
    for b in range(ctd.NUM_BANKS):
        if w == CAL_WAY and b < N_MATS:
            mat = Mat(bank_index=b, way=w,
                      max_cycles=10 * c0.compute_cycles())
            cal_mats.append(mat)
        else:
            mat = Mat(bank_index=b, way=w)
        mats.append(mat)
system.mats = mats
for mat in system.mats:
    system.bank_arb.mat_side = mat.array_side

# L path, as in the RTL: Mat -> AutoLoadL -> AccessBankArb (client 3). One
# AutoLoadL serves all four Mats, round robin.
system.autoload_l = AutoLoadL(system=system)
system.autoload_l.cb_port = system.bank_arb.autoload_side
for mat in cal_mats:
    system.autoload_l.inst_port = mat.load_l_port

# Command path.
system.setup_router = MatSetUpRouter(mats=system.mats,
                                     num_banks=ctd.NUM_BANKS)
system.pic_transport = PicMmioTransport(request_gap=1,
                                        protocol_retry_delay=50)
system.test_frontend = PicTestFrontend(
    transport=system.pic_transport,
    trace="program",
    program_cmds=program,
    start_delay=FRONTEND_START,
    exit_on_finish=False,
)
system.scheduler = Scheduler()
system.cmd_state_helper = CmdStateHelper()
system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.scheduler.cb_port = system.setup_router.cmd_side

system.acc = Accumulator(
    system=system,
    wordline_nums=512,
    arrays_per_mat=4,
    exit_on_done=True,
)
system.scheduler.acc_port = system.acc.inst_port
system.acc.bank_port = system.bank_arb.acc_side

# Data in before the program, through the arbiter like a P2S engine.
system.filler = ArbTestClient(
    kind='p2s', ops=[f"W {a:#x} {d:#x}" for a, d in prefill.items()])
system.filler.port = system.bank_arb.p2s_side[0]
system.idle_p2s = [ArbTestClient(kind='p2s', ops=[]) for _ in range(2)]
system.bank_arb.p2s_side = system.idle_p2s[0].port
system.bank_arb.p2s_side = system.idle_p2s[1].port

# Read back after ACC, on the BankFetch client port (8-byte reads).
system.reader = ArbTestClient(
    kind='autoload', ops=[f"R {a:#x} {v:#x}" for a, v in checks],
    start_delay=READ_START, exit_when_done=True)
system.reader.port = system.bank_arb.bank_fetch_side

# ---- input / expected files -------------------------------------------------

outdir = m5.options.outdir
with open(os.path.join(outdir, 'cal_acc_input.txt'), 'w') as f:
    f.write(f"case {CASE}: " + ", ".join(
        f"{k}={v}" for k, v in c0.cal_params().items()
        if k != 'l_vec_fetch_addr') + "\n")
    f.write(f"{N_MATS} Mats x ({c0.rows} L rows x {c0.cols} C rows) -> "
            f"{acc_rows} M words each; ACC sums them into {ACC_DST:#x}\n\n")
    f.write("Program (frontend trace 'program', command IDs from 1):\n")
    for line in program:
        f.write(f"  {line}\n")
    f.write(f"\nCycles: data in from 1, program @ {FRONTEND_START}, "
            f"ACC sent @ {ACC_SEND}, read back @ {READ_START}\n\n")
    for c in cases:
        f.write(f"Mat (way {c.mat[0]}, bank {c.mat[1]}) base "
                f"{c.mat_base:#x}: L in (way {c.l_mat[0]}, bank "
                f"{c.l_mat[1]}) base {c.l_base:#x}, seed {c.seed}\n")
        for l, row in enumerate(c.golden()):
            f.write(f"  M[{l}] = {row}\n")

with open(os.path.join(outdir, 'cal_acc_expected.csv'), 'w') as f:
    f.write("array_addr,mat,bank,array,wordline,value,what\n")
    for c in cases:
        for a, v in c.m_expected().items():
            f.write(f"{a:#x},{','.join(map(str, ctd.split_addr(a)))},"
                    f"{v:#x},M of Mat {c.mat}\n")
    for a, v in acc_expected.items():
        f.write(f"{a:#x},{','.join(map(str, ctd.split_addr(a)))},"
                f"{v:#x},ACC sum\n")

# ---- run --------------------------------------------------------------------

m5.instantiate()
print(f"case {CASE}: {len(prefill)} words in; program @ cycle "
      f"{FRONTEND_START}: {N_MATS} EXEs back to back, wait {WAIT}, ACC "
      f"(sent @ cycle {ACC_SEND}); read back @ cycle {READ_START}")

mat_done = []
acc_done = None
while True:
    exit_event = m5.simulate()
    cause = exit_event.getCause()
    if cause == 'Mat finished':
        mat_done.append(m5.curTick())
        continue
    if cause == 'system.acc done':
        acc_done = m5.curTick()
        continue
    break

cyc = lambda t: t // TICKS_PER_CYCLE
print(f"Exiting @ tick {m5.curTick()} because {cause}")
print(f"Mats finished @ cycles {[cyc(t) for t in mat_done]}")
print(f"ACC done @ cycle {cyc(acc_done) if acc_done else None}")

assert cause == 'ArbTestClient done', cause
assert len(mat_done) == N_MATS, \
    f"{len(mat_done)} of {N_MATS} Mats finished"
assert max(mat_done) < ACC_SEND * TICKS_PER_CYCLE, (
    f"a Mat finished @ cycle {cyc(max(mat_done))}, after ACC was sent "
    f"(@ {ACC_SEND}); raise WAIT")
assert acc_done is not None and acc_done < READ_START * TICKS_PER_CYCLE, (
    f"ACC done @ {cyc(acc_done) if acc_done else None}, read back "
    f"started @ {READ_START}; raise ACC_CYCLES")
print(f"PASS: {N_MATS} CALs + ACC, {len(checks)} words checked")
