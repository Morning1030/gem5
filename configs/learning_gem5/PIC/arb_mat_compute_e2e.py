# One Mat runs a cal() job while clients read and write its sub-arrays
# through AccessBankArb at the same time. Follows the RTL (Mat.scala):
#   - the M-array is not separate storage, it is the Mem-mode sub-arrays;
#     with n_buf=1 that is sub-array 0, and C data lives in sub-arrays 1..3
#   - while a job runs, accesses from the arbiter are ignored (writes are
#     lost), so they cannot change the result
#
#   gem5.opt arb_mat_compute_e2e.py baseline    fill C data, then compute
#   gem5.opt arb_mat_compute_e2e.py unrelated   + writes during compute to
#                                                 rows compute does not read
#   gem5.opt arb_mat_compute_e2e.py overlap     + writes during compute to a
#                                                 row compute reads
#   gem5.opt arb_mat_compute_e2e.py after       the same writes, but only
#                                                 after compute finished
#
# baseline, unrelated and overlap must give the same checksum and the same
# M results, and the dropped rows must read back unchanged. In "after" the
# same writes are accepted and must read back.
# After compute, the reader reads the M results back through the arbiter
# (the way the store path does) and the rows that were written.
# The L vector comes from a MatSink (all ones) through the real AutoLoadL.

import sys

import m5
from m5.objects import *

mode = sys.argv[1] if len(sys.argv) > 1 else 'baseline'
assert mode in ('baseline', 'unrelated', 'overlap', 'after')

NUM_BANKS = 4
NUM_WAYS = 16
FIRST_PIC_WAY = 1
R_BLOCK_ROW = 8          # compute reads C-array rows 0..R_BLOCK_ROW
COMPUTE_START = 2000     # cycles
M_ROWS = 12              # M results to read back (sub-array 0)
TRAFFIC_OFFSET = int(sys.argv[2]) if len(sys.argv) > 2 else 7


def addr(mat, bank, array, row):
    # [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | wordline(9) ]
    return (mat << 13) | (bank << 11) | (array << 9) | row


def W(a, d):
    return f"W {a:#x} {d:#x}"


def R(a, d):
    return f"R {a:#x} {d:#x}"


system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)

# The Mat under test is bank 0, way 1 (matID 1). All other Mats are storage.
system.mats = [
    Mat(bank_index=b, way=w,
        start_cycles=COMPUTE_START if (b, w) == (0, 1) else 10**9,
        n_buf=1, n_cal=3, acc_width=16,
        r_block_row=R_BLOCK_ROW, l_block_row=1, l_precision=0)
    for w in range(FIRST_PIC_WAY, NUM_WAYS)
    for b in range(NUM_BANKS)
]
for mat in system.mats:
    system.bank_arb.mat_side = mat.array_side

# L vector path: Mat -> real AutoLoadL -> MatSink (all ones).
system.l_mem = MatSink(read_default=0xFFFFFFFFFFFFFFFF)
system.autoload_l = AutoLoadL(system=system)
system.autoload_l.cb_port = system.l_mem.arr_side
system.autoload_l.inst_port = system.mats[0].load_l_port

# Fill the C data (sub-arrays 1..3) of the Mat under test before it starts.
fill = []
for arr in range(1, 4):
    for row in range(R_BLOCK_ROW + 1):
        fill.append((addr(1, 0, arr, row), 0x0101010101010101 * (arr + 1)
                     + row))
system.filler = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in fill])
system.filler.port = system.bank_arb.p2s_side[0]

# Traffic while the Mat computes. The job runs ~18 cycles from
# COMPUTE_START; the first write arrives a few cycles into it.
during = []
if mode == 'unrelated':
    # rows 300.. are never read by compute (it reads 0..R_BLOCK_ROW)
    # (5 writes, all inside the ~18 cycle compute window)
    during = [(addr(1, 0, 1, 300 + i), 0xD000 + i) for i in range(5)]
elif mode in ('overlap', 'after'):
    during = [(addr(1, 0, a, 3), 0xBAD0BAD0BAD0BAD0 + a) for a in range(4)]
traffic_delay = COMPUTE_START + TRAFFIC_OFFSET
if mode == 'after':
    traffic_delay = COMPUTE_START + 100   # compute is long finished
system.traffic = ArbTestClient(
    kind='p2s', ops=[W(a, d) for a, d in during],
    start_delay=traffic_delay)
system.traffic.port = system.bank_arb.p2s_side[1]

# p2s_side needs exactly 3 connections; the third is idle.
system.idle = ArbTestClient(kind='p2s', ops=[])
system.idle.port = system.bank_arb.p2s_side[2]

# Read back after compute: the C data, the M results (sub-array 0), and the
# rows that were written during or after compute.
# M results of the baseline run (sub-array 0, row -> value); the other rows
# of the first M_ROWS stay zero. Regression values from this model, not from
# the RTL.
GOLD = {addr(1, 0, 0, r): v for r, v in enumerate(
    [0x4c004b0048, 0x51004c004c004b, 0x51004c004c004f])}
expect = dict((a, d) for a, d in fill)
for a, d in during:
    if mode == 'after':
        expect[a] = d            # accepted
    # else dropped: the row keeps what it had
reads = [R(a, d) for a, d in expect.items()]
for r in range(M_ROWS):
    a = addr(1, 0, 0, r)
    if a in GOLD:
        reads.append(R(a, GOLD[a]))
    else:
        reads.append(f"R {a:#x} *")
for a, d in during:
    if a not in expect and mode != 'after':
        # never written, never computed: still zero
        reads.append(R(a, 0)) if (a >> 9) & 3 != 0 or (a & 511) >= M_ROWS \
            else None
system.reader = ArbTestClient(
    kind='autoload', ops=[x for x in reads if x],
    start_delay=COMPUTE_START * 20, exit_when_done=True)
system.reader.port = system.bank_arb.autoload_side

m5.instantiate()
print(f"mode={mode}: {len(fill)} fills, {len(during)} writes during compute")
while True:
    exit_event = m5.simulate()
    cause = exit_event.getCause()
    print(f"Exit @ tick {m5.curTick()}: {cause}")
    if cause != 'Mat finished':
        break
