# LOAD into the cache banks, end to end:
#
#   LoadTestClient (stands in for the Scheduler's ld_port)
#     -> Load_ctl -> DMA_read_page_split (+ TLBReqScheduler) -> DRAM
#     -> ReadPostProcess -> AccessBankArb (client 0) -> real Mats
#
# Five LOADs exercise one-shot merging, strided rows, an unaligned DRAM
# address with odd row sizes, rows crossing a 4KB page, and a destination
# that runs from one bank into the next. When they are done, every word
# is read back through AccessBankArb.autoload_side by array address, and a
# wrong value panics. Words next to each LOAD's range that nothing writes
# must read 0.
#
# Where each word landed is checked separately by check_load_bank.py,
# from the Mats' own write log (debug flag MatBank: "bank B way W: write
# array A row R = V"). A Mat's bank and way are its own parameters, so
# that check does not depend on the arbiter's address decode.
#
# Run (both steps write into the output directory):
#   build/RISCV/gem5.opt --debug-flags=MatBank,PICLoadCtl,PICDMA,PICReadPost \
#       --debug-file=trace.txt --outdir=m5out/load_bank_e2e \
#       configs/learning_gem5/PIC/load_bank_e2e.py
#   python3 configs/learning_gem5/PIC/check_load_bank.py m5out/load_bank_e2e
#
# Files:
#   load_test_input.txt    -- the LOADs, the DRAM pattern, every expected word
#   load_test_expected.csv -- the expected words, one per line
#   load_test_result.txt   -- readback result (written only if all matched)
#   load_test_location.txt -- check_load_bank.py's result
#   trace.txt              -- debug trace, including every Mat write
# simerr.txt has each LOAD's latency and every read back ("... OK").

import os
import sys

import m5
from m5.objects import *

sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', '..', 'src', 'learning_gem5', 'PIC'),
)
from pic_cache import PICCache

NUM_BANKS = 4
FIRST_PIC_MAT = 1
READ_START = 300000   # cycles; readers start after every LOAD is done


def array_addr(mat, bank, array, row):
    # [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | wordline(9) ]
    return (mat << 13) | (bank << 11) | (array << 9) | row


def split_addr(a):
    return (a >> 13) & 0xF, (a >> 11) & 0x3, (a >> 9) & 0x3, a & 0x1FF


def dram_byte(a):
    """DRAM test pattern: one byte per address, every byte different from
    its neighbours, so a shifted or dropped byte shows up."""
    return ((a * 0x9E) ^ (a >> 7) ^ 0x5A) & 0xFF


# (name, src vaddr, dst array address, row, bytePerRow, offset)
CASES = [
    ("A contiguous rows (one_shot)",
     0x80100000, array_addr(1, 0, 0, 0), 4, 64, 64),
    ("B strided rows",
     0x80110000, array_addr(2, 1, 1, 10), 8, 16, 100),
    ("C unaligned src, 13 B rows (bytes carry across rows)",
     0x80120003, array_addr(5, 2, 0, 505), 5, 13, 40),
    ("D each row crosses a 4KB page",
     0x80130FF0, array_addr(15, 3, 3, 0), 3, 48, 4096),
    ("E 512 B row over a page, dst runs into the next bank",
     0x80140F00, array_addr(3, 0, 3, 508), 1, 512, 512),
]


def expected_words(src, dst, row, bpr, off):
    """What ReadPostProcess should write: the rows' bytes back to back
    (the Aligner only flushes at the end of the whole LOAD), packed into
    little-endian 64-bit words at dst, dst + 1, ...; a short last word is
    zero-padded."""
    data = bytearray()
    for r in range(row):
        base = src + r * off
        data += bytes(dram_byte(base + i) for i in range(bpr))
    data += bytes(-len(data) % 8)
    return [(dst + k, int.from_bytes(data[8 * k:8 * k + 8], 'little'))
            for k in range(len(data) // 8)]


expected = []   # (case index, array address, value)
for ci, (_, src, dst, row, bpr, off) in enumerate(CASES):
    expected += [(ci, a, v) for a, v in expected_words(src, dst, row, bpr,
                                                       off)]
written = {a: v for _, a, v in expected}
assert len(written) == len(expected), "two cases write the same word"

# Neighbours of each case's range that nothing writes must stay 0.
guards = []
for ci, (_, src, dst, row, bpr, off) in enumerate(CASES):
    n = sum(1 for c, _, _ in expected if c == ci)
    for a in (dst - 1, dst + n):
        if a not in written and split_addr(a)[0] >= FIRST_PIC_MAT:
            guards.append(a)

# --------------------------------------------------------------------------
# System
# --------------------------------------------------------------------------

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

# memory system (DRAM, bus, system port for DMA patterns)
system.membus = SystemXBar()
system.memory = SimpleMemory(range=system.mem_ranges[0], latency='10ns')
system.memory.port = system.membus.mem_side_ports
system.system_port = system.membus.cpu_side_ports

# scheduler
system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)

# Load path, one SimObject per RTL module.
system.load_ctl = Load_ctl()
system.dma = DMA_read_page_split(chunk_bytes=64, block_bytes=64,
                                 max_outstanding=4)
system.tlb_sched = TLBReqScheduler()
system.read_post = ReadPostProcess()

system.load.cmd_side = system.scheduler.ld_port
system.load_ctl.dma_side = system.dma.req_side
system.load_ctl.post_side = system.read_post.req_side
system.dma.data_side = system.read_post.data_side
system.dma.tlb_side = system.tlb_sched.cpu_side
system.dma.mem_side = system.membus.cpu_side_ports
system.read_post.buf_side = system.load_ctl.buf_side

# Cache banks: PICCache's 60 PIC Mats behind the arbiter. Mats never start
# their own cal() job here, they are only storage.
cache = PICCache(system, pic_way_mask=0xFFFE, start_cycles=10**9)
system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)
for mat in cache.mats:
    system.bank_arb.mat_side = mat.array_side 
system.read_post.bank_side = system.bank_arb.load_post_side

# The arbiter needs its three P2S ports connected; nothing writes on them.
system.idle_p2s = [ArbTestClient(kind='p2s', ops=[]) for _ in range(3)]
for i, c in enumerate(system.idle_p2s):
    c.port = system.bank_arb.p2s_side[i]

system.loader = LoadTestClient(
    loads=[f"{src:#x} {dst:#x} {row} {bpr} {off}"
           for _, src, dst, row, bpr, off in CASES],
    exit_when_done=True,
)
system.loader.port = system.load_ctl.cmd_side

# Read back through the arbiter, by array address.
system.arb_reader = ArbTestClient(
    kind='autoload',
    ops=[f"R {a:#x} {v:#x}" for _, a, v in expected] +
        [f"R {a:#x} 0x0" for a in guards],
    start_delay=READ_START, exit_when_done=True, print_reads=True,
)
system.arb_reader.port = system.bank_arb.autoload_side

# --------------------------------------------------------------------------
# Input / expected files
# --------------------------------------------------------------------------

outdir = m5.options.outdir
with open(os.path.join(outdir, 'load_test_input.txt'), 'w') as f:
    f.write("DRAM pattern: byte at address a = "
            "((a * 0x9E) ^ (a >> 7) ^ 0x5A) & 0xFF\n")
    f.write("Array address = [matID(4) | bank(2) | array(2) | wordline(9)]"
            ", one 64-bit word each\n\n")
    for ci, (name, src, dst, row, bpr, off) in enumerate(CASES):
        words = [(a, v) for c, a, v in expected if c == ci]
        f.write(f"LOAD {ci}: {name}\n")
        f.write(f"  src {src:#x}  dst {dst:#x}  row {row}  bytePerRow "
                f"{bpr}  offset {off}  -> {row * bpr} B, {len(words)} "
                f"words\n")
        for r in range(row):
            b = src + r * off
            f.write(f"  row {r}: DRAM [{b:#x}, {b + bpr:#x})  bytes "
                    + " ".join(f"{dram_byte(b + i):02x}" for i in range(bpr))
                    + "\n")
        for a, v in words:
            mat, bank, array, row_ = split_addr(a)
            f.write(f"  word @{a:#07x} (mat {mat:2d} bank {bank} array "
                    f"{array} wordline {row_:3d}) = {v:#018x}\n")
        f.write("\n")
    f.write("Guard words (must stay 0): "
            + " ".join(f"{a:#x}" for a in guards) + "\n")

with open(os.path.join(outdir, 'load_test_expected.csv'), 'w') as f:
    f.write("load,array_addr,mat,bank,array,wordline,value\n")
    for ci, a, v in expected:
        f.write(f"{ci},{a:#x},{','.join(map(str, split_addr(a)))},{v:#x}\n")
    for a in guards:
        f.write(f"guard,{a:#x},{','.join(map(str, split_addr(a)))},0x0\n")

# --------------------------------------------------------------------------
# Run
# --------------------------------------------------------------------------

m5.instantiate()

# Fill each LOAD's whole DRAM span, gaps between rows included, so reading
# a gap byte by mistake gives a wrong value instead of 0.
for _, src, dst, row, bpr, off in CASES:
    end = src + (row - 1) * off + bpr
    root.system.physProxy.write(
        src, bytearray(dram_byte(a) for a in range(src, end)))

print(f"{len(CASES)} LOADs, {len(expected)} words, {len(guards)} guards")

ev = m5.simulate()
assert ev.getCause() == "LoadTestClient done", ev.getCause()
loads_done = m5.curTick()
read_start = READ_START * 1000   # 1GHz: 1000 ticks per cycle
assert loads_done < read_start, (
    f"LOADs took until tick {loads_done}; raise READ_START")
print(f"All LOADs done @ tick {loads_done}")

ev = m5.simulate()
assert ev.getCause() == "ArbTestClient done", ev.getCause()

total_reads = len(expected) + len(guards)
with open(os.path.join(outdir, 'load_test_result.txt'), 'w') as f:
    f.write(f"PASS: {len(CASES)} LOADs, all LOADs done @ tick "
            f"{loads_done}\n")
    f.write(f"{len(expected)} words + {len(guards)} guards read back "
            f"through the arbiter: {total_reads} reads, all matched\n")
print(f"PASS: {total_reads} reads matched @ tick {m5.curTick()}")
