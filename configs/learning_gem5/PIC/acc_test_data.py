# ACC test data for acc_pic_e2e.py, shared with check_acc_mat.py: the
# source words preloaded into the Mats before ACC starts, and the sums ACC
# must write back. Matches the "acc" trace in pic_test_frontend.cc:
# src 0x3000, dst 0x4000, 2 sources, 64 rows.
#
# Addresses are 17-bit array addresses (wordline units):
#   [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | wordline(9) ]
# Source s, row r is read from SRC + s * SOURCE_STRIDE + r, as
# Accumulator::sourceAddress() computes it; the sum goes to DST + r.
#
# No m5 import: check_acc_mat.py loads this outside gem5.

import random

SRC = 0x3000
DST = 0x4000
NUM_SOURCES = 2
ROWS = 64
# Accumulator wordline_nums * arrays_per_mat
SOURCE_STRIDE = 512 * 4
SEEDS = (0xACC0, 0xACC1)

ADDR_BITS = 17
MAT_SHIFT = 13      # matID_in_bank starts at bit 13
FIRST_PIC_MAT = 1   # mat 0 is the normal cache way

# At least this fraction of lanes must overflow when the sources are summed.
MIN_OVERFLOW_FRACTION = 0.25

LANE_BITS = {16: 16, 32: 32}

_rngs = [random.Random(seed) for seed in SEEDS]
# SOURCES[s][r]: 64-bit word of source s, row r.
SOURCES = [[rng.getrandbits(64) for _ in range(ROWS)] for rng in _rngs]


def source_addr(s, row):
    return SRC + s * SOURCE_STRIDE + row


def dst_addr(row):
    return DST + row


def lane_add(acc, data, width):
    """Lane-wise add, each lane wrapping on its own (Accumulator add16 /
    add32)."""
    bits = LANE_BITS[width]
    mask = (1 << bits) - 1
    out = 0
    for shift in range(0, 64, bits):
        out |= (((acc >> shift) & mask) + ((data >> shift) & mask)
                & mask) << shift
    return out


def expected_sum(row, width):
    acc = 0
    for s in range(NUM_SOURCES):
        acc = lane_add(acc, SOURCES[s][row], width)
    return acc


def overflowing_lanes(width):
    """(lanes whose true sum does not fit, total lanes) over all rows."""
    bits = LANE_BITS[width]
    mask = (1 << bits) - 1
    over = 0
    for row in range(ROWS):
        for shift in range(0, 64, bits):
            total = sum((SOURCES[s][row] >> shift) & mask
                        for s in range(NUM_SOURCES))
            over += total > mask
    return over, ROWS * (64 // bits)


def preload():
    """[(addr, word)] to write into the Mats before ACC starts."""
    check()
    return [(source_addr(s, row), SOURCES[s][row])
            for s in range(NUM_SOURCES) for row in range(ROWS)]


def check():
    """Raise if sources and dst overlap, leave the 17-bit address space,
    touch mat 0, or overflow too few lanes."""
    src = {source_addr(s, r) for s in range(NUM_SOURCES) for r in range(ROWS)}
    dst = {dst_addr(r) for r in range(ROWS)}
    if len(src) != NUM_SOURCES * ROWS:
        raise ValueError("source rows overlap each other")
    if src & dst:
        raise ValueError(f"sources and dst overlap at "
                         f"{[hex(a) for a in sorted(src & dst)[:4]]}")
    for a in src | dst:
        if a >> ADDR_BITS:
            raise ValueError(f"{a:#x} is wider than {ADDR_BITS} bits")
        if (a >> MAT_SHIFT) < FIRST_PIC_MAT:
            raise ValueError(f"{a:#x} is in mat {a >> MAT_SHIFT}, a normal "
                             f"cache way")
    for width in LANE_BITS:
        over, total = overflowing_lanes(width)
        if over < MIN_OVERFLOW_FRACTION * total:
            raise ValueError(f"only {over}/{total} {width}-bit lanes "
                             f"overflow, need {MIN_OVERFLOW_FRACTION:.0%}")
