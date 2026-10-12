# AccessBankArb -> real Mats. Three P2S-style clients write 64-bit words
# into Mat sub-arrays through the arbiter, then an AutoLoadL-style client
# reads them back and checks the values. Reads of addresses nobody wrote
# must return 0 (checks that a write did not land in the wrong Mat).
#
# Built with PICCache (tags + arbiter + 60 Mats); the clients are test
# clients connected from outside. No AutoLoadL, Scheduler or DMA.

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

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

NUM_BANKS = 4
NUM_WAYS = 16
FIRST_PIC_WAY = 1


def addr(mat, bank, array, row):
    # [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | wordline(9) ]
    return (mat << 13) | (bank << 11) | (array << 9) | row


def W(a, d):
    return f"W {a:#x} {d:#x}"


def R(a, d):
    return f"R {a:#x} {d:#x}"


# (address, value); a later entry for the same address overwrites.
writes0 = [
    (addr(1, 0, 0, 5), 0x1111111111111111),
    (addr(3, 2, 3, 100), 0x3333333333333333),
    (addr(1, 0, 0, 5), 0xAAAAAAAAAAAAAAAA),  # overwrite
]
writes1 = [
    (addr(15, 3, 2, 511), 0xFFFFFFFFFFFFFFFE),
    (addr(7, 1, 1, 0), 0x7777777777777777),
] + [(addr(4, 0, 0, r), 0x0100 + r) for r in range(16)]
writes2 = [
    (addr(2, 0, 0, 5), 0x2222222222222222),
    (addr(1, 0, 1, 5), 0x5555555555555555),  # same Mat, other sub-array
]

final = {}
for w in (writes0, writes1, writes2):
    for a, d in w:
        final[a] = d

unwritten = [
    addr(1, 1, 0, 5),   # same matID/array/row, other bank
    addr(2, 1, 0, 5),
    addr(1, 0, 2, 5),   # same Mat, untouched sub-array
]
reads = [R(a, d) for a, d in final.items()] + [R(a, 0) for a in unwritten]

# Mats never start their own cal() job here, they are only storage.
# All ways but the normal cache way are in PIC mode (the PIC channel needs it).
cache = PICCache(system, pic_way_mask=0xFFFE, start_cycles=10**9)
arb = system.access_bank_arb

system.writer0 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes0])
system.writer1 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes1])
system.writer2 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes2])
system.writer0.port = arb.p2s_side[0]
system.writer1.port = arb.p2s_side[1]
system.writer2.port = arb.p2s_side[2]

system.reader = ArbTestClient(
    kind='autoload', ops=reads, start_delay=500, exit_when_done=True
)
system.reader.port = arb.autoload_side

m5.instantiate()
print(f"{len(writes0) + len(writes1) + len(writes2)} writes, "
      f"{len(reads)} reads")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
