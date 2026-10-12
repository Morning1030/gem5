# AccessBankArb Block state. MatSink turns away the first request on every
# Mat port and sends a retry 20 cycles later, so the arbiter has to hold
# the packet, mark that bank not ready, freeze other clients that need it,
# and resend on retry. Data written through this must still read back
# correctly (MatSink keeps the data per Mat port).
#
# Run with --debug-flags=PICBankArb to see the refusals, the "bank busy"
# grants and the retries.

import m5
from m5.objects import *

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


# writer0 and writer1 hit the same bank (0), different Mats. writer2 hits
# bank 1, so it should only be held up while the arbiter is in Block.
writes0 = [(addr(1, 0, 0, 0), 0xA0A0A0A0A0A0A0A0),
           (addr(1, 0, 0, 1), 0xA1A1A1A1A1A1A1A1)]
writes1 = [(addr(2, 0, 0, 0), 0xB0B0B0B0B0B0B0B0),
           (addr(2, 0, 0, 1), 0xB1B1B1B1B1B1B1B1)]
writes2 = [(addr(1, 1, 0, 0), 0xC0C0C0C0C0C0C0C0),
           (addr(1, 1, 0, 1), 0xC1C1C1C1C1C1C1C1)]

final = {}
for w in (writes0, writes1, writes2):
    for a, d in w:
        final[a] = d
reads = [R(a, d) for a, d in final.items()]
reads.append(R(addr(3, 0, 0, 0), 0))   # never written, also refused once

system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)

system.writer0 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes0])
system.writer1 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes1])
system.writer2 = ArbTestClient(kind='p2s', ops=[W(a, d) for a, d in writes2])
system.writer0.port = system.bank_arb.p2s_side[0]
system.writer1.port = system.bank_arb.p2s_side[1]
system.writer2.port = system.bank_arb.p2s_side[2]

system.reader = ArbTestClient(
    kind='autoload', ops=reads, start_delay=1000, exit_when_done=True
)
system.reader.port = system.bank_arb.autoload_side

system.mat_sink = MatSink(refuse_first=True, retry_delay=20)
for _ in range(NUM_BANKS * (NUM_WAYS - FIRST_PIC_WAY)):
    system.bank_arb.mat_side = system.mat_sink.arr_side

m5.instantiate()
print(f"{len(final)} words written, {len(reads)} reads")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
