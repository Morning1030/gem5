# ACC end-to-end on real Mats: Scheduler -> Accumulator -> AccessBankArb
# (acc_side) -> PICCache Mats. The "acc" trace in pic_test_frontend.cc runs
# one ACC command: src 0x3000, dst 0x4000, 2 sources, 64 rows.
#
# The sources are written into the Mats before ACC starts by an
# ArbTestClient filler on p2s_side[0] (data: acc_test_data.py); the
# frontend's start_delay keeps ACC behind it. The run ends when ACC
# reports done. Check with --debug-flags=Accumulator,PICBankArb,MatBank and
# check_acc_mat.py.

import argparse
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

# acc_test_data sits next to this config; gem5 does not always put the
# config's own directory on sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import acc_test_data

parser = argparse.ArgumentParser(description="PolymorPIC ACC PIC cache E2E")
parser.add_argument("--width", type=int, choices=[16, 32], default=32,
                    help="ACC lane width: 16 or 32 bits")
args = parser.parse_args()

# Cycles before the frontend sends the ACC command. The filler writes 128
# words, a few cycles each through the arbiter; check_acc_mat.py confirms
# its last write lands before ACC's first read.
FRONTEND_START = 2000


def W(a, d):
    return f"W {a:#x} {d:#x}"


system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'

system.pic_transport = PicMmioTransport(
    system=system,
    request_gap=1,
    protocol_retry_delay=50,
)

system.test_frontend = PicTestFrontend(
    transport=system.pic_transport,
    start_delay=FRONTEND_START,
    inter_command_gap=1,
    exit_on_finish=False,
    trace="acc",
    acc_bit_width_code=1 if args.width == 32 else 0,
)

system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)

system.acc = Accumulator(
    system=system,
    wordline_nums=512,
    arrays_per_mat=4,
    exit_on_done=True,
)

# Mats are storage only: start_cycles keeps them from starting their own
# cal() job, which would drop every external write. Way 0 is the normal
# cache way; all other ways are in PIC mode.
cache = PICCache(system, pic_way_mask=0xFFFE, start_cycles=10**9)

system.bank_arb = AccessBankArb(num_banks=4)
for mat in cache.mats:
    system.bank_arb.mat_side = mat.array_side

# Preload the ACC sources through the arbiter, like a P2S engine would.
system.filler = ArbTestClient(
    kind='p2s', ops=[W(a, d) for a, d in acc_test_data.preload()])
system.filler.port = system.bank_arb.p2s_side[0]

# p2s_side needs exactly 3 connections; the other two are idle.
system.idle1 = ArbTestClient(kind='p2s', ops=[])
system.idle1.port = system.bank_arb.p2s_side[1]
system.idle2 = ArbTestClient(kind='p2s', ops=[])
system.idle2.port = system.bank_arb.p2s_side[2]

system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.acc_port = system.acc.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.acc.bank_port = system.bank_arb.acc_side

m5.instantiate()

print('Starting ACC {}-bit PIC cache E2E: {} source words preloaded'
      .format(args.width, len(acc_test_data.preload())))

exit_event = m5.simulate()

print(
    'Exiting @ tick {} because {}'.format(
        m5.curTick(),
        exit_event.getCause()
    )
)
