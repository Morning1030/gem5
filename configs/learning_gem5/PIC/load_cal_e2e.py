import os
import sys

import m5
from m5.objects import *
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

# test case front-end
system.pic_transport = PicMmioTransport(
    system=system,
    request_gap=1,
    protocol_retry_delay=50,
)

system.test_frontend = PicTestFrontend(
    transport=system.pic_transport,
    start_delay=1,
    inter_command_gap=1,
    exit_on_finish=False,
    trace="store",
    acc_bit_width_code=1
)

# scheduler
system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)

system.acc = Accumulator(
    system=system,
    wordline_nums=512,
    arrays_per_mat=4,
    exit_on_done=False,
)

# Store path, one SimObject per RTL module.
system.store_ctl = StoreCtrl(system=system)
system.bank_fetch = BankFetch(system=system)
system.dma = DMAWritePageSplit(system=system)
system.tlb_sched = TLBReqScheduler()
cache = PICCache(system, pic_way_mask=0xFFFE, start_cycles=10**9)
system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)
system.autoloadl = AutoLoadL(system=system)


system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.acc_port = system.acc.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.scheduler.st_port = system.store_ctl.inst_port
system.acc.bank_port = system.bank_arb.acc_side
system.store_ctl.bank_fetch_port = system.bank_fetch.inst_port
system.bank_fetch.dma_port = system.dma.req_side

system.dma.tlb_side = system.tlb_sched.cpu_side
system.dma.mem_side = system.membus.cpu_side_ports

# Cache banks: PICCache's 60 PIC Mats behind the arbiter. Mats never start
# their own cal() job here, they are only storage.
system.bank_fetch.cb_port = system.bank_arb.bank_fetch_side
for mat in cache.mats:
    system.bank_arb.mat_side = mat.array_side
    system.autoloadl.inst_port =  mat.load_l_port

system.autoloadl.cb_port = system.bank_arb.autoload_side

system.dummy_loadpost = PyTrafficGen()
system.dummy_loadpost.port = system.bank_arb.load_post_side

# The arbiter needs its three P2S ports connected; nothing writes on them.
system.idle_p2s = [PyTrafficGen() for _ in range(3)]
for i, c in enumerate(system.idle_p2s):
    c.port = system.bank_arb.p2s_side[i]


m5.instantiate()
exit_event = m5.simulate()

print(
    'Exiting @ tick {} because {}'.format(
        m5.curTick(),
        exit_event.getCause()
    )
)

# export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | grep -v condabin | paste -sd: -)
# which python3-config   # 确认指向 /usr/bin/python3-config
# rm -rf build/ALL/gem5.build
# scons build/ALL/gem5.opt -j$(nproc)
# PYTHONPATH=src/learning_gem5/PIC build/ALL/gem5.opt \
#   --debug-flags=Scheduler \
#   configs/learning_gem5/PIC/switch_load_cal_e2e.py