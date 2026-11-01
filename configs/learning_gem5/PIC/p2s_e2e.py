# P2S L / R / RT end-to-end: Scheduler -> P2S -> AccessBankArb -> real Mats
# (PICCache). Pick the P2S command to run in pic_test_frontend.cc
# (trace "p2s"). Check what landed in the Mats with --debug-flags=MatBank:
# every Mat write prints "bank B way W: write array A row R = 0x...", and
# the 17-bit array address is (W << 13) | (B << 11) | (A << 9) | R.

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

# p2s_test_data sits next to this config; gem5 does not always put the
# config's own directory on sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p2s_test_data

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]

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
    trace="p2s"
)

system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)

# The frontend's exit_on_finish fires once the SETs are sent, before P2S has
# written anything, so the run ends when the P2S under test reports done.
# Run one P2S command per simulation: the first done ends it.
system.p2s_l = P2S_L(system=system, exit_on_done=True)
system.p2s_r = P2S_R(system=system, exit_on_done=True)
system.p2s_r_t = P2S_R_T(system=system, exit_on_done=True)

# Load path, one SimObject per RTL module (PolymorPIC_main_in_cache.scala):
# P2S -> Load_ctl -> DMA_read_page_split -> ReadPostProcess -> Load_ctl -> P2S
system.load_ctl = Load_ctl(system=system)
system.dma = DMA_read_page_split(
    system=system,
    chunk_bytes=64,
    block_bytes=64,
    max_outstanding=4,
)
system.read_post = ReadPostProcess(system=system)

system.load_ctl.dma_side = system.dma.req_side
system.load_ctl.post_side = system.read_post.req_side
system.dma.data_side = system.read_post.data_side
system.read_post.buf_side = system.load_ctl.buf_side
system.tlb_sched = TLBReqScheduler()
system.dma.tlb_side = system.tlb_sched.cpu_side

# Mats are storage only: start_cycles keeps them from starting their own
# cal() job, which would drop every external write. Way 0 is the normal
# cache way; all other ways are in PIC mode.
cache = PICCache(system, pic_way_mask=0xFFFE, start_cycles=10**9)

system.bank_arb = AccessBankArb(num_banks=4)
for mat in cache.mats:
    system.bank_arb.mat_side = mat.array_side

system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.scheduler.p2sl_port = system.p2s_l.inst_port
system.scheduler.p2sr_port = system.p2s_r.inst_port
system.scheduler.p2srt_port = system.p2s_r_t.inst_port

system.p2s_l.dma_port = system.load_ctl.p2s_side
system.p2s_r.dma_port = system.load_ctl.p2s_side
system.p2s_r_t.dma_port = system.load_ctl.p2s_side

# Index 0/1/2 = P2S_L/P2S_R/P2S_R_T (AccessBankArb's kClientP2S_L offset).
system.p2s_l.cb_port = system.bank_arb.p2s_side[0]
system.p2s_r.cb_port = system.bank_arb.p2s_side[1]
system.p2s_r_t.cb_port = system.bank_arb.p2s_side[2]
system.read_post.bank_side = system.bank_arb.load_post_side

system.pic_membus = SystemXBar()

system.pic_memory = SimpleMemory(
    range=system.mem_ranges[0],
    latency='10ns',
)

system.dma.mem_side = system.pic_membus.cpu_side_ports
system.system_port = system.pic_membus.cpu_side_ports
system.pic_memory.port = system.pic_membus.mem_side_ports

m5.instantiate()

p2s_test_data.write_all(root.system.physProxy)

print('Initialized unified P2S L -> R -> RT PIC cache E2E DRAM patterns')

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