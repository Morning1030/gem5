# Same P2S L -> R -> RT end-to-end pipeline as p2s_e2e.py, except cb_port
# is wired to a shared AccessBankArb instead of a P2SRealGoldenSink per
# engine. AccessBankArb only arbitrates/passes bit-plane writes through --
# it does not check bit-slice correctness the way P2SRealGoldenSink does.
# This config is a *plumbing* smoke test (do requests actually flow P2S ->
# arbiter -> grant/ack -> P2S without panicking or deadlocking?), not a
# correctness test -- a clean exit here does not mean the arbiter verified
# the math, only that nothing got stuck.

import m5
from m5.objects import *

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
    # p2s_e2e.py relies on its golden sinks' exit_on_pass=True to end the
    # simulation instead. This config has no sink, so the frontend's own
    # completion signal (Scheduler-reported commandFinished, which itself
    # depends on each P2S engine's write queue actually draining through
    # cb_port) is what ends the run -- and what makes a hung cb_port
    # protocol show up as "never exits" instead of running forever.
    exit_on_finish=True,
)

system.scheduler = Scheduler()
system.pic_transport.command_port = system.scheduler.inst_port

system.p2s_l = P2S_L(system=system)
system.p2s_r = P2S_R(system=system)
system.p2s_r_t = P2S_R_T(system=system)

system.dma = DMAEngine(
    system=system,
    chunk_bytes=64,
    block_bytes=64,
    max_outstanding=8,
    tlb_latency=1,
)

system.p2s_l.dma_port = system.dma.p2s_side
system.p2s_r.dma_port = system.dma.p2s_side
system.p2s_r_t.dma_port = system.dma.p2s_side

system.scheduler.mem_side_p2sl = system.p2s_l.inst_port
system.scheduler.mem_side_p2sr = system.p2s_r.inst_port
system.scheduler.mem_side_p2srt = system.p2s_r_t.inst_port

system.bank_arb = AccessBankArb(num_banks=4)

# Index 0/1/2 = P2S_L/P2S_R/P2S_R_T, matching AccessBankArb's own
# kClientP2S_L-offset convention (see access_bank_arb.py/.hh).
system.p2s_l.cb_port = system.bank_arb.p2s_side[0]
system.p2s_r.cb_port = system.bank_arb.p2s_side[1]
system.p2s_r_t.cb_port = system.bank_arb.p2s_side[2]

system.pic_membus = SystemXBar()

system.pic_memory = SimpleMemory(
    range=system.mem_ranges[0],
    latency='10ns',
)

system.dma.mem_side = system.pic_membus.cpu_side_ports
system.system_port = system.pic_membus.cpu_side_ports
system.pic_memory.port = system.pic_membus.mem_side_ports

m5.instantiate()

for row in range(64):
    addr = 0x80010000 + row * 128
    data = bytearray(
        ((row + col) & 0xff)
        for col in range(64)
    )
    root.system.physProxy.write(addr, data)

for row in range(64):
    addr = 0x80030000 + row * 32
    data = bytearray(
        ((row * 13 + col * 29 + 7) & 0xff)
        for col in range(8)
    )
    root.system.physProxy.write(addr, data)

for row in range(2):
    addr = 0x80020000 + row * 128
    data = bytearray(
        ((row + col) & 0xff)
        for col in range(64)
    )
    root.system.physProxy.write(addr, data)

print('Initialized unified P2S L -> R -> RT arbiter E2E DRAM patterns')

exit_event = m5.simulate()

print(
    'Exiting @ tick {} because {}'.format(
        m5.curTick(),
        exit_event.getCause()
    )
)
