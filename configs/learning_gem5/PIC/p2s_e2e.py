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
    exit_on_finish=False,
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

system.l_sink = P2SRealGoldenSink(
    mode='L',
    base_pic_addr=0x1000,
    rows=64,
    cols=64,
    precision=7,
    exit_on_pass=False,
)

system.r_sink = P2SRealGoldenSink(
    mode='R',
    base_array=6,
    rows=64,
    cols=8,
    precision=7,
    buf_num=2,
    wordline_nums=512,
    exit_on_pass=False,
)

system.rt_sink = P2SRealGoldenSink(
    mode='RT',
    base_array=4,
    rows=2,
    cols=64,
    precision=7,
    buf_num=2,
    wordline_nums=512,
    exit_on_pass=True,
)

system.p2s_l.cb_port = system.l_sink.port
system.p2s_r.cb_port = system.r_sink.port
system.p2s_r_t.cb_port = system.rt_sink.port

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

print('Initialized unified P2S L -> R -> RT E2E DRAM patterns')

exit_event = m5.simulate()

print(
    'Exiting @ tick {} because {}'.format(
        m5.curTick(),
        exit_event.getCause()
    )
)
