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
    trace="p2s"
)

system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)

system.switchCtrl = SwitchController

system.dma = DMAEngine(
    system=system,
    chunk_bytes=64,
    block_bytes=64,
    max_outstanding=8,
)
system.tlb_sched = TLBReqScheduler()
system.dma.tlb_side = system.tlb_sched.cpu_side



system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port

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

# export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | grep -v condabin | paste -sd: -)
# which python3-config   # 确认指向 /usr/bin/python3-config
# rm -rf build/ALL/gem5.build
# scons build/ALL/gem5.opt -j$(nproc)