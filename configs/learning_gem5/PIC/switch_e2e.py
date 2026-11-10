import m5
from m5.objects import *
from pic_cache import PICCache
NUM_BANKS = 4

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1GHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'timing'
system.mem_ranges = [AddrRange(start=0x80000000, size='64MiB')]
system.membus = SystemXBar()
system.memory = SimpleMemory(range=system.mem_ranges[0], latency='10ns')
system.system_port = system.membus.cpu_side_ports
system.memory.port = system.membus.mem_side_ports

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
    trace="switch"
)

system.scheduler = Scheduler(system=system)
system.cmd_state_helper = CmdStateHelper(system=system)
pic_cache = PICCache(system=system, pic_way_mask=0, with_ctrl=True, start_cycles=10**10)
system.switchCtrl = SwitchController(
    system=system,
    cache=pic_cache.ctrl
)
system.bank_arb = AccessBankArb(num_banks=NUM_BANKS)
system.autoloadl = AutoLoadL(system=system)
for mat in pic_cache.mats:
    mat.array_side = system.bank_arb.mat_side
    mat.load_l_port = system.autoloadl.inst_port

system.autoloadl.cb_port = system.bank_arb.autoload_side

system.idle_p2s = [PyTrafficGen() for _ in range(3)]
for i, c in enumerate(system.idle_p2s):
    c.port = system.bank_arb.p2s_side[i]


system.dummy_loadpost = PyTrafficGen()
system.dummy_loadpost.port = system.bank_arb.load_post_side

system.dummy_acc = PyTrafficGen()
system.dummy_acc.port = system.bank_arb.acc_side

system.pic_transport.mmio_port = system.scheduler.inst_port
system.scheduler.csh_port = system.cmd_state_helper.inst_port
system.scheduler.sc_port = system.switchCtrl.inst_port
system.switchCtrl.cache_side = pic_cache.ctrl.pic_ctrl_port
system.switchCtrl.cache_side = pic_cache.ctrl.pic_flush_port

# Just to pass of base cache, TBD
system.tgen = PyTrafficGen()          # 或 SimpleTrafficGen / 你自己的 requestor
system.tgen.port = system.pic_ctrl.cpu_side
system.pic_ctrl.mem_side = system.membus.cpu_side_ports

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
#   configs/learning_gem5/PIC/switch_e2e.py