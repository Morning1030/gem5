import m5
from m5.objects import *

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = "timing"
system.mem_ranges = [AddrRange("512MB")]

system.cpu = X86TimingSimpleCPU()

system.memobj = SimpleMemobj()

system.cpu.icache_port = system.memobj.inst_port
system.cpu.dcache_port = system.memobj.data_port

system.membus = SystemXBar()

system.memobj.mem_side = system.membus.cpu_side_ports

system.cpu.createInterruptController()
system.cpu.interrupts[0].pio = system.membus.mem_side_ports
system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.mem_ctrl = DDR3_1600_8x8()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

# PolymorPIC modules
system.scheduler = Scheduler()
system.p2sl = P2S_L()
system.p2sr = P2S_R()
system.p2srt = P2S_R_T()
system.accumulator = Accumulator()
system.switch_ctrl = SwitchController=()
system.csh = CmdStateHelper()

# PolymorPIC direct port wiring
system.accumulator.inst_port = system.scheduler.acc_port
system.p2sl.inst_port = system.scheduler.p2sl_port
system.p2sr.inst_port = system.scheduler.p2sr_port
system.p2srt.inst_port = system.scheduler.p2srt_port
system.switch_ctrl.inst_port = system.scheduler.sc_port
system.csh.inst_port = system.scheduler.csh_port


process = Process()
process.cmd = ["tests/test-progs/hello/bin/x86/linux/hello"]
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system=False, system=system)
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print("Exiting @ tick %i because %s" % (m5.curTick(), exit_event.getCause()))
