import argparse
import m5
from m5.objects import *

parser = argparse.ArgumentParser(description="PolymorPIC MAC golden/unit test")
parser.add_argument("--random-cases", type=int, default=10000)
args = parser.parse_args()

system = System()
root = Root(full_system=False, system=system)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = "timing"
system.mac_test = MacTest(random_cases=args.random_cases)

m5.instantiate()
print("Starting PolymorPIC MAC unit test")
exit_event = m5.simulate()
print("Exiting @ tick {} because {}".format(m5.curTick(), exit_event.getCause()))
