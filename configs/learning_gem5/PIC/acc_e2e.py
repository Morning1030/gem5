import argparse

import m5
from m5.objects import *


parser = argparse.ArgumentParser(
    description="PolymorPIC ACC frontend E2E test"
)

parser.add_argument(
    "--width",
    type=int,
    choices=[16, 32],
    default=32,
    help="ACC lane width: 16 or 32 bits",
)

args = parser.parse_args()


acc32 = args.width == 32
bit_width_code = 1 if acc32 else 0


system = System()
root = Root(full_system=False, system=system)

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"


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
    trace="acc",
    acc_bit_width_code=bit_width_code,
)


#
# Test-only replacement for the not-yet-integrated
# Scheduler -> ACC dispatch path.
#
# No Scheduler source is modified.
#
system.acc_mmio_bridge = AccMmioBridge(
    system=system,
)


system.acc = Accumulator(
    system=system,
    wordline_nums=512,
    arrays_per_mat=4,
)


system.acc_bank = AccMockBank(
    base_src_addr=0x00003000,
    dest_addr=0x00004000,
    source_count=2,
    row_count=64,
    acc32_bit=acc32,
    wordline_nums=512,
    arrays_per_mat=4,
    response_latency=1,
)


# Test-only ACC E2E wiring
system.pic_transport.command_port = system.acc_mmio_bridge.mmio_port
system.acc_mmio_bridge.acc_port = system.acc.inst_port
system.acc.bank_port = system.acc_bank.port

m5.instantiate()


print(
    "Starting ACC {}-bit frontend E2E: "
    "PicTestFrontend -> "
    "PicMmioTransport -> "
    "AccMmioBridge(test-only) -> "
    "Accumulator -> "
    "AccMockBank".format(args.width)
)


exit_event = m5.simulate()


print(
    "Exiting @ tick {} because {}".format(
        m5.curTick(),
        exit_event.getCause(),
    )
)
