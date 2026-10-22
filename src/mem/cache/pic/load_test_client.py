# -*- mode:python -*-

# LoadTestClient -- test stand-in for the Scheduler's ld_port. Sends LOAD
# commands to Load_ctl.cmd_side one at a time. See load_test_client.hh.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class LoadTestClient(ClockedObject):
    type = "LoadTestClient"
    cxx_header = "mem/cache/pic/load_test_client.hh"
    cxx_class = "gem5::LoadTestClient"

    port = RequestPort("Connect to Load_ctl.cmd_side")
    system = Param.System(Parent.any, "System, for the requestor ID")

    loads = VectorParam.String(
        [],
        "One LOAD per entry: '<src> <dst> <row> <bytePerRow> <offset>' "
        "(src = DRAM vaddr, dst = array address in 64-bit words)",
    )
    start_delay = Param.Cycles(1, "Cycles before the first LOAD")
    exit_when_done = Param.Bool(
        False, "End the simulation when the last LOAD is done"
    )
