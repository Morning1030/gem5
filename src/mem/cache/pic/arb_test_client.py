# -*- mode:python -*-

# ArbTestClient -- scripted test client for AccessBankArb. See
# arb_test_client.hh.

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class ArbTestClient(ClockedObject):
    type = "ArbTestClient"
    cxx_header = "mem/cache/pic/arb_test_client.hh"
    cxx_class = "gem5::ArbTestClient"

    port = RequestPort("Connect to an AccessBankArb client port")
    system = Param.System(Parent.any, "System, for the requestor ID")

    kind = Param.String(
        "p2s",
        "'p2s': 16-byte P2SWritePayload writes (arb p2s_side). "
        "'autoload': 8-byte reads (arb autoload_side). "
        "'raw': 8-byte reads and writes straight to a Mat's array_side",
    )
    ops = VectorParam.String(
        [], "Script, one op at a time: 'W <addr> <data>' or 'R <addr> <expect>'"
    )
    start_delay = Param.Cycles(1, "Cycles before the first op")
    exit_when_done = Param.Bool(False, "End the simulation after the last op")
    print_reads = Param.Bool(
        False, "Print every checked read (address, value) to stdout"
    )
