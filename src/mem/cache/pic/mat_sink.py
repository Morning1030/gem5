# -*- mode:python -*-

# MatSink -- test stand-in for the Mats behind AccessBankArb.mat_side.
# See mat_sink.hh.

from m5.params import *
from m5.objects.ClockedObject import ClockedObject


class MatSink(ClockedObject):
    type = "MatSink"
    cxx_header = "mem/cache/pic/mat_sink.hh"
    cxx_class = "gem5::MatSink"

    arr_side = VectorResponsePort("One port per Mat, connect in Mat order")
    latency = Param.Cycles(1, "Cycles from request to response")
    refuse_first = Param.Bool(
        False,
        "Each port refuses its first request once, then sends a retry "
        "retry_delay cycles later (to exercise the arbiter's Block state)",
    )
    read_default = Param.UInt64(
        0, "Value returned when reading an address nothing wrote"
    )
    retry_delay = Param.Cycles(20, "Cycles from a refusal to its retry")
