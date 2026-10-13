# -*- mode:python -*-

# TLBReqScheduler -- see tlb_req_scheduler.hh

from m5.params import *
from m5.objects.ClockedObject import ClockedObject


class TLBReqScheduler(ClockedObject):
    type = "TLBReqScheduler"
    cxx_header = "mem/cache/pic/tlb_req_scheduler.hh"
    cxx_class = "gem5::TLBReqScheduler"

    # 0 = read side, 1 = write side
    cpu_side = VectorResponsePort(
        "Ports the DMA page-split modules send vaddr queries on"
    )

    virtualization = Param.Bool(
        True, "False = identity mapping, no translation latency"
    )
    translate_latency = Param.Cycles(
        10,
        "Cycles one translation takes (placeholder for the PTW latency)",
    )
    state_latency = Param.Cycles(1, "Cycles per FSM state transition")
