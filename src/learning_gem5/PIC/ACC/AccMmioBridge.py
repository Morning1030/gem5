from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class AccMmioBridge(ClockedObject):
    type = "AccMmioBridge"
    cxx_header = "learning_gem5/PIC/ACC/acc_mmio_bridge.hh"
    cxx_class = "gem5::pic::AccMmioBridge"

    system = Param.System(
        Parent.any,
        "System used to allocate the ACC bridge requestor ID",
    )

    mmio_port = ResponsePort(
        "Test-only MMIO endpoint receiving SET_SRC/SET_DST/SET_PARAM(ACC)"
    )

    acc_port = RequestPort(
        "Test-only control path forwarding one packed ACC request"
    )
