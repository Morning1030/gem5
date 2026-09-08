from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class CmdStateHelper(ClockedObject):
    type = "CmdStateHelper"
    cxx_header = "learning_gem5/PIC/cmd_state_helper.hh"
    cxx_class = "gem5::CmdStateHelper"
    system = Param.System(
        Parent.any, "System used to allocate CmdStateHelper requestor ID"
    )

    inst_port = ResponsePort("CPU side port, receives MMIO requests")