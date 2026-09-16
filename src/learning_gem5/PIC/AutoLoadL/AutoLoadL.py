from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class AutoLoadL(ClockedObject):
    type = "AutoLoadL"
    cxx_header = "learning_gem5/PIC/autoload_l.hh"
    cxx_class = "gem5::AutoLoadL"
    system = Param.System(
        Parent.any, "System used to allocate CmdStateHelper requestor ID"
    )

    inst_port = ResponsePort("Response port to PIC MAT Controller")
    cb_port = RequestPort("Request port to PIC Cache Bank")