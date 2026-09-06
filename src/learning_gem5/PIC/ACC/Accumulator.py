from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Accumulator(ClockedObject):
    type = "Accumulator"
    cxx_header = "learning_gem5/PIC/ACC/accumulator.hh"
    cxx_class = "gem5::Accumulator"

    system = Param.System(
        Parent.any,
        "System used to allocate the ACC requestor ID"
    )

    inst_port = ResponsePort(
        "Control port receiving one packed ACC request"
    )

    bank_port = RequestPort(
        "Array-bank access port used by ACC for READ/WRITE"
    )

    wordline_nums = Param.Unsigned(
        512,
        "PolymorPIC wordlines per array"
    )

    arrays_per_mat = Param.Unsigned(
        4,
        "PolymorPIC arrays per Mat; ACC source groups are spaced by this many arrays"
    )
