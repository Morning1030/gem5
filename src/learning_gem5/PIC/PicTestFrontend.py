from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class PicTestFrontend(ClockedObject):
    type = "PicTestFrontend"
    cxx_header = "learning_gem5/PIC/pic_test_frontend.hh"
    cxx_class = "gem5::pic::PicTestFrontend"

    transport = Param.PicMmioTransport(
        "Frontend-independent transport that receives generated SET requests"
    )

    start_delay = Param.Cycles(
        1,
        "Cycles from simulation startup to the first test request",
    )

    inter_command_gap = Param.Cycles(
        1,
        "Cycles between one completed SET and the next test SET",
    )

    exit_on_finish = Param.Bool(
        True,
        "Exit after every request in the fixed smoke trace completes",
    )
