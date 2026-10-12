from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Scheduler(ClockedObject):
    type = "Scheduler"
    cxx_header = "learning_gem5/PIC/Control/scheduler.hh"
    cxx_class = "gem5::Scheduler"

    system = Param.System(
            Parent.any,
            "System used to allocate the scheduler requestor ID",
    )

    inst_port = ResponsePort("Scheduler port, receives MMIO requests")
    ld_port = RequestPort("Scheduler direct port to load Ctrl")
    st_port = RequestPort("Scheduler direct port to store Ctrl")
    cc_port = RequestPort("Scheduler port, send request to cache controller")
    p2sl_port = RequestPort("Scheduler direct command port to P2S_L")
    p2sr_port = RequestPort("Scheduler direct command port to P2S_R")
    p2srt_port = RequestPort("Scheduler direct command port to P2S_R_T")
    cb_port = RequestPort("Scheduler direct command port to Cache Bank")
    acc_port = RequestPort("Scheduler direct command port to Accumulator")
    sc_port = RequestPort("Scheduler direct command port to switch controller")
    csh_port = RequestPort("Scheduler direct commnad port to cmdState Helper")
    # time_to_wait = Param.Latency("Time before firing the event")
    # number_of_fires = Param.Int(
    #     1, "Number of times to fire the event before goodbye"
    # )
