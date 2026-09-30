from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Scheduler(ClockedObject):
    type = "Scheduler"
    cxx_header = "learning_gem5/PIC/scheduler.hh"
    cxx_class = "gem5::Scheduler"

    inst_port = ResponsePort("Scheduler port, receives MMIO requests")
    cc_port = RequestPort("Scheduler port, send request to cache controller")
    p2sl_port = RequestPort("Scheduler direct command port to P2S_L")
    p2sr_port = RequestPort("Scheduler direct command port to P2S_R")
    p2srt_port = RequestPort("Scheduler direct command port to P2S_R_T")
    cb_port = RequestPort("Scheduler direct command port to Cache Bank")
    acc_port = RequestPort("Scheduler direct command port to Accumulator")
    sc_port = RequestPort("Scheduler direct command port to switch controller")
    # time_to_wait = Param.Latency("Time before firing the event")
    # number_of_fires = Param.Int(
    #     1, "Number of times to fire the event before goodbye"
    # )
