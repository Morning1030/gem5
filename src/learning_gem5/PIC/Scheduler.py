from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Scheduler(ClockedObject):
    type = "Scheduler"
    cxx_header = "learning_gem5/PIC/scheduler.hh"
    cxx_class = "gem5::Scheduler"

    inst_port = ResponsePort("Scheduler port, receives MMIO requests")
    mem_side_cc = RequestPort("Scheduler port, send request to cache controller")
    mem_side_p2sl = RequestPort("Scheduler direct command port to P2S_L")
    mem_side_p2sr = RequestPort("Scheduler direct command port to P2S_R")
    mem_side_p2srt = RequestPort("Scheduler direct command port to P2S_R_T")
    mem_side_cb = RequestPort("Scheduler port, send request to cache bank")
    # time_to_wait = Param.Latency("Time before firing the event")
    # number_of_fires = Param.Int(
    #     1, "Number of times to fire the event before goodbye"
    # )
