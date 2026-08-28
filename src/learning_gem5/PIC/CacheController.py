from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class CacheController(ClockedObject):
    type = "CacheController"
    cxx_header = "learning_gem5/PIC/cache_controller.hh"
    cxx_class = "gem5::CacheController"

    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    data_port = ResponsePort("CPU side port, receives MMIO request")
    mem_side = RequestPort("Memory side port, sends requests")

    # time_to_wait = Param.Latency("Time before firing the event")
    # number_of_fires = Param.Int(
    #     1, "Number of times to fire the event before goodbye"
    # )
