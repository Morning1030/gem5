from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class StoreCtrl(ClockedObject):
    type = "StoreCtrl"
    cxx_header = "learning_gem5/PIC/Store/store_ctrl.hh"
    cxx_class = "gem5::StoreCtrl"
    system = Param.System(
        Parent.any, "System used to allocate StoreCtrl requestor ID"
    )

    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    dma_port = RequestPort("DMA side port, request to write data")
    bank_fetch_port = RequestPort("Cache Bank side port, request to take data from cache bank")