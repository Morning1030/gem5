from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class BankFetch(ClockedObject):
    type = "BankFetch"
    cxx_header = "learning_gem5/PIC/Store/bank_fetch.hh"
    cxx_class = "gem5::BankFetch"
    system = Param.System(
        Parent.any, "System used to allocate BankFetch requestor ID"
    )

    inst_port = ResponsePort("Store Ctrl port, get request to fetch data")
    cb_port = RequestPort("Cache Bank side port, request to take data from cache bank")
    dma_port = RequestPort("DMA write page split port, send data to DMA write")