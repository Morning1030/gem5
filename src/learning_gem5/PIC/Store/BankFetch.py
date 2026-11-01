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

    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    store_ctrl_port = RequestPort("DMA side port, request to write data")
    bank_fetch_port = RequestPort("Cache Bank side port, request to take data from cache bank")