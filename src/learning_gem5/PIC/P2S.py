from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class P2S_L(ClockedObject):
    type = "P2S_L"
    cxx_header = "learning_gem5/PIC/p2s.hh"
    cxx_class = "gem5::P2S_L"
    system = Param.System(
        Parent.any,
        "System used to allocate P2S_L requestor ID"
    )
    
    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    dma_port = RequestPort("Request port to DMA")
    cb_port = RequestPort("Request port to PIC Cache Bank")

class P2S_R(ClockedObject):
    type = "P2S_R"
    cxx_header = "learning_gem5/PIC/p2s.hh"
    cxx_class = "gem5::P2S_R"
    system = Param.System(
        Parent.any,
        "System used to allocate P2S_R requestor ID"
    )

    wordline_nums = Param.Unsigned(
        512,
        "PolymorPIC wordlines per array"
    )

    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    dma_port = RequestPort("Request port to DMA")
    cb_port = RequestPort("Request port to PIC Cache Bank")

class P2S_R_T(ClockedObject):
    type = "P2S_R_T"
    cxx_header = "learning_gem5/PIC/p2s.hh"
    cxx_class = "gem5::P2S_R_T"

    system = Param.System(
        Parent.any,
        "System used to allocate P2S_R_T requestor ID"
    )

    wordline_nums = Param.Unsigned(
        512,
        "PolymorPIC wordlines per array"
    )

    inst_port = ResponsePort("CPU side port, receives MMIO requests")
    dma_port = RequestPort("Request port to DMA")
    cb_port = RequestPort("Request port to PIC Cache Bank")
