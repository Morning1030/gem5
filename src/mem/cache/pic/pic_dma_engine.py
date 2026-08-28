# -*- mode:python -*-

# DMAEngine -- SimObject. Issues DMA reads for P2S engines and reassembles
# the responses. See pic_dma_engine.hh for the port topology and the
# "one request from P2S = one row = one response" design note.

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class DMAEngine(ClockedObject):
    type = "DMAEngine"
    cxx_header = "mem/cache/pic/pic_dma_engine.hh"
    cxx_class = "gem5::DMAEngine"

    # One connection per P2S engine. Connection index doubles as the
    # gem5::P2SDest value (pic_dpm_types.hh): 0=P2S_L, 1=P2S_R, 2=P2S_R_T.
    # Each P2S engine's own RequestPort (its "dma_port") connects here.
    p2s_side = VectorResponsePort(
        "Port(s) P2S engines send DMA row-read requests on"
    )

    # This DMAEngine's own request port into the real memory hierarchy.
    mem_side = RequestPort("Port this DMAEngine uses to actually read DRAM")

    system = Param.System(Parent.any, "System this DMAEngine is part of")

    chunk_bytes = Param.UInt32(
        64, "Max bytes DMAEngine puts in one physical read (<= block_bytes)"
    )
    block_bytes = Param.UInt32(
        64, "Cache block size -- chunk ceiling; must match the LLC's block size"
    )
    max_outstanding = Param.UInt32(
        8, "Max concurrent in-flight physical reads DMAEngine may issue"
    )
    tlb_latency = Param.Cycles(
        1,
        "Serialized per-chunk translation latency. translate() is "
        "currently an identity-mapped placeholder (see pic_dma_engine.cc) "
        "-- this still models the round-trip cost even though there's no "
        "real MMU behind it yet.",
    )
