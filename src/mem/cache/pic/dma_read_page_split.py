# -*- mode:python -*-

# mod: p2s_arbiter
# DMA_read_page_split -- SimObject, port of DMA_read_page_split.scala. Takes
# one row read at a time from Load_ctl, splits it at page boundaries,
# translates each page through TLBReqScheduler, reads DRAM, and streams the
# data in order to ReadPostProcess. See dma_read_page_split.hh.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class DMA_read_page_split(ClockedObject):
    type = "DMA_read_page_split"
    cxx_header = "mem/cache/pic/dma_read_page_split.hh"
    cxx_class = "gem5::DMA_read_page_split"

    # RTL io.req / io.busy: Load_ctl.dma_side connects here.
    req_side = ResponsePort("Row read requests (DMAReadReq) from Load_ctl")

    # RTL io.dataOut: connects to ReadPostProcess.data_side.
    data_side = RequestPort("Data stream (StreamBeat) to ReadPostProcess")

    # Queries go to TLBReqScheduler's cpu_side[0] (read side).
    tlb_side = RequestPort("Per-page vaddr->paddr queries to TLBReqScheduler")

    # RTL dmaReader (icenet StreamReader) TileLink port.
    mem_side = RequestPort("Port used to read DRAM")

    system = Param.System(Parent.any, "System, for the requestor ID")

    chunk_bytes = Param.UInt32(
        64, "Max bytes in one DRAM read (RTL maxBytes; <= block_bytes)"
    )
    block_bytes = Param.UInt32(
        64,
        "Cache block size -- chunk ceiling; must match the LLC's block size",
    )
    max_outstanding = Param.UInt32(
        4, "Max DRAM reads in flight (RTL nXacts)"
    )
    skip_same_page = Param.Bool(
        False,
        "False (RTL): query the TLB for every page segment. "
        "True: reuse the last translation if the segment is in the same page",
    )
