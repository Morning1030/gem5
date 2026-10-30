# -*- mode:python -*-

# mod: p2s_arbiter
# DMAWritePageSplit -- SimObject, port of DMAWritePageSplit.scala. Takes
# one row write at a time from Load_ctl, splits it at page boundaries,
# translates each page through TLBReqScheduler, writes DRAM, and streams the
# data in order to ReadPostProcess. See dma_write_page_split.hh.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class DMAWritePageSplit(ClockedObject):
    type = "DMAWritePageSplit"
    cxx_header = "learning_gem5/PIC/Store/dma_write_page_split.hh"
    cxx_class = "gem5::DMAWritePageSplit"
    system = Param.System(
        Parent.any, "System used to allocate BankFetch requestor ID"
    )
    # RTL io.req / io.busy: Load_ctl.dma_side connects here.
    req_side = ResponsePort("Row write requests (DMAReadReq) from Bank Fetch")

    # Queries go to TLBReqScheduler's cpu_side[0] (write side).
    tlb_side = RequestPort("Per-page vaddr->paddr queries to TLBReqScheduler")

    # RTL dmaReader (icenet StreamReader) TileLink port.
    mem_side = RequestPort("Port used to write DRAM")

    system = Param.System(Parent.any, "System, for the requestor ID")

    # chunk_bytes = Param.UInt32(
    #     64, "Max bytes in one DRAM write (RTL maxBytes; <= block_bytes)"
    # )
    # block_bytes = Param.UInt32(
    #     64,
    #     "Cache block size -- chunk ceiling; must match the LLC's block size",
    # )
    # max_outstanding = Param.UInt32(
    #     4, "Max DRAM writes in flight (RTL nXacts)"
    # )
    # skip_same_page = Param.Bool(
    #     False,
    #     "False (RTL): query the TLB for every page segment. "
    #     "True: reuse the last translation if the segment is in the same page",
    # )
