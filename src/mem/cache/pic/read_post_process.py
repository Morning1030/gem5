# -*- mode:python -*-

# ReadPostProcess -- SimObject, port of ReadPostProcess.scala. Aligns the
# DMA_read_page_split data stream into 64-bit words and sends them to the
# cache banks (through AccessBankArb) or to a P2S input buffer (through
# Load_ctl). See read_post_process.hh.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class ReadPostProcess(ClockedObject):
    type = "ReadPostProcess"
    cxx_header = "mem/cache/pic/read_post_process.hh"
    cxx_class = "gem5::ReadPostProcess"

    # RTL io.req / io.busy: Load_ctl.post_side connects here.
    req_side = ResponsePort("Setup (ReadPostProcessReq) from Load_ctl")

    # RTL io.unaligned_dataIn: DMA_read_page_split.data_side connects here.
    data_side = ResponsePort("Data stream (StreamBeat) from page split")

    # RTL io.accessBank: AccessBankArb client 0 (load_post_process).
    bank_side = RequestPort("Aligned words to AccessBankArb.load_post_side")

    # RTL io.dataToBuf / dataToBufAvailableP2S*: to Load_ctl.buf_side,
    # which answers the waiting P2S engine.
    buf_side = RequestPort("Aligned words for a P2S buffer, to Load_ctl")

    system = Param.System(Parent.any, "System, for the requestor ID")
