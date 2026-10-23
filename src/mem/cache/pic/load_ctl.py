# -*- mode:python -*-

# Load_ctl -- SimObject, port of Load_ctl.scala. Round-robin arbiter over
# the load clients plus the FSM that sets up ReadPostProcess and walks
# the rows through DMA_read_page_split. See load_ctl.hh.

from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Load_ctl(ClockedObject):
    type = "Load_ctl"
    cxx_header = "mem/cache/pic/load_ctl.hh"
    cxx_class = "gem5::Load_ctl"

    # RTL io.load_req (client generalLoad): Scheduler's LOAD command
    # (LSPayload WriteReq). Answered when the load is done.
    cmd_side = ResponsePort("LOAD commands from the Scheduler")

    # RTL io.p2sL_req / p2sR_req / p2sR_T_req. Exactly three connections,
    # in order P2S_L, P2S_R, P2S_R_T (each engine's dma_port). A P2S row
    # read is answered with the row's data.
    p2s_side = VectorResponsePort("Row reads from P2S_L, P2S_R, P2S_R_T")

    # RTL io.dma_read_req / io.dma_busy.
    dma_side = RequestPort("Row reads to DMA_read_page_split.req_side")

    # RTL io.post_process_req / io.process_busy.
    post_side = RequestPort("Setup to ReadPostProcess.req_side")

    # Stand-in for the RTL dataToBuf wires from ReadPostProcess to the
    # P2S buffers: words for the active P2S read arrive here.
    buf_side = ResponsePort("P2S buffer words from ReadPostProcess.buf_side")

    system = Param.System(Parent.any, "System, for the requestor ID")
