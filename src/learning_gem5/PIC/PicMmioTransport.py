from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import Parent


class PicMmioTransport(ClockedObject):
    type = "PicMmioTransport"
    cxx_header = "learning_gem5/PIC/pic_mmio_transport.hh"
    cxx_class = "gem5::pic::PicMmioTransport"

    command_port = RequestPort(
        "PIC-facing RequestPort used to send one SET transaction"
    )

    system = Param.System(
        Parent.any,
        "System used to allocate a valid gem5 requestor ID",
    )

    request_gap = Param.Cycles(
        1,
        "Minimum delay before sending an accepted frontend request",
    )

    protocol_retry_delay = Param.Cycles(
        50,
        "Delay before reissuing SET_PARAM after an all-ones response",
    )
