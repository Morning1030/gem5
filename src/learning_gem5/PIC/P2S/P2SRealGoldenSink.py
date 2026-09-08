from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class P2SRealGoldenSink(ClockedObject):
    type = "P2SRealGoldenSink"
    cxx_header = (
        "learning_gem5/PIC/p2s_real_golden_sink.hh"
    )
    cxx_class = "gem5::P2SRealGoldenSink"

    port = ResponsePort(
        "Receives real P2S CacheBank write packets"
    )

    mode = Param.String(
        "R",
        "P2S checker mode: L, R, or RT"
    )

    base_pic_addr = Param.Addr(
        0,
        "P2S_L destination PIC base address"
    )

    base_array = Param.Unsigned(
        0,
        "P2S_R/P2S_R_T first destination array ID"
    )

    rows = Param.Unsigned(
        1,
        "Input row count"
    )

    cols = Param.Unsigned(
        1,
        "Input column count"
    )

    precision = Param.Unsigned(
        7,
        "Inclusive last bit index"
    )

    buf_num = Param.Unsigned(
        2,
        "P2SR bufNum; used by R/RT checking"
    )

    wordline_nums = Param.Unsigned(
        512,
        "PolymorPIC wordlines per array"
    )

    exit_on_pass = Param.Bool(
        True,
        "Exit simulation when this sink finishes"
    )
