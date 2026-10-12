from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class AccMockBank(ClockedObject):
    type = "AccMockBank"
    cxx_header = "learning_gem5/PIC/ACC/acc_mock_bank.hh"
    cxx_class = "gem5::AccMockBank"

    port = ResponsePort("Mock PolymorPIC array-bank port")

    base_src_addr = Param.Addr(0x1000, "Base source PIC row address")
    dest_addr = Param.Addr(0x9000, "Destination PIC row address")
    source_count = Param.Unsigned(3, "Number of ACC source groups")
    row_count = Param.Unsigned(8, "Number of 64-bit rows to accumulate")
    acc32_bit = Param.Bool(True, "True: two 32-bit lanes; False: four 16-bit lanes")

    wordline_nums = Param.Unsigned(512, "PolymorPIC wordlines per array")
    arrays_per_mat = Param.Unsigned(4, "PolymorPIC arrays per Mat")
    response_latency = Param.Unsigned(1, "Mock read response latency in cycles")
