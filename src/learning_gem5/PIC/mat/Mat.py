from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class Mat(ClockedObject):
    type = "Mat"
    cxx_header = "learning_gem5/PIC/mat/mat.hh"
    cxx_class = "gem5::Mat"

    system = Param.System(
        Parent.any, "System used to allocate Mat requestor ID"
    )

    # This Mat's (bank, way) coordinate. Needed to report busy into
    # tags's (bank, way) coordinate and to compute its L row addresses.
    bank_index = Param.Unsigned(0, "Cache bank index this Mat lives in")
    way = Param.Unsigned(1, "Way index within its bank")

    # RTL Mat(pic=...). False = a normal cache Mat (ways below
    # FIRST_PIC_WAY): only the 4 x 512 x 64-bit SRAM (RTL NormalArray), no
    # controller, never computes. array_side then reads/writes it as plain
    # memory; the arbiter never connects to it (RTL asserts on that).
    pic = Param.Bool(True, "PIC-capable Mat; False = storage only")

    start_cycles = Param.Cycles(1, "Cycles before the first step")
    max_cycles = Param.Unsigned(100000, "Panic if not finished by then")

    # Optional: reports this Mat's non-idle span into tags.setMatBusy()
    # so PICLLCTags can deny CPU access while its job runs. Leave NULL to
    # run without a tag store (e.g. plain unit tests).
    tags = Param.PICLLCTags(NULL, "Tag store to report busy into")

    # Cache-wide AutoLoadL, shared by every Mat (round-robin arbiter --
    # one fetch across the whole cache at a time, matches the RTL).
    load_l_port = RequestPort("Request port to AutoLoadL inst_port")

    # Real C-/M-array read/write traffic to CacheController's cache
    # banks. Same {addr, optype, data} shape as AutoLoadL's own request
    # (RTL: the PolyArray SRAM port is a plain 64-bit read/write port --
    # bit-slicing is a P2S-side concept, never seen past the SRAM port).
    # Declared, and the response path exists, but nothing in mat_fsm/
    # mat_datapath drives it yet -- see mat.cc's file comment.
    cache_port = RequestPort("Request port to CacheController")

    # 8-byte reads/writes from AccessBankArb.mat_side into this Mat's
    # four sub-arrays. Packet address = arrayID_in_mat << 9 | wordline.
    array_side = ResponsePort("Array reads/writes from AccessBankArb")
    array_latency = Param.Cycles(1, "Cycles from array access to response")

    # cal() command setup for this one Mat.
    n_buf = Param.Unsigned(1, "nBuf")
    n_cal = Param.Unsigned(3, "nCal")
    acc_width = Param.Unsigned(16, "accWidth in bits (16 or 32)")
    r_base_bit = Param.Unsigned(0, "R_base_bit")
    r_block_row = Param.Unsigned(1, "R_block_row")
    l_block_row = Param.Unsigned(1, "L_block_row")
    l_precision = Param.Unsigned(0, "L_precision")
    signed_l = Param.Bool(False, "signed_L")
    signed_r_last_exist = Param.Bool(False, "signed_R_last_exist")
    l_vec_fetch_addr = Param.UInt64(0, "L row index this Mat starts at")
