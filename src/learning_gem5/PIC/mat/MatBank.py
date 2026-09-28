from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class MatBank(ClockedObject):
    type = "MatBank"
    cxx_header = "learning_gem5/PIC/mat/mat_bank.hh"
    cxx_class = "gem5::MatBank"

    system = Param.System(
        Parent.any, "System used to allocate MatBank requestor ID"
    )

    # One SimObject per cache bank; owns num_mats plain MatFSM instances
    # stepped by a single per-bank tick event.
    num_mats = Param.Unsigned(16, "Number of Mats in this bank")
    start_cycles = Param.Cycles(1, "Cycles before the first Mat step")
    max_cycles = Param.Unsigned(100000, "Panic if not finished by then")

    # Way 0 is the reserved normal-cache level: no FSM, no port. Ways
    # first_pic_way..num_mats-1 each get an FSM and one load_l_port
    # (connect them in ascending way order).
    first_pic_way = Param.Unsigned(1, "First PIC-capable way (Mat)")

    # Which bank index this instance is (0..numBanks-1) -- needed to
    # report per-Mat busy into tags's (bank, way) coordinate.
    bank_index = Param.Unsigned(0, "This bank's index")

    # Optional: reports each Mat's non-idle span into tags.setMatBusy()
    # so PICLLCTags can deny CPU access while that Mat's job runs. Leave
    # NULL to run without a tag store (e.g. plain unit tests).
    tags = Param.PICLLCTags(NULL, "Tag store to report per-Mat busy into")

    load_l_port = VectorRequestPort(
        "One request port per PIC Mat, to an AutoLoadL inst_port")

    # Shared cal() command setup (same for every Mat in this bank).
    n_buf = Param.Unsigned(1, "nBuf")
    n_cal = Param.Unsigned(3, "nCal")
    acc_width = Param.Unsigned(16, "accWidth in bits (16 or 32)")
    r_base_bit = Param.Unsigned(0, "R_base_bit")
    r_block_row = Param.Unsigned(1, "R_block_row")
    l_block_row = Param.Unsigned(1, "L_block_row")
    l_precision = Param.Unsigned(0, "L_precision")
    signed_l = Param.Bool(False, "signed_L")
    signed_r_last_exist = Param.Bool(False, "signed_R_last_exist")
    l_vec_fetch_addr = Param.UInt64(0, "L row index for Mat 0")
    l_vec_mat_stride = Param.UInt64(0, "Added to L row index per Mat")
