from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class PicTestFrontend(ClockedObject):
    type = "PicTestFrontend"
    cxx_header = "learning_gem5/PIC/pic_test_frontend.hh"
    cxx_class = "gem5::pic::PicTestFrontend"

    transport = Param.PicMmioTransport(
        "Frontend-independent transport that receives generated SET requests"
    )

    start_delay = Param.Cycles(
        1,
        "Cycles from simulation startup to the first test request",
    )

    inter_command_gap = Param.Cycles(
        1,
        "Cycles between one completed SET and the next test SET",
    )

    exit_on_finish = Param.Bool(
        True,
        "Exit after every request in the fixed smoke trace completes",
    )

    trace = Param.String(
        "p2s",
        "Frontend trace: p2s, acc, load (commands from load_cmds), cal "
        "(commands from cal_cmds), or program (commands from program_cmds)",
    )

    load_cmds = VectorParam.String(
        [],
        "trace 'load': one LOAD per entry, "
        "'<src> <dst> <row> <bytePerRow> <offset>' (src = DRAM vaddr, "
        "dst = array address in 64-bit words); command IDs count from 1",
    )

    # Same fields, same order as RTL ISA.c EXE().
    cal_cmds = VectorParam.String(
        [],
        "trace 'cal': one EXE per entry, '<baseLAddr> <targetMatID> "
        "<R_Valid_nRols> <nBufPerMat> <nCalPerMat> <Base_R_Bit> "
        "<L_Precision> <L_Block_Row> <SignL> <SignR_bitLast> <accWidth> "
        "<needL>' (baseLAddr = array address of the first L word, "
        "targetMatID = way * 4 + bank, accWidth 1 = 32-bit); command IDs "
        "count from 1",
    )

    # gem5 only: several commands sent back to back, none waiting for the
    # one before it to finish.
    program_cmds = VectorParam.String(
        [],
        "trace 'program': one command per entry, sent in order without "
        "waiting for completion. 'cal <the 12 cal_cmds fields>'; "
        "'acc <src> <dst> <srcNum> <accRowNum> <bitWidth>' (ISA.c ACC() "
        "order, array addresses, bitWidth 1 = 32-bit); 'wait <cycles>' holds "
        "the next command's first SET that many cycles, in place of ISA.c "
        "waitEnqCmdFree(). Command IDs count from 1",
    )

    acc_bit_width_code = Param.Unsigned(
        1,
        "ACC bit-width code: 0=16-bit lanes, 1=32-bit lanes",
    )
