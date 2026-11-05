from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


# RTL BanksShell.scala L84-99 + Bank.scala L63-76: CMD_resolver_new's
# set_up_addr / set_up_io go to every bank and every Mat; only the Mat whose
# matID matches set_up_addr sees exec.
class MatSetUpRouter(ClockedObject):
    type = "MatSetUpRouter"
    cxx_header = "learning_gem5/PIC/mat/mat_setup_router.hh"
    cxx_class = "gem5::MatSetUpRouter"

    # Scheduler.cb_port connects here: one CalPayload per EXE.
    cmd_side = ResponsePort("EXE set-ups from Scheduler.cb_port")

    # Every Mat an EXE may target, in any order: a Mat is found by its own
    # (bank_index, way), not by its place in this list. Each one must have
    # wait_for_setup=True.
    mats = VectorParam.Mat("Mats to route EXE set-ups to")

    # RTL SysConfig.get_bankID / get_in_bank_matID: the matID is
    # {in-bank matID (= way), bankID}, bankID in the low log2(num_banks)
    # bits, so matID = way * num_banks + bank.
    num_banks = Param.Unsigned(4, "Cache banks")
