# -*- mode:python -*-

# mod: p2s_arbiter
# AccessBankArb -- SimObject. Single round-robin arbiter shared by the P2S
# engines (and, eventually, the other SysConfig clients) for access to the
# PIC bank hierarchy's shared physical port. See access_bank_arb.hh for the
# full design note and the port protocol P2S speaks on p2s_side.

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class AccessBankArb(ClockedObject):
    type = "AccessBankArb"
    cxx_header = "mem/cache/pic/access_bank_arb.hh"
    cxx_class = "gem5::AccessBankArb"

    # Exactly one connection per P2S engine. Connection index doubles as
    # (BankArbClient - kClientP2S_L): 0=P2S_L, 1=P2S_R, 2=P2S_R_T. Each
    # P2S engine's own RequestPort (its "cb_port") connects here.
    p2s_side = VectorResponsePort(
        "Port(s) P2S engines send bit-plane WRITE requests on"
    )

    num_banks = Param.Unsigned(4, "Number of cache banks (paper default: 4)")
