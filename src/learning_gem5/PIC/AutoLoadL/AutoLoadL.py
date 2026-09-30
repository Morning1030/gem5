from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class AutoLoadL(ClockedObject):
    type = "AutoLoadL"
    cxx_header = "learning_gem5/PIC/AutoLoadL/AutoLoadL.hh"
    cxx_class = "gem5::AutoLoadL"
    system = Param.System(
        Parent.any, "System used to allocate AutoLoadL requestor ID"
    )

    # One port per PIC Mat (the RTL's matID*numBanks+bankID request port).
    # Round-robin priority follows connection order.
    inst_port = VectorResponsePort("Response ports, one per PIC Mat")
    cb_port = RequestPort("Request port to the cache bank / AccessBankArb")
