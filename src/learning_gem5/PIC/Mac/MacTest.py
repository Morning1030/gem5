from m5.objects.ClockedObject import ClockedObject
from m5.params import *

class MacTest(ClockedObject):
    type = "MacTest"
    cxx_header = "learning_gem5/PIC/Mac/mac_test.hh"
    cxx_class = "gem5::pic::MacTest"

    random_cases = Param.Unsigned(
        10000,
        "Number of deterministic randomized MAC primitive golden cases",
    )
