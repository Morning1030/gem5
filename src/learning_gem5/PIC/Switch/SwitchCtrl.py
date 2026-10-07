from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *

class SwitchController(ClockedObject):
    type = "SwitchController"
    cxx_header = "learning_gem5/PIC/Switch/switchCtrl.hh"
    cxx_class = "gem5::SwitchController"

    system = Param.System(Parent.any,
        "System used to allocate the SwitchController requestor ID")

    inst_port = ResponsePort("Control port receiving switch requests from Scheduler")
    cache_ctrl_port = RequestPort("Port to CacheController PIC control")
    flush_port = RequestPort("Flush request port to CacheController")
    
    num_sets = Param.Unsigned(1024, "LLC sets")
    num_ways = Param.Unsigned(16, "LLC associativity")
    n_way_per_level = Param.Unsigned(1, "Cache ways per PIC level")
    pic_avail_levels = Param.Unsigned(15, "Max PIC levels (total_levels - 1)")