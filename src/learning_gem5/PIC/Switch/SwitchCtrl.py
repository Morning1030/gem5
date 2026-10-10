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
    total_mat_num = Param.Unsigned(64, "Total Mats in the LLC (RTL totalMatNum)")
    cache = Param.CacheController("the LLC this switch gates")

        # 由 RTL 推得：fire 讀 dir → +1 MSHR 拿 meta → +2 schedule（假設 dirReg=false、無仲裁 stall）
    flush_lookup_lat = Param.Cycles(2, "FlushReq fire -> MSHR schedule")
    # 以下三個是佔位值，需要從 RTL waveform 校正
    flush_miss_done_lat = Param.Cycles(2, "miss: schedule -> FlushDone (SourceX->Router)")
    flush_release_ack_lat = Param.Cycles(20, "clean hit: Release -> ReleaseAck -> FlushDone")
    flush_release_data_ack_lat = Param.Cycles(30, "dirty hit: ReleaseData -> ReleaseAck -> FlushDone")