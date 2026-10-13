from m5.objects.Cache import Cache
from m5.params import *
from m5.proxy import *


# PIC-aware LLC. 繼承 gem5 的 Cache（BaseCache 的具體子類別）：
# cpu_side / mem_side / tags / system 等參數都由 BaseCache 提供。
class CacheController(Cache):
    type = "CacheController"
    cxx_header = "learning_gem5/PIC/cache_controller.hh"
    cxx_class = "gem5::CacheController"

    # SwitchController 的 cache_ctrl_port 接這裡（RTL: queryDir / queryRes）
    pic_ctrl_port = ResponsePort("PIC control port from SwitchController")
    # SwitchController 的 flush_port 接這裡（RTL: flushReq / flushDone）
    pic_flush_port = ResponsePort("PIC flush port from SwitchController")

    # ---- Flush latency（RTL: FlushReqRouter → Scheduler → MSHR → SourceX）----
    # 由 RTL 推得：fire 讀 dir → +1 MSHR 拿 meta → +2 schedule（假設 dirReg=false、無仲裁 stall）
    flush_lookup_lat = Param.Cycles(2, "FlushReq accepted -> MSHR schedule")
    # 以下是佔位值，需要從 RTL waveform 校正
    flush_miss_done_lat = Param.Cycles(
        2, "miss: schedule -> FlushDone (SourceX -> Router)")
    flush_release_ack_lat = Param.Cycles(
        20, "clean hit: Release -> ReleaseAck -> FlushDone")
    flush_release_data_ack_lat = Param.Cycles(
        30, "dirty hit: ReleaseData -> ReleaseAck -> FlushDone")
    flush_probe_lat = Param.Cycles(
        10, "inner copy: Probe(toN) -> ProbeAck")
    flush_probe_data_lat = Param.Cycles(
        12, "inner dirty copy: Probe(toN) -> ProbeAckData")
