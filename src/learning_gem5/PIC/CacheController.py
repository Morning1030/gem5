from m5.objects.Cache import Cache
from m5.params import *
from m5.proxy import *


# PIC-aware LLC. 繼承 gem5 的 Cache（BaseCache 的具體子類別）：
# cpu_side / mem_side / tags / system 等參數都由 BaseCache 提供。
class CacheController(Cache):
    type = "CacheController"
    cxx_header = "learning_gem5/PIC/cache_controller.hh"
    cxx_class = "gem5::CacheController"

    # SwitchController 的 cache_side[0] 接這裡（RTL: queryDir / queryRes）
    pic_ctrl_port = ResponsePort("PIC control port from SwitchController")
    # SwitchController 的 cache_side[1] 接這裡（RTL: flushReq / flushDone）
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

    # ---- switch 期間被擋住的 sinkA ----
    # RTL 是 sinkA.ready = 0，上游卡住；gem5 的 cpu_side 只有一個 port，
    # 只擋 A 不擋 C（writeback）做不到，所以改成收下來排隊。上限是安全網，
    # 實際深度看 stats 的 pic.heldSinkAPeak
    max_held_sink_a = Param.Unsigned(
        1024, "panic if more sinkA requests than this are held (0 = no limit)")

    # ---- Cache / PIC bank 衝突（RTL: BankSellPIC L71、L112-116）----
    # cache 路徑用到某個 bank 時 picMem_ready = false，cache 永遠優先。
    # NULL = 不模擬 bank 衝突（現有測試行為不變）
    bank_arb = Param.AccessBankArb(
        NULL, "AccessBankArb told which banks the cache path is using")
    # 近似值，之後對 RTL waveform 校正
    bank_busy_delay = Param.Cycles(
        1, "request received -> first bank beat")
    # 從 Chipyard 預設參數推得（writeBytes=8），待確認實際 config
    bank_beat_bytes = Param.Unsigned(8, "bytes per bank beat")
