# PICCache -- the PolymorPIC cache bank hierarchy, as one reusable,
# structurally-fixed component. Any testbench builds it with a single
# line, e.g.:
#
#   system.pic_cache = PICCache(system)
#
# and always gets the same real topology (Section 4.1.3 of the paper):
# 4 banks, 16 ways each. Ways below FIRST_PIC_WAY are PERMANENTLY plain
# cache ways -- no Mat, no PolyArray at all (Bank.scala's pic=false
# branch); this isn't "nothing has switched them yet", the RTL's own
# SWITCH precondition (activate_pre_check's valid_op) makes it
# impossible to ever reach the last FIRST_PIC_WAY-many ways, so their
# data lives ONLY in CacheController's own tag/data store. Ways >=
# FIRST_PIC_WAY are PIC-capable, one Mat SimObject each -> 60 Mats at
# the default split -- their data lives ONLY in that Mat (same physical
# SRAM whether the way is in cache mode or PIC mode, see mat.hh);
# CacheController must forward a cache-mode access to that Mat rather
# than keep its own copy. The split itself (how many ways fall below
# FIRST_PIC_WAY) is DERIVED from cache geometry parameters (RTL:
# in_bank_first_matID/pic_avail_levels depend on waysPerSet/
# cacheSizeBytes/numBanks), not an architectural constant -- 1 vs 15 is
# just where this file's fixed default config lands, so comments/code
# here say "way < FIRST_PIC_WAY", never a literal "way 0"/"way 1-15".
#
# One cache-wide AutoLoadL, one AccessBankArb, one PICLLCTags/
# CacheController pair for the metadata + FIRST_PIC_WAY-and-below data.
# Bank/way counts are module constants, not constructor arguments --
# this is fixed hardware, not a test knob (see NUM_BANKS/NUM_WAYS/
# FIRST_PIC_WAY below). Only per-command Mat cal() parameters (nBuf/
# nCal/accWidth/...) are meant to vary call to call.
#
# This file defines no SimObject of its own -- it's a plain Python
# assembly helper, gem5's usual pattern for packaging a reusable
# sub-system (see e.g. configs/common/CacheConfig.py). It needs no
# SConscript entry; every class it references (PICLLCTags, CacheController,
# AccessBankArb, AutoLoadL, Mat) is already registered there.

from m5.objects import (
    NULL,
    AccessBankArb,
    AutoLoadL,
    CacheController,
    LRURP,
    Mat,
    P2S_L,
    P2S_R,
    P2S_R_T,
    PICLLCTags,
)

# Fixed by the PolymorPIC paper's default config (Section 4.1.3) -- not
# test knobs. A testbench that wants a smaller/faster smoke test should
# build Mat()/AutoLoadL()/AccessBankArb() directly itself, so it's never
# mistaken for "the real cache" at a different size.
NUM_BANKS = 4
NUM_WAYS = 16
# Ways below this are permanently plain cache (no Mat) -- derived from
# cache geometry (in_bank_first_matID/pic_avail_levels), not a constant;
# see the file header comment. 1 is where the default config above lands.
FIRST_PIC_WAY = 1

# PICLLCTags/BaseTags geometry implied by the same fixed config:
# 512-wordline SRAM per Mat sub-array -> 512 sets; 4 sub-arrays/Mat x
# 16 bytes/sub-array = 64-byte line; 16-way associative.
_BLOCK_SIZE = 64
_NUM_SETS = 512
_CACHE_SIZE = f"{(_NUM_SETS * NUM_WAYS * _BLOCK_SIZE) // 1024}KiB"  # 512KiB


class PICCache:
    """One instance = one fully-built PIC cache bank hierarchy, attached
    to `system`. See the file header for the fixed topology and why
    it's a class instantiated once, not a function called with
    different arguments per test.

    `mat_cal_params` are forwarded to every Mat -- the one thing that's
    legitimately per-command, not architectural (nBuf, nCal, accWidth,
    R_base_bit, R_block_row, L_block_row, L_precision, signed_l,
    signed_r_last_exist, l_vec_fetch_addr).
    """

    def __init__(self, system, **mat_cal_params):
        # ---- Tags: metadata for every way, real block data only for
        # ---- ways < FIRST_PIC_WAY (see the file header comment) --------
        # BaseTags's size/block_size/assoc/tag_latency/warmup_percentage/
        # sequential_access/replacement_policy/partitioning_manager
        # normally proxy from the owning BaseCache (Parent.xxx) -- but
        # CacheController.py doesn't extend BaseCache yet (known,
        # deferred base-class bug), so there is no such parent to proxy
        # from right now. Supplied explicitly here so PICLLCTags itself
        # is still constructible while that's unresolved.
        system.pic_tags = PICLLCTags(
            num_banks=NUM_BANKS,
            normal_cache_way=0,
            num_mats_per_bank=NUM_WAYS,
            num_sub_arrays_per_mat=4,
            mat_slice_bytes=16,
            size=_CACHE_SIZE,
            block_size=_BLOCK_SIZE,
            assoc=NUM_WAYS,
            tag_latency=1,
            warmup_percentage=0,
            sequential_access=False,
            replacement_policy=LRURP(),
            partitioning_manager=NULL,
        )

        # ---- CacheController ---------------------------------------------
        # Constructible (the name now exists -- see SConscript), but
        # NOT wired to pic_tags: CacheController.py has no `tags` param
        # (it isn't BaseCache-derived yet). Blocked on the same deferred
        # base-class fix; left disconnected rather than silently faked.
        # Its inst_port/data_port/mem_side are also left unconnected --
        # confirmed non-fatal by itself (gem5's PortRef.ccConnect() just
        # skips a port with no peer; only AccessBankArb's own ctor
        # enforces a connection COUNT, and that's unrelated to this
        # object). Nothing in "the cache banks" has a legitimate MMIO
        # source to hand these to yet anyway -- that's the Scheduler
        # subsystem's job, out of scope here, same as P2S_L/R/R_T's
        # inst_port/dma_port above.
        #
        # Once wired, its job for a way < FIRST_PIC_WAY access is to
        # service it from pic_tags's own CacheBlk data directly (that's
        # the only real data store those ways have). For a way >=
        # FIRST_PIC_WAY access, it must NOT read/write its own copy --
        # pic_tags.isMatBusy(bank, way) (already checked by
        # accessBlock()/findVictim(), see pic_llc_tags.cc) blocks CPU
        # access outright while that Mat is mid-job; when not busy, the
        # access has to be forwarded to that Mat's own storage over
        # cache_port (also not built yet -- see mat.hh) rather than read
        # from any CacheController-local copy.
        system.cache_controller = CacheController()

        # ---- Shared L-vector fetch + bank-access arbitration -------------
        system.access_bank_arb = AccessBankArb(num_banks=NUM_BANKS)
        system.autoload_l = AutoLoadL(system=system)
        system.autoload_l.cb_port = system.access_bank_arb.autoload_side

        # AccessBankArb.p2s_side requires exactly 3 connections (its own
        # ctor asserts this) -- P2S_L/P2S_R/P2S_R_T are fixed clients of
        # the SAME shared bank-access arbiter as AutoLoadL/the Mats
        # (SysConfig.scala's 7-client RRArbiter), not test stubs, so
        # building them here is part of the real fixed hardware, same
        # category as NUM_BANKS/NUM_WAYS above. Their OTHER ports
        # (inst_port from Scheduler, dma_port to DMAEngine) are a
        # different subsystem's job to wire, out of scope for "the cache
        # banks" -- left unconnected here (gem5 doesn't require every
        # declared port to be connected, only AccessBankArb's own
        # connection-count check on p2s_side).
        system.p2s_l = P2S_L(system=system)
        system.p2s_r = P2S_R(system=system)
        system.p2s_r_t = P2S_R_T(system=system)
        system.p2s_l.cb_port = system.access_bank_arb.p2s_side[0]
        system.p2s_r.cb_port = system.access_bank_arb.p2s_side[1]
        system.p2s_r_t.cb_port = system.access_bank_arb.p2s_side[2]

        # ---- The 60 PIC Mats ---------------------------------------------
        # Connection order matches the RTL's matID*numBanks+bankID
        # interleaving (confirmed against SysConfig.scala): same in-bank
        # way, bank varies fastest.
        self.mats = [
            Mat(
                system=system,
                bank_index=b,
                way=w,
                tags=system.pic_tags,
                **mat_cal_params,
            )
            for w in range(FIRST_PIC_WAY, NUM_WAYS)
            for b in range(NUM_BANKS)
        ]
        system.mats = self.mats
        for mat in self.mats:
            mat.load_l_port = system.autoload_l.inst_port
            # mat.cache_port: not yet connected -- see mat.hh's own doc
            # comment (protocol exists, MatFSM/MatDatapath don't drive
            # it yet, and CacheController has no matching port for it
            # either while its base-class fix is pending).
