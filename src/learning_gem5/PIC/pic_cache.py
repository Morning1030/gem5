# PICCache -- the PolymorPIC cache bank hierarchy, as one reusable,
# structurally-fixed component. Any testbench builds it with a single
# line, e.g.:
#
#   system.pic_cache = PICCache(system)
#
# and always gets the same real topology (Section 4.1.3 of the paper):
# 4 banks, 16 ways each (way 0 = the reserved normal-cache way, no Mat;
# ways 1-15 = PIC-capable, one Mat SimObject each -> 60 Mats total), one
# cache-wide AutoLoadL, one AccessBankArb, one PICLLCTags/CacheController
# owning the tag/data store. Bank/way counts are module constants, not
# constructor arguments -- this is fixed hardware, not a test knob (see
# NUM_BANKS/NUM_WAYS/FIRST_PIC_WAY below). Only per-command Mat cal()
# parameters (nBuf/nCal/accWidth/...) are meant to vary call to call.
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
    PICLLCTags,
)

# Fixed by the PolymorPIC paper's default config (Section 4.1.3) -- not
# test knobs. A testbench that wants a smaller/faster smoke test should
# build Mat()/AutoLoadL()/AccessBankArb() directly itself, so it's never
# mistaken for "the real cache" at a different size.
NUM_BANKS = 4
NUM_WAYS = 16
FIRST_PIC_WAY = 1  # way 0 = reserved normal-cache way, no Mat

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
        # ---- Tag/data store -------------------------------------------
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

        # ---- CacheController --------------------------------------------
        # Constructible (the name now exists -- see SConscript), but
        # NOT wired to pic_tags: CacheController.py has no `tags` param
        # (it isn't BaseCache-derived yet). Blocked on the same deferred
        # base-class fix; left disconnected rather than silently faked.
        system.cache_controller = CacheController()

        # ---- Shared L-vector fetch + bank-access arbitration -------------
        system.access_bank_arb = AccessBankArb(num_banks=NUM_BANKS)
        system.autoload_l = AutoLoadL(system=system)
        system.autoload_l.cb_port = system.access_bank_arb.autoload_side

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
