# PICCache -- the PIC cache banks as one reusable component:
# PICLLCTags + 60 Mats (RTL: BankedStore / BankShellPIC / Bank / Mat).
# A testbench builds it with
#
#   cache = PICCache(system)
#
# Fixed topology (paper section 4.1.3): 4 banks x 16 ways. Ways below
# FIRST_PIC_WAY are plain cache ways with no Mat (their data would live in
# CacheController, which is not part of this file yet). Every other way has
# one Mat per bank: (16 - 1) x 4 = 60 Mats. Bank and way counts are fixed
# constants; only the Mat cal() parameters can be passed in.
#
# The AccessBankArb and its clients (P2S, AutoLoadL, ...) are not in here:
# in the RTL they sit in the PIC main module (PolymorPIC_main_in_cache),
# and the banks only see the arbiter's output. Connect from outside:
#   - for every Mat in .mats, in order:
#       arbiter.mat_side = mat.array_side   (RTL picMem_* path)
#       mat.load_l_port = AutoLoadL.inst_port (RTL request_vec / vec_data,
#                                             the only direct client link)
# .mats order = (way - FIRST_PIC_WAY) * NUM_BANKS + bank, the same order
# AccessBankArb.mat_side and AutoLoadL.inst_port expect.

from m5.objects import ( #m5:the module gem5 provide
    NULL,
    LRURP,    # gem5's built in replacement policy
    Mat,      #60 mats
    PICLLCTags,     #mark states of each cache line
)

###configuration of PIC cache banks###
NUM_BANKS = 4
NUM_WAYS = 16
FIRST_PIC_WAY = 1

# Sub-array: 512 wordlines x 64 bitlines = 4KB. Mat = 4 sub-arrays = 16KB.
# One way = one Mat per bank, so cache = banks x ways x Mat size = 1MB.
_WORDLINES = 512
_BITLINES = 64
_ARRAYS_PER_MAT = 4
_BLOCK_SIZE = 64
_MAT_BYTES = _ARRAYS_PER_MAT * _WORDLINES * _BITLINES // 8  # 16KB
_CACHE_SIZE = f"{NUM_BANKS * NUM_WAYS * _MAT_BYTES // 1024}KiB"  # 1024KB


class PICCache:
    #" **mat_cal_params " will expand -> nBuf, nCal, accWidth, etc. It will change in each cal()
    def __init__(self, system, pic_way_mask=0, **mat_cal_params):
        #system. : send external object in 
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
            initial_pic_ways=pic_way_mask,
        )

        # ---- The 60 PIC Mats ---------------------------------------------
        # Connection order matches the RTL's matID*numBanks+bankID
        self.mats = []

        for w in range(1, 16):
            for b in range(4):
                mat = Mat(
                    system=system,
                    bank_index=b,
                    way=w,
                    tags=system.pic_tags,
                    **mat_cal_params,
                )
                self.mats.append(mat)
        system.mats = self.mats

