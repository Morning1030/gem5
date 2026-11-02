# Test data and golden model for cal_mat_e2e.py (one Mat runs one cal()
# job), shared with check_cal_mat.py so the two can't drift apart.
#
# What a cal() job computes (RTL Controller.scala, Mat.scala, MAC.scala):
#
#   M[l][c] = sum over k of L[l][k] * Rpart[k][c]      (mod 2^accWidth)
#
#   l = 0 .. L_block_row-1   L rows, one L word per row per bit slice
#   c = 0 .. R_block_row     C-array rows (R_block_row is the last row read)
#   k = 0 .. 63              bit k of every L/C word
#
#   L[l][k]     = the (L_precision+1)-bit value of element k of L row l,
#                 two's complement if signed_L
#   Rpart[k][c] = the bits R_base_bit .. R_base_bit+nCal-1 of R[k][c], each
#                 at its own weight 2^bit; the top one counts negative if
#                 signed_R_last_exist (this Mat holds R's sign bit)
#
# golden() computes that as a plain integer matrix product. It does not
# copy the bit-serial AND / PopCount / shift the hardware uses, so a wrong
# shift, sign or bit-slice order shows up as a mismatch.
#
# Where the data goes:
#   C data   sub-array nBuf+j, row c = bit (R_base_bit+j) of R[0..63][c]
#            (Controller main_idle: bitID_R(i) = R_base_bit + i - nBuf)
#   L data   word l_vec_fetch_addr + s*L_block_row + l = bit s of
#            L[l][0..63] (main_wait_L: _L_vec_addr + 1 per fetch, never
#            reset between bit slices)
#   M result n = l*(R_block_row+1) + c, packed in order from M address 0
#            ({array(2), row(9)}, so it runs on into the next Mem-mode
#            sub-array): 16-bit = 4 per word in bits [16j+15:16j], 32-bit =
#            2 per word in bits [32j+31:32j] (wBuf, Cat(wBuf.reverse)).
#            Lanes of the last word past the last result keep what wBuf
#            still holds: the same lanes of the word before (0 if none).
#
# No m5 import: check_cal_mat.py loads this outside gem5.

import random

NUM_BANKS = 4
NUM_WAYS = 16
FIRST_PIC_WAY = 1
WORDLINES = 512

# The Mat under test, and where its L words are (another Mat).
MAT = (1, 0)            # (matID_in_bank = way, bank)
L_MAT = (2, 1)


def array_addr(mat, bank, array, row):
    # [ matID_in_bank(4) | bankID(2) | arrayID_in_mat(2) | wordline(9) ]
    return (mat << 13) | (bank << 11) | (array << 9) | row


def split_addr(a):
    return (a >> 13) & 0xF, (a >> 11) & 0x3, (a >> 9) & 0x3, a & 0x1FF


MAT_BASE = array_addr(*MAT, 0, 0)
L_BASE = array_addr(*L_MAT, 0, 0)

# Junk the M words hold before the job: the first bit slice must not add to
# it (Controller: is_first_slice uses 0, not the M readout).
def junk(a):
    return 0xDEAD000000000000 | a


CASES = {
    # one bit slice, 10 results: the last word is half full
    'u16': dict(n_buf=1, n_cal=3, acc_width=16, r_base_bit=0,
                r_block_row=9, l_block_row=1, l_precision=0,
                signed_l=False, signed_r_last_exist=False, seed=1),
    # 3 L rows x 7 results (wBuf carries on across L rows), 4 bit slices
    # (later slices add to what the M words hold)
    'u16_multi': dict(n_buf=1, n_cal=3, acc_width=16, r_base_bit=0,
                      r_block_row=6, l_block_row=3, l_precision=3,
                      signed_l=False, signed_r_last_exist=False, seed=2),
    # signed L and R, 32-bit, two Mem sub-arrays, R bits 2..3
    's32': dict(n_buf=2, n_cal=2, acc_width=32, r_base_bit=2,
                r_block_row=4, l_block_row=2, l_precision=3,
                signed_l=True, signed_r_last_exist=True, seed=3),
    # 5 x 512 results: the M words run from sub-array 0 into sub-array 1
    'big': dict(n_buf=2, n_cal=2, acc_width=16, r_base_bit=0,
                r_block_row=511, l_block_row=5, l_precision=1,
                signed_l=True, signed_r_last_exist=True, seed=4),
}


class Case:
    def __init__(self, name):
        p = CASES[name]
        self.name = name
        self.__dict__.update(p)
        self.cols = self.r_block_row + 1
        self.rows = self.l_block_row
        rng = random.Random(self.seed)
        # Raw bit patterns: L has L_precision+1 bits, R a full byte (only
        # the bits this Mat holds matter).
        self.L = [[rng.randrange(1 << (self.l_precision + 1))
                   for _ in range(64)] for _ in range(self.rows)]
        self.R = [[rng.randrange(256) for _ in range(self.cols)]
                  for _ in range(64)]
        assert self.n_buf >= 1 and self.n_buf + self.n_cal <= 4
        assert self.r_base_bit + self.n_cal <= 8
        assert self.m_words_needed() <= self.n_buf * WORDLINES, \
            "results do not fit the Mem-mode sub-arrays"

    # ---- golden: integer matrix product ----------------------------------

    def l_value(self, raw):
        if self.signed_l and (raw >> self.l_precision) & 1:
            return raw - (1 << (self.l_precision + 1))
        return raw

    def r_part(self, raw):
        v = 0
        for j in range(self.n_cal):
            bit = self.r_base_bit + j
            w = 1 << bit
            if self.signed_r_last_exist and j == self.n_cal - 1:
                w = -w
            v += ((raw >> bit) & 1) * w
        return v

    def golden(self):
        """M[l][c] as a signed integer (before wrapping to accWidth)."""
        Lv = [[self.l_value(x) for x in row] for row in self.L]
        Rv = [[self.r_part(x) for x in row] for row in self.R]
        return [[sum(Lv[l][k] * Rv[k][c] for k in range(64))
                 for c in range(self.cols)] for l in range(self.rows)]

    # ---- where everything lives ------------------------------------------

    def per_word(self):
        return 64 // self.acc_width

    def m_words_needed(self):
        return -(-self.rows * self.cols // self.per_word())

    def c_words(self):
        """{array address: word} for the C data in the Mat under test."""
        out = {}
        for j in range(self.n_cal):
            bit = self.r_base_bit + j
            for c in range(self.cols):
                word = 0
                for k in range(64):
                    word |= ((self.R[k][c] >> bit) & 1) << k
                out[MAT_BASE + ((self.n_buf + j) << 9) + c] = word
        return out

    def l_words(self):
        """{array address: word} for the L data in L_MAT."""
        out = {}
        for s in range(self.l_precision + 1):
            for l in range(self.rows):
                word = 0
                for k in range(64):
                    word |= ((self.L[l][k] >> s) & 1) << k
                out[L_BASE + s * self.rows + l] = word
        return out

    def m_layout(self):
        """[(array address, [(lane, l, c) or (lane, None, None), ...])]
        for each M word the job writes; (None, None) = stale lane."""
        per = self.per_word()
        out = []
        for w in range(self.m_words_needed()):
            lanes = []
            for j in range(per):
                n = w * per + j
                if n < self.rows * self.cols:
                    lanes.append((j, n // self.cols, n % self.cols))
                else:
                    lanes.append((j, None, None))
            out.append((MAT_BASE + w, lanes))
        return out

    def m_expected(self):
        """{array address: word} the M words must hold after the job."""
        g = self.golden()
        per, bits = self.per_word(), self.acc_width
        mask = (1 << bits) - 1
        out = {}
        prev = [0] * per            # wBuf lanes left from the word before
        for a, lanes in self.m_layout():
            vals = list(prev)
            for j, l, c in lanes:
                if l is not None:
                    vals[j] = g[l][c] & mask
            out[a] = sum(v << (bits * j) for j, v in enumerate(vals))
            prev = vals
        return out

    def guards(self):
        """{array address: word} that must not change: the M word after the
        last result, and every C word."""
        out = {}
        nxt = MAT_BASE + self.m_words_needed()
        if (nxt - MAT_BASE) < self.n_buf * WORDLINES:
            out[nxt] = junk(nxt)
        out.update(self.c_words())
        return out

    def prefill(self):
        """{array address: word} written before the job starts."""
        out = {}
        for w in range(self.m_words_needed() + 1):
            if w < self.n_buf * WORDLINES:
                out[MAT_BASE + w] = junk(MAT_BASE + w)
        out.update(self.c_words())
        out.update(self.l_words())
        return out

    def mat_params(self):
        return dict(n_buf=self.n_buf, n_cal=self.n_cal,
                    acc_width=self.acc_width, r_base_bit=self.r_base_bit,
                    r_block_row=self.r_block_row,
                    l_block_row=self.l_block_row,
                    l_precision=self.l_precision, signed_l=self.signed_l,
                    signed_r_last_exist=self.signed_r_last_exist,
                    l_vec_fetch_addr=L_BASE)

    def compute_cycles(self):
        """Rough upper bound on the job's length, for scheduling reads."""
        return (self.l_precision + 1) * self.rows * (self.cols + 40) + 200
