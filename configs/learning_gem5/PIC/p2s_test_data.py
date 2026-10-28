# DRAM test data for p2s_e2e.py, shared with check_p2s_mat.py so the two
# can't drift apart. One block per P2S mode, covering exactly what the
# command in pic_test_frontend.cc (trace "p2s") reads: row r of a block
# starts at src + r * stride and holds bytes_per_row bytes. Everything
# outside the blocks stays 0.
#
# No m5 import: check_p2s_mat.py loads this outside gem5.

MEM_BASE = 0x80000000      # p2s_e2e.py system.mem_ranges
MEM_SIZE = 64 * 1024 * 1024


class Block:
    def __init__(self, name, src, rows, bytes_per_row, stride, value):
        self.name = name
        self.src = src
        self.rows = rows
        self.bytes_per_row = bytes_per_row
        self.stride = stride
        self.value = value

    def row_addr(self, row):
        return self.src + row * self.stride

    def end(self):
        """One past the last byte the block covers."""
        return self.row_addr(self.rows - 1) + self.bytes_per_row

    def row_bytes(self, row):
        return bytearray(self.value(row, col) & 0xff
                         for col in range(self.bytes_per_row))


# Frontend commands (pic_test_frontend.cc), SET_SIZE as RTL reads it:
#   p2sl (0x80010000, ..., row 64, bytePerRow 64 (unused), offset 128)
#   p2sr (0x80030000, ..., row 64, bytePerRow 512, offset 2560)
#   p2srt(0x80020000, ..., row 128, bytePerRow 64, offset 64)
# P2S_L always reads 64 B per row.
BLOCKS = {
    'L': Block('L', 0x80010000, 64, 64, 128,
               lambda row, col: (row * 7 + col * 13 + 1) & 0xff),
    'R': Block('R', 0x80030000, 64, 512, 2560,
               lambda row, col: (row * 11 + col * 5 + 3) & 0xff),
    'RT': Block('RT', 0x80020000, 128, 64, 64,
                lambda row, col: (row * 3 + col * 17 + 9) & 0xff),
}


def check():
    """Raise if the blocks overlap, leave memory, or have a byte equal to
    its neighbour in the next row or next column."""
    blocks = sorted(BLOCKS.values(), key=lambda b: b.src)
    for b in blocks:
        if b.stride < b.bytes_per_row:
            raise ValueError(f"{b.name}: stride {b.stride} < bytes_per_row "
                             f"{b.bytes_per_row}, rows overlap")
        if b.src < MEM_BASE or b.end() > MEM_BASE + MEM_SIZE:
            raise ValueError(f"{b.name}: {b.src:#x}..{b.end():#x} is outside "
                             f"memory {MEM_BASE:#x}..{MEM_BASE + MEM_SIZE:#x}")
        for row in range(b.rows):
            for col in range(b.bytes_per_row):
                v = b.value(row, col)
                if row + 1 < b.rows and b.value(row + 1, col) == v:
                    raise ValueError(f"{b.name}: rows {row} and {row + 1} "
                                     f"have the same value at col {col}")
                if col + 1 < b.bytes_per_row and b.value(row, col + 1) == v:
                    raise ValueError(f"{b.name}: cols {col} and {col + 1} "
                                     f"have the same value in row {row}")
    for a, b in zip(blocks, blocks[1:]):
        if a.end() > b.src:
            raise ValueError(f"{a.name} ({a.src:#x}..{a.end():#x}) overlaps "
                             f"{b.name} ({b.src:#x}..{b.end():#x})")


def write_all(phys_proxy):
    """Write every block into gem5 memory (call after m5.instantiate())."""
    check()
    for b in BLOCKS.values():
        for row in range(b.rows):
            phys_proxy.write(b.row_addr(row), b.row_bytes(row))


def as_dict():
    """{addr: byte} for every byte write_all() writes."""
    check()
    mem = {}
    for b in BLOCKS.values():
        for row in range(b.rows):
            base = b.row_addr(row)
            for col, v in enumerate(b.row_bytes(row)):
                mem[base + col] = v
    return mem
