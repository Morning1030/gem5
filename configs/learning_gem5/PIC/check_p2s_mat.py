#!/usr/bin/env python3
# Check a p2s_e2e.py log (--debug-flags=<P2S flag>,PICBankArb,MatBank)
# against what P2S should have written into the Mats.
#
#   python3 check_p2s_mat.py {L|R|RT} <log>
#
# Expected values come from the DRAM data in p2s_test_data.py and the
# command in pic_test_frontend.cc (CMDS below; keep the two in sync). A
# bit-plane holds bit b of 64 elements, element i in bit i:
#   L : element e of DRAM row r,  addr = dst + r + b * rows
#   R : row r of DRAM column c,   addr = (base + off[b]) * 512 + c
#   RT: column c of DRAM row r,   addr = (base + off[b]) * 512 + r
# DRAM row r starts at src + r * stride.

import os
import re
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p2s_test_data

WORDLINES = 512
# get_array_relative_offset() in p2s.hh, buffersPerMat = 2
OFFSET_FOR_BIT = [0, 1, 4, 5, 8, 9, 12, 13]

# Commands in pic_test_frontend.cc, with the meaning RTL CMD_resolver_new
# gives SET_SIZE (the stride the frontend means, not what P2S ended up
# reading): rows = SIZE.row, stride = SIZE.offset (bytes),
# cols = SIZE.bytePerRow (R/RT only; P2S_L does not use it).
CMDS = {
    'L': dict(src=0x80010000, dst=0x2000, rows=64, stride=128, prec=7),
    'R': dict(src=0x80030000, base=0x16, rows=64, stride=2560, cols=512,
              prec=7),
    'RT': dict(src=0x80020000, base=0x14, rows=128, stride=64, cols=64,
               prec=7),
}


def dram():
    """Bytes p2s_e2e.py writes before simulate(); everything else is 0."""
    return p2s_test_data.as_dict()


def plane(values, bit):
    word = 0
    for i, v in enumerate(values):
        word |= ((v >> bit) & 1) << i
    return word


def expected(mode):
    c = CMDS[mode]
    mem = dram()
    byte = lambda r, col: mem.get(c['src'] + r * c['stride'] + col, 0)
    exp = {}
    for b in range(c['prec'] + 1):
        if mode == 'L':
            for r in range(c['rows']):
                exp[c['dst'] + r + b * c['rows']] = plane(
                    [byte(r, e) for e in range(64)], b)
        elif mode == 'R':
            for col in range(c['cols']):
                exp[(c['base'] + OFFSET_FOR_BIT[b]) * WORDLINES + col] = plane(
                    [byte(r, col) for r in range(c['rows'])], b)
        else:
            for r in range(c['rows']):
                exp[(c['base'] + OFFSET_FOR_BIT[b]) * WORDLINES + r] = plane(
                    [byte(r, col) for col in range(c['cols'])], b)
    return exp


MAT_RE = re.compile(r'bank (\d+) way (\d+): write array (\d+) row (\d+) = '
                    r'(0x[0-9a-f]+|0)\b')
P2S_RE = re.compile(r'system\.p2s_\w+: Send (?:Request writing bit \d+|'
                    r'bit=\d+ bitslice writing) to ([0-9a-f]+)')


def main():
    mode, path = sys.argv[1], sys.argv[2]
    text = open(path, errors='replace').read()

    mat = []
    for m in MAT_RE.finditer(text):
        b, w, a, r, d = m.groups()
        addr = (int(w) << 13) | (int(b) << 11) | (int(a) << 9) | int(r)
        mat.append((addr, int(d, 16)))
    p2s = [int(a, 16) for a in P2S_RE.findall(text)]
    exp = expected(mode)

    fails = []

    def check(ok, what):
        print(f"[{'PASS' if ok else 'FAIL'}] {what}")
        if not ok:
            fails.append(what)

    check(len(mat) == len(exp),
          f"Mat writes {len(mat)}, expected {len(exp)}")
    check([a for a, _ in mat] == p2s,
          f"Mat write addresses == P2S log addresses, in order "
          f"({len(p2s)} in P2S log)")

    dup = {a: n for a, n in Counter(a for a, _ in mat).items() if n > 1}
    check(not dup, f"no address written twice ({len(dup)} written twice)")
    for a in list(dup)[:5]:
        print(f"    {a:#07x} written {dup[a]} times")

    missing = sorted(set(exp) - {a for a, _ in mat})
    extra = sorted({a for a, _ in mat} - set(exp))
    check(not missing, f"every expected address written ({len(missing)} "
          f"missing)")
    check(not extra, f"no unexpected address ({len(extra)} unexpected)")
    for a in missing[:5]:
        print(f"    missing {a:#07x}")
    for a in extra[:5]:
        print(f"    unexpected {a:#07x}")

    bad = [(a, d) for a, d in mat if a in exp and exp[a] != d]
    check(not bad, f"data matches expected ({len(bad)} mismatches)")
    for a, d in bad[:5]:
        print(f"    {a:#07x}: got {d:#018x} expected {exp[a]:#018x}")

    nonzero = sum(1 for d in exp.values() if d)
    print(f"    ({nonzero} of {len(exp)} expected words are non-zero)")

    dropped = len(re.findall(r'dropped', text))
    check(dropped == 0, f"'dropped' count = {dropped}")

    exits = re.findall(r'Exiting @ tick \d+ because (.*)', text)
    cause = exits[-1] if exits else '(no exit line)'
    check(bool(exits) and 'limit reached' not in cause
          and 'done' in cause, f"exit cause: {cause}")

    for pat in ('panic', 'fatal', 'Segmentation', 'Aborted'):
        n = len(re.findall(pat, text))
        if n:
            check(False, f"'{pat}' appears {n} times")

    print('RESULT:', 'PASS' if not fails else f'FAIL ({len(fails)})')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
