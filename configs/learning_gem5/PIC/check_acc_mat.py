#!/usr/bin/env python3
# Check an acc_pic_e2e.py log (--debug-flags=Accumulator,PICBankArb,MatBank)
# against acc_test_data.py.
#
#   python3 check_acc_mat.py {16|32} <log>
#
# Every Mat access prints "bank B way W: read/write array A row R = 0x...";
# the 17-bit address is (W << 13) | (B << 11) | (A << 9) | R. The filler's
# writes to the source rows come first, then ACC reads source 0 and 1 of
# each row and writes their lane-wise sum to dst + row. Expected values come
# from acc_test_data.py, never from what the log read back.

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import acc_test_data as d

MAT_RE = re.compile(r'^\s*(\d+): system\.mats\d+: bank (\d+) way (\d+): '
                    r'(read|write) array (\d+) row (\d+) = (0x[0-9a-f]+|0)\b',
                    re.M)


def main():
    width, path = int(sys.argv[1]), sys.argv[2]
    text = open(path, errors='replace').read()

    accesses = []   # (tick, op, addr, data)
    for m in MAT_RE.finditer(text):
        tick, b, w, op, a, r, data = m.groups()
        addr = (int(w) << 13) | (int(b) << 11) | (int(a) << 9) | int(r)
        accesses.append((int(tick), op, addr, int(data, 16)))

    src = {d.source_addr(s, r): d.SOURCES[s][r]
           for s in range(d.NUM_SOURCES) for r in range(d.ROWS)}
    dst = {d.dst_addr(r): d.expected_sum(r, width) for r in range(d.ROWS)}

    fills = [x for x in accesses if x[1] == 'write' and x[2] in src]
    reads = [x for x in accesses if x[1] == 'read']
    writes = [x for x in accesses if x[1] == 'write' and x[2] in dst]
    other = [x for x in accesses
             if x not in fills and x not in reads and x not in writes]

    fails = []

    def check(ok, what):
        print(f"[{'PASS' if ok else 'FAIL'}] {what}")
        if not ok:
            fails.append(what)

    m = re.search(r'Starting ACC (\d+)-bit', text)
    check(m is not None and int(m.group(1)) == width,
          f"log is a {m.group(1) if m else '?'}-bit run, checking {width}-bit")

    # Preload
    check(len(fills) == len(src),
          f"filler wrote {len(fills)} source words, expected {len(src)}")
    bad = [(a, v) for _, _, a, v in fills if src[a] != v]
    check(not bad, f"preloaded values match acc_test_data ({len(bad)} wrong)")

    # ACC reads, in ACC's order: row 0 source 0, row 0 source 1, row 1 ...
    order = [d.source_addr(s, r)
             for r in range(d.ROWS) for s in range(d.NUM_SOURCES)]
    check(len(reads) == len(order),
          f"ACC reads {len(reads)}, expected {len(order)}")
    check([a for _, _, a, _ in reads] == order,
          "ACC read addresses in order (row-major, source 0 then 1)")
    bad = [(a, v) for _, _, a, v in reads if a not in src or src[a] != v]
    check(not bad, f"ACC read values == preloaded values "
          f"({len(reads) - len(bad)}/{len(reads)} match)")
    for a, v in bad[:5]:
        print(f"    read {a:#07x}: got {v:#018x} expected "
              f"{src.get(a, 0):#018x}")

    # ACC write-back
    check(len(writes) == d.ROWS,
          f"ACC writes {len(writes)}, expected {d.ROWS}")
    check([a for _, _, a, _ in writes] == [d.dst_addr(r)
                                           for r in range(d.ROWS)],
          "ACC write addresses in order (dst + row)")
    bad = [(a, v) for _, _, a, v in writes if dst[a] != v]
    check(not bad, f"ACC sums == lane-wise sum of the sources "
          f"({len(writes) - len(bad)}/{len(writes)} match)")
    for a, v in bad[:5]:
        r = a - d.DST
        print(f"    row {r} ({a:#07x}): got {v:#018x} expected {dst[a]:#018x}"
              f"  sources " + ' '.join(f"{d.SOURCES[s][r]:#018x}"
                                       for s in range(d.NUM_SOURCES)))
    over, total = d.overflowing_lanes(width)
    print(f"    ({over}/{total} {width}-bit lanes overflow and must wrap)")

    check(not other, f"no Mat access outside sources/dst ({len(other)})")
    for t, op, a, v in other[:5]:
        print(f"    {t}: {op} {a:#07x} = {v:#x}")

    if fills and reads:
        check(fills[-1][0] < reads[0][0],
              f"last filler write (tick {fills[-1][0]}) before first ACC "
              f"read (tick {reads[0][0]})")

    dropped = len(re.findall(r'dropped', text))
    check(dropped == 0, f"'dropped' count = {dropped}")

    exits = re.findall(r'Exiting @ tick \d+ because (.*)', text)
    cause = exits[-1] if exits else '(no exit line)'
    check(cause == 'system.acc done', f"exit cause: {cause}")

    for pat in ('panic', 'fatal', 'Segmentation', 'Aborted', 'Assertion'):
        n = len(re.findall(pat, text))
        if n:
            check(False, f"'{pat}' appears {n} times")

    print('RESULT:', 'PASS' if not fails else f'FAIL ({len(fails)})')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
