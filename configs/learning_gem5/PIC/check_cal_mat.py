#!/usr/bin/env python3
# Check a cal_mat_e2e.py log (--debug-flags=MatBank, stdout and stderr in
# one file) against the golden model in cal_test_data.py.
#
#   python3 check_cal_mat.py <case> <log>
#
# Every M word the job writes is compared lane by lane; a wrong lane is
# reported as M[l][c] (got vs expected), so a wrong shift, sign or layout is
# easy to tell apart. Guard words (C data, the M word after the last
# result) must read back unchanged.

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cal_test_data as ctd

READ_RE = re.compile(r'system\.reader: read (0x[0-9a-f]+|0) = '
                     r'(0x[0-9a-f]+|0)\b')


def main():
    case = ctd.Case(sys.argv[1])
    text = open(sys.argv[2], errors='replace').read()

    got = {int(a, 16): int(v, 16) for a, v in READ_RE.findall(text)}
    expected = case.m_expected()
    guards = case.guards()
    golden = case.golden()
    per, bits = case.per_word(), case.acc_width
    mask = (1 << bits) - 1

    fails = []

    def check(ok, what):
        print(f"[{'PASS' if ok else 'FAIL'}] {what}")
        if not ok:
            fails.append(what)

    want = set(expected) | set(guards)
    missing = sorted(want - set(got))
    check(not missing, f"every M and guard word read back ({len(missing)} "
          f"missing of {len(want)})")

    # M words, lane by lane. A word that was not read back counts as wrong.
    bad = []
    for a, lanes in case.m_layout():
        if a not in got:
            bad += [(a, j, l, c, None, None) for j, l, c in lanes]
            continue
        for j, l, c in lanes:
            g = (got[a] >> (bits * j)) & mask
            e = (expected[a] >> (bits * j)) & mask
            if g != e:
                bad.append((a, j, l, c, g, e))
    n_results = case.rows * case.cols
    check(not bad, f"M results match the integer matrix product "
          f"({n_results} results in {len(expected)} words, "
          f"{len(bad)} wrong lanes)")
    for a, j, l, c, g, e in bad[:12]:
        who = f"M[{l}][{c}]" if l is not None else "stale lane"
        if g is None:
            print(f"    {a:#07x} lane {j} {who}: not read back")
            continue
        true = f" (= {golden[l][c]} before wrapping)" if l is not None \
            else ""
        print(f"    {a:#07x} lane {j} {who}: got {g:#x}, expected {e:#x}"
              f"{true}")
    if len(bad) > 12:
        print(f"    ... {len(bad) - 12} more")

    nonzero = sum(1 for row in golden for v in row if v & mask)
    print(f"    ({nonzero} of {n_results} expected results are non-zero)")

    bad_guard = [a for a in guards if got.get(a) != guards[a]]
    check(not bad_guard, f"guard words unchanged ({len(guards)} words, "
          f"{len(bad_guard)} changed)")
    for a in bad_guard[:5]:
        g = f"{got[a]:#x}" if a in got else "nothing"
        print(f"    {a:#07x} {ctd.split_addr(a)}: got {g}, "
              f"expected {guards[a]:#x}")

    dropped = len(re.findall(r'dropped, Mat is busy', text))
    check(dropped == 0, f"no array access dropped while the Mat was busy "
          f"({dropped})")

    finished = 'finished' in text and 'because Mat finished' in text
    check(finished, "the Mat finished its job")
    exits = re.findall(r'Exiting @ tick \d+ because (.*)', text)
    cause = exits[-1] if exits else '(no exit line)'
    check(cause == 'ArbTestClient done', f"exit cause: {cause}")

    for pat in ('panic', 'fatal', 'Segmentation', 'Aborted'):
        n = len(re.findall(pat, text))
        if n:
            check(False, f"'{pat}' appears {n} times")

    print('RESULT:', 'PASS' if not fails else f'FAIL ({len(fails)})')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
