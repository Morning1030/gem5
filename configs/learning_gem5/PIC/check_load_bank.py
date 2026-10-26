# Location check for load_bank_e2e.py: compares the Mats' own log (debug
# flag MatBank in trace.txt) with load_test_expected.csv. A Mat logs its
# own bank and way (= matID_in_bank), so this does not depend on the
# arbiter's address decode.
#
# 1. Writes: every expected word was written by the Mat with that bank
#    and way, at that sub-array and wordline, with that value; no Mat was
#    written anywhere else.
# 2. Reads after the LOADs: when load_bank_e2e.py reads every word (and
#    guard) back, the Mat that answers reads that value out of that
#    sub-array and wordline.
# Also: Load_ctl finished every LOAD before the first read back (debug
# flag PICLoadCtl), and each LOAD's latency in Load_ctl.
#
# Usage: python3 check_load_bank.py <load_bank_e2e output directory>
# Writes load_test_location.txt there; exits non-zero on any mismatch.

import csv
import os
import re
import sys

outdir = sys.argv[1]
# Not anchored at line start: some DPRINTFs end without a newline, so the
# next message can share its line.
RECEIVED = re.compile(r"(\d+): system\.load_ctl: client 0: load vaddr")
DONE = re.compile(r"(\d+): system\.load_ctl: client 0: load done")
FIRST_SET = re.compile(r"(\d+): system\.scheduler: Got request")
FIRST_READ = re.compile(r"(\d+): system\.mats\d+: bank \d+ way \d+: read ")
ACCESS = re.compile(r"system\.mats\d+: bank (\d+) way (\d+): (write|read) "
                    r"array (\d+) row (\d+) = (0x[0-9a-f]+|0)")

expected = {}   # words only
readback = {}   # words and guards
num_loads = 0
with open(os.path.join(outdir, 'load_test_expected.csv')) as f:
    for row in csv.DictReader(f):
        if row['load'] != 'guard':
            num_loads = max(num_loads, int(row['load']) + 1)
        key = (int(row['bank']), int(row['mat']), int(row['array']),
               int(row['wordline']))
        readback[key] = int(row['value'], 16)
        if row['load'] != 'guard':
            expected[key] = readback[key]

written = {}
read = {}
received, done, first_read, first_set = [], [], None, None
with open(os.path.join(outdir, 'trace.txt')) as f:
    for line in f:
        for regex, ticks in ((RECEIVED, received), (DONE, done)):
            m = regex.search(line)
            if m:
                ticks.append(int(m.group(1)))
        m = FIRST_SET.search(line)
        if m and first_set is None:
            first_set = int(m.group(1))
        m = FIRST_READ.search(line)
        if m and first_read is None:
            first_read = int(m.group(1))
        m = ACCESS.search(line)
        if m:
            b, w, kind, a, r, v = m.groups()
            key = (int(b), int(w), int(a), int(r))
            (written if kind == 'write' else read)[key] = int(v, 16)

missing = [k for k in expected if k not in written]
wrong = [k for k in expected if k in written and written[k] != expected[k]]
extra = [k for k in written if k not in expected]
not_read = [k for k in readback if k not in read]
read_wrong = [k for k in readback if k in read and read[k] != readback[k]]

def report(lines, name, keys, got, want):
    lines.append(f"  {name}: {len(keys)}")
    for k in sorted(keys)[:20]:
        lines.append(f"    bank {k[0]} mat {k[1]} array {k[2]} wordline "
                     f"{k[3]}: got {got.get(k, 'nothing')}, expected "
                     f"{want.get(k, 'nothing')}")


loads_ok = (len(received) == len(done) == num_loads and
            first_read is not None and done and done[-1] < first_read)
lines = [f"0. LOADs done in Load_ctl: {len(done)} of {num_loads}, last @ tick "
         f"{done[-1] if done else '-'}, first read back @ tick {first_read}"]
for i, (r, d) in enumerate(zip(received, done)):
    lines.append(f"  LOAD {i}: Load_ctl got it @ {r}, done @ {d} -> "
                 f"{(d - r) // 1000} cycles")
if first_set is not None and done:
    lines.append(f"  first SET reached the Scheduler @ {first_set}; all "
                 f"LOADs done {(done[-1] - first_set) // 1000} cycles later")
lines.append(f"1. Mat writes during the LOADs: {len(written)}, expected words: "
             f"{len(expected)}")
report(lines, "missing", missing, written, expected)
report(lines, "wrong value", wrong, written, expected)
report(lines, "written but not expected", extra, written, expected)
lines.append(f"2. Mat reads after the LOADs: {len(read)}, expected "
             f"(words + guards): {len(readback)}")
report(lines, "not read", not_read, read, readback)
report(lines, "wrong value read out", read_wrong, read, readback)
ok = loads_ok and not (missing or wrong or extra or not_read or
                       read_wrong)
lines.append("PASS: every word landed in the right bank/Mat/array/wordline "
             "and reads back from there" if ok else "FAIL")

with open(os.path.join(outdir, 'load_test_location.txt'), 'w') as f:
    f.write("\n".join(lines) + "\n")
print("\n".join(lines))
sys.exit(0 if ok else 1)
