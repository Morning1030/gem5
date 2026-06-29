#!/usr/bin/env bash
set -euo pipefail

cd ~/LEGOSIM_MICRO/gem5

GEM5=./build/X86/gem5.opt
SCRIPT=my_run/run_hello.py

mkdir -p logs

# Make sure binaries exist.
if [ ! -f my_test/compute_bench ]; then
    echo "[ERROR] my_tests/compute_bench not found."
    echo "Compile it first:"
    echo "  gcc -O2 -static my_test/compute_bench.c -o my_test/compute_bench"
    exit 1
fi

if [ ! -f my_test/memory_seq_bench ]; then
    echo "[ERROR] my_tests/memory_seq_bench not found."
    echo "Compile it first:"
    echo "  gcc -O2 -static my_test/memory_seq.c -o my_test/memory_seq_bench"
    exit 1
fi

run_one() {
    local workload=$1
    local case_name=$2
    local l1i=$3
    local l1d=$4
    local l2=$5

    local outdir="m5out/${workload}_timing_${case_name}"
    local logfile="logs/${workload}_timing_${case_name}.log"

    echo "=================================================="
    echo "Running: workload=${workload}, case=${case_name}"
    echo "L1I=${l1i}, L1D=${l1d}, L2=${l2}"
    echo "outdir=${outdir}"
    echo "log=${logfile}"
    echo "=================================================="

    WORKLOAD=${workload} \
    L1I_SIZE=${l1i} \
    L1D_SIZE=${l1d} \
    L2_SIZE=${l2} \
    ${GEM5} --outdir=${outdir} ${SCRIPT} > ${logfile} 2>&1

    echo "[DONE] ${workload} ${case_name}"
}

# Cache settings:
# case_name  L1I    L1D    L2
run_all_for_workload() {
    local workload=$1

    run_one ${workload} small    16KiB 16KiB 128KiB
    run_one ${workload} baseline 32KiB 32KiB 256KiB
    run_one ${workload} medium   64KiB 64KiB 512KiB
    run_one ${workload} large    64KiB 64KiB 1MiB
}

run_all_for_workload compute_bench
run_all_for_workload memory_seq_bench

echo "All cache experiments finished."
