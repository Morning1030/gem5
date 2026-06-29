# ========================
# HOW TO TEST
#
# On server:
#
#   cd ~/LEGOSIM_MICRO/gem5
#
# Compile (compute/memory_seq) benchmark:
#
#   gcc -O2 -static my_test/compute_bench.c -o my_test/compute_bench
#
# Run gem5:
#
#   ./build/X86/gem5.opt --outdir=m5out/compute_atomic my_run/run_hello.py
#
# Check output:
#
#   cat m5out/compute_atomic/simout
#
# Check stats:
#
#   grep -E "simTicks|simSeconds|simInsts|hostSeconds|hostInstRate" \
#       m5out/compute_atomic/stats.txt
#
# ========================

from pathlib import Path
import os

from gem5.simulate.simulator import Simulator

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.memory.single_channel import SingleChannelDDR3_1600

from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes

from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)

from gem5.isas import ISA
from gem5.resources.resource import BinaryResource


# ========================
# 0. Basic Configuration
# ========================
# This script is located at:
#
#   ~/LEGOSIM_MICRO/gem5/my_run/run_hello.py
#
# Therefore:
#
#   Path(__file__).resolve().parent        -> ~/LEGOSIM_MICRO/gem5/my_run
#   Path(__file__).resolve().parent.parent -> ~/LEGOSIM_MICRO/gem5
#
# GEM5_ROOT is the gem5 root directory.
GEM5_ROOT = Path(__file__).resolve().parent.parent

# Workload binary path.
# This points to:
#
#   ~/LEGOSIM_MICRO/gem5/my_test/memory_seq_bench
#
WORKLOAD = os.environ.get("WORKLOAD", "compute_bench")
BINARY_PATH = GEM5_ROOT / "my_test" / WORKLOAD


CPU_TYPE = CPUTypes.TIMING # CPUTypes.ATOMIC, CPUTypes.TIMING, CPUTypes.O3
ISA_TYPE = ISA.X86         # ISA.X86, ISA.RISCV, ISA.AARCH64    

NUM_CORES = 1
CLK_FREQ = "3GHz"
MEM_SIZE = "512MB"


L1I_SIZE = os.environ.get("L1I_SIZE", "32KiB")
L1D_SIZE = os.environ.get("L1D_SIZE", "32KiB")
L2_SIZE = os.environ.get("L2_SIZE", "256KiB")


# ========================
# 1. CPU
# ========================

processor = SimpleProcessor(
    cpu_type=CPU_TYPE,
    isa=ISA_TYPE,
    num_cores=NUM_CORES,
)


# ========================
# 2. Cache hierarchy
# ========================
# Architecture:
#
#   CPU -> L1I / L1D -> L2 -> DDR3 Memory
#
# Note:
#   PrivateL1PrivateL2CacheHierarchy does not support L3 cache.

cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size=L1D_SIZE,
    l1i_size=L1I_SIZE,
    l2_size=L2_SIZE,
)


# ========================
# 3. Memory
# ========================

memory = SingleChannelDDR3_1600(
    size=MEM_SIZE,
)


# ========================
# 4. Board
# ========================
# SimpleBoard connects CPU, cache hierarchy, and memory
# into one complete simulated computer.
board = SimpleBoard(
    clk_freq=CLK_FREQ,
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)


# ========================
# 5. Workload
# ========================
# SE mode = syscall emulation mode.
#
# In SE mode, gem5 runs a user-level binary without booting a full OS.
#
# BinaryResource expects the path to an executable binary, not a .c file.
# So make sure you already compiled:
#
#   gcc -O2 -static my_test/compute_bench.c -o my_test/compute_bench
#
print("========== gem5 run configuration ==========")
print(f"GEM5 root   : {GEM5_ROOT}")
print(f"Binary path : {BINARY_PATH}")
print(f"CPU type    : {CPU_TYPE.name}")
print(f"ISA         : {ISA_TYPE.name}")
print(f"Cores       : {NUM_CORES}")
print(f"Clock       : {CLK_FREQ}")
print(f"Memory      : {MEM_SIZE}")
print(f"L1I / L1D   : {L1I_SIZE} / {L1D_SIZE}")
print(f"L2          : {L2_SIZE}")
print("============================================")

board.set_se_binary_workload(
    BinaryResource(str(BINARY_PATH))
)


# ========================
# 6. Run simulation
# ========================
simulator = Simulator(board=board)
simulator.run()