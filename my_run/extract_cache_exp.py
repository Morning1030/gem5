from pathlib import Path
import csv
import re

GEM5_ROOT = Path(__file__).resolve().parent.parent

# ========================
# Experiment configuration
# ========================

CASES = [
    {
        "case": "small",
        "l1i": "16KiB",
        "l1d": "16KiB",
        "l2": "128KiB",
    },
    {
        "case": "baseline",
        "l1i": "32KiB",
        "l1d": "32KiB",
        "l2": "256KiB",
    },
    {
        "case": "medium",
        "l1i": "64KiB",
        "l1d": "64KiB",
        "l2": "512KiB",
    },
    {
        "case": "large",
        "l1i": "64KiB",
        "l1d": "64KiB",
        "l2": "1MiB",
    },
]

WORKLOADS = [
    "compute_bench",
    "memory_seq_bench",
]

CPU_MODEL = "Timing"


# ========================
# Stats to extract
# ========================

STAT_KEYS = {
    # Overall performance
    "simInsts": "simInsts",
    "simOps": "simOps",
    "simTicks": "simTicks",
    "simSeconds": "simSeconds",
    "hostSeconds": "hostSeconds",
    "hostInstRate": "hostInstRate",

    # CPU
    "numCycles": "board.processor.cores.core.numCycles",
    "ipc": "board.processor.cores.core.ipc",

    # L1D
    "l1d_accesses": "board.cache_hierarchy.l1dcaches.demandAccesses::total",
    "l1d_misses": "board.cache_hierarchy.l1dcaches.demandMisses::total",
    "l1d_miss_rate": "board.cache_hierarchy.l1dcaches.demandMissRate::total",

    # L1I
    "l1i_accesses": "board.cache_hierarchy.l1icaches.demandAccesses::total",
    "l1i_misses": "board.cache_hierarchy.l1icaches.demandMisses::total",
    "l1i_miss_rate": "board.cache_hierarchy.l1icaches.demandMissRate::total",

    # L2
    "l2_accesses": "board.cache_hierarchy.l2caches.demandAccesses::total",
    "l2_misses": "board.cache_hierarchy.l2caches.demandMisses::total",
    "l2_miss_rate": "board.cache_hierarchy.l2caches.demandMissRate::total",
}


def parse_stats(stats_path):
    stats = {}

    with open(stats_path, "r") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            # Typical gem5 stat line:
            # stat.name    value    # comment
            parts = line.split()
            if len(parts) < 2:
                continue

            key = parts[0]
            value = parts[1]

            for out_name, gem5_key in STAT_KEYS.items():
                if key == gem5_key:
                    try:
                        stats[out_name] = float(value)
                    except ValueError:
                        stats[out_name] = value

    return stats


def sci(x):
    if x == "" or x is None:
        return ""
    if isinstance(x, str):
        return x
    return f"{x:.6e}"


def fixed(x, digits=6):
    if x == "" or x is None:
        return ""
    if isinstance(x, str):
        return x
    return f"{x:.{digits}f}"


def main():
    rows = []

    for workload in WORKLOADS:
        for case in CASES:
            case_name = case["case"]
            outdir = GEM5_ROOT / "m5out" / f"{workload}_timing_{case_name}"
            stats_path = outdir / "stats.txt"

            if not stats_path.exists():
                print(f"[WARN] Missing stats.txt: {stats_path}")
                continue

            stats = parse_stats(stats_path)

            ipc = stats.get("ipc", "")
            cpi = ""
            if isinstance(ipc, float) and ipc != 0:
                cpi = 1.0 / ipc

            row = {
                "workload": workload,
                "cpu_model": CPU_MODEL,
                "case": case_name,
                "l1i_size": case["l1i"],
                "l1d_size": case["l1d"],
                "l2_size": case["l2"],

                "simInsts": stats.get("simInsts", ""),
                "simOps": stats.get("simOps", ""),
                "simTicks": stats.get("simTicks", ""),
                "simSeconds": stats.get("simSeconds", ""),
                "numCycles": stats.get("numCycles", ""),
                "ipc": ipc,
                "cpi": cpi,
                "hostSeconds": stats.get("hostSeconds", ""),
                "hostInstRate": stats.get("hostInstRate", ""),

                "l1d_accesses": stats.get("l1d_accesses", ""),
                "l1d_misses": stats.get("l1d_misses", ""),
                "l1d_miss_rate": stats.get("l1d_miss_rate", ""),
                "l1d_miss_rate_percent": stats.get("l1d_miss_rate", "") * 100
                    if isinstance(stats.get("l1d_miss_rate", ""), float) else "",

                "l1i_accesses": stats.get("l1i_accesses", ""),
                "l1i_misses": stats.get("l1i_misses", ""),
                "l1i_miss_rate": stats.get("l1i_miss_rate", ""),
                "l1i_miss_rate_percent": stats.get("l1i_miss_rate", "") * 100
                    if isinstance(stats.get("l1i_miss_rate", ""), float) else "",

                "l2_accesses": stats.get("l2_accesses", ""),
                "l2_misses": stats.get("l2_misses", ""),
                "l2_miss_rate": stats.get("l2_miss_rate", ""),
                "l2_miss_rate_percent": stats.get("l2_miss_rate", "") * 100
                    if isinstance(stats.get("l2_miss_rate", ""), float) else "",
            }

            rows.append(row)

    output_csv = GEM5_ROOT / "my_run" / "cache_exp_summary.csv"

    fieldnames = [
        "workload",
        "cpu_model",
        "case",
        "l1i_size",
        "l1d_size",
        "l2_size",

        "simInsts",
        "simOps",
        "simTicks",
        "simSeconds",
        "numCycles",
        "ipc",
        "cpi",
        "hostSeconds",
        "hostInstRate",

        "l1d_accesses",
        "l1d_misses",
        "l1d_miss_rate",
        "l1d_miss_rate_percent",

        "l1i_accesses",
        "l1i_misses",
        "l1i_miss_rate",
        "l1i_miss_rate_percent",

        "l2_accesses",
        "l2_misses",
        "l2_miss_rate",
        "l2_miss_rate_percent",
    ]

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[OK] Wrote CSV: {output_csv}")
    print()
    print("Quick summary:")
    print("workload,case,L1I,L1D,L2,simSeconds,IPC,CPI,L1D miss %,L2 miss %")

    for row in rows:
        print(
            f"{row['workload']},"
            f"{row['case']},"
            f"{row['l1i_size']},"
            f"{row['l1d_size']},"
            f"{row['l2_size']},"
            f"{fixed(row['simSeconds'])},"
            f"{fixed(row['ipc'])},"
            f"{fixed(row['cpi'])},"
            f"{fixed(row['l1d_miss_rate_percent'])},"
            f"{fixed(row['l2_miss_rate_percent'])}"
        )


if __name__ == "__main__":
    main()
