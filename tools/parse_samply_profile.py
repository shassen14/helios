#!/usr/bin/env python3
"""
Parse a samply (Firefox Profiler format) JSON profile and report hotspots.

Automatically symbolicates using the best available tool for the platform:
  - macOS: atos  (ships with Xcode command-line tools)
  - Linux: addr2line or llvm-symbolizer  (install: apt install binutils / llvm)

Samply always saves unsymbolicated JSON to disk; symbolication happens here.

Usage:
    python3 tools/parse_samply_profile.py samply-profile.json
    python3 tools/parse_samply_profile.py samply-profile.json --top 20
    python3 tools/parse_samply_profile.py samply-profile.json --filter helios
    python3 tools/parse_samply_profile.py samply-profile.json --filter helios_core

Profiling workflow (run from workspace root):
    cargo build --profile profiling --bin helios_research

    # macOS:
    BEVY_ASSET_ROOT=./helios_sim samply record --output samply-profile.json \\
        ./target/profiling/helios_research -- \\
        --duration-secs 60 --scenario configs/scenarios/00_tutorial_showcase.toml

    # Linux (samply uses perf internally — may need: sudo sysctl kernel.perf_event_paranoid=1):
    BEVY_ASSET_ROOT=./helios_sim samply record --output samply-profile.json \\
        ./target/profiling/helios_research -- \\
        --duration-secs 60 --scenario configs/scenarios/00_tutorial_showcase.toml
"""

import json
import sys
import argparse
import platform
import shutil
import subprocess
import re
from collections import defaultdict
from typing import Optional

MACOS_ARM64_BASE = 0x100000000
BATCH_SIZE = 500


def load_profile(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def _symbolicate_atos(binary_path: str, arch: str, addresses: list[int]) -> dict[int, str]:
    """macOS: use atos with the arm64 load base."""
    base = MACOS_ARM64_BASE
    result = {}
    for i in range(0, len(addresses), BATCH_SIZE):
        batch = addresses[i:i + BATCH_SIZE]
        cmd = ["atos", "-o", binary_path, "-arch", arch, "-l", hex(base)] + [hex(base + a) for a in batch]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            for addr, line in zip(batch, proc.stdout.strip().splitlines()):
                result[addr] = line.strip()
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
    return result


def _symbolicate_addr2line(binary_path: str, addresses: list[int]) -> dict[int, str]:
    """Linux: use llvm-symbolizer (preferred) or addr2line."""
    tool = "llvm-symbolizer" if shutil.which("llvm-symbolizer") else "addr2line"
    result = {}
    for i in range(0, len(addresses), BATCH_SIZE):
        batch = addresses[i:i + BATCH_SIZE]
        if tool == "llvm-symbolizer":
            cmd = ["llvm-symbolizer", "--obj", binary_path, "--output-style=GNU"] + [hex(a) for a in batch]
        else:
            cmd = ["addr2line", "-f", "-C", "-e", binary_path] + [hex(a) for a in batch]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            lines = [l.strip() for l in proc.stdout.strip().splitlines() if l.strip()]
            # addr2line outputs two lines per address (func name + file:line); take every other
            if tool == "addr2line":
                func_lines = lines[0::2]
                for addr, name in zip(batch, func_lines):
                    result[addr] = name if name != "??" else hex(addr)
            else:
                for addr, name in zip(batch, lines):
                    result[addr] = name if name not in ("??", "?") else hex(addr)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
    return result


def symbolicate_addresses(binary_path: str, arch: str, addresses: list[int]) -> dict[int, str]:
    """Resolve relative addresses to symbol names using the best available tool."""
    if not addresses:
        return {}
    if platform.system() == "Darwin":
        result = _symbolicate_atos(binary_path, arch, addresses)
    else:
        result = _symbolicate_addr2line(binary_path, addresses)
    # Fill any misses with hex
    for addr in addresses:
        if addr not in result:
            result[addr] = hex(addr)
    return result


def build_symbol_map(profile: dict) -> dict[str, dict[int, str]]:
    """
    For each lib that has a local binary, symbolicate all frame addresses that
    belong to it. Returns {lib_name: {raw_addr: symbol_str}}.
    """
    libs = profile.get("libs", [])
    symbol_map: dict[str, dict[int, str]] = {}

    for lib in libs:
        path = lib.get("path", "")
        name = lib.get("name", "")
        arch = lib.get("arch", "arm64").rstrip("e")  # arm64e -> arm64

        # Only attempt local binaries (skip system dylibs)
        if not path or path.startswith("/usr/") or path.startswith("/System/"):
            continue

        symbol_map[name] = {"_path": path, "_arch": arch}

    # Collect all addresses per lib across all threads
    lib_addresses: dict[str, set] = defaultdict(set)
    for thread in profile["threads"]:
        ft = thread["frameTable"]
        func_table = thread["funcTable"]
        rt = thread["resourceTable"]

        resource_to_lib_idx = rt["lib"]
        func_to_resource = func_table["resource"]
        frame_to_func = ft["func"]
        frame_addresses = ft["address"]

        for frame_idx in range(ft["length"]):
            func_idx = frame_to_func[frame_idx]
            res_idx = func_to_resource[func_idx]
            if res_idx is None or res_idx < 0 or res_idx >= len(resource_to_lib_idx):
                continue
            lib_idx = resource_to_lib_idx[res_idx]
            if lib_idx is None or lib_idx >= len(libs):
                continue
            lib_name = libs[lib_idx]["name"]
            if lib_name in symbol_map:
                addr = frame_addresses[frame_idx]
                if addr is not None:
                    lib_addresses[lib_name].add(addr)

    # Symbolicate each lib's addresses
    for lib_name, addrs in lib_addresses.items():
        if lib_name not in symbol_map:
            continue
        path = symbol_map[lib_name]["_path"]
        arch = symbol_map[lib_name]["_arch"]
        resolved = symbolicate_addresses(path, arch, sorted(addrs))
        symbol_map[lib_name].update(resolved)

    return symbol_map


def demangle(name: str) -> str:
    """Light cleanup: strip crate hash suffixes and (in helios_research) module paths."""
    # Remove hash suffix like ::h1a2b3c4d5e6f
    name = re.sub(r"::h[0-9a-f]{16}", "", name)
    # atos sometimes leaves " (in binary_name)" suffix
    name = re.sub(r"\s+\(in \S+\)", "", name)
    return name.strip()


def resolve_frame_name(
    frame_idx: int,
    thread: dict,
    libs: list,
    symbol_map: dict[str, dict[int, str]],
) -> str:
    ft = thread["frameTable"]
    func_table = thread["funcTable"]
    rt = thread["resourceTable"]
    sa = thread["stringArray"]

    func_idx = ft["func"][frame_idx]
    raw_name = sa[func_table["name"][func_idx]]

    res_idx = func_table["resource"][func_idx]
    if res_idx is None or res_idx < 0 or res_idx >= len(rt["lib"]):
        return raw_name

    lib_idx = rt["lib"][res_idx]
    if lib_idx is None or lib_idx >= len(libs):
        return raw_name

    lib_name = libs[lib_idx]["name"]
    addr = ft["address"][frame_idx]
    if addr is not None and lib_name in symbol_map and addr in symbol_map[lib_name]:
        return demangle(symbol_map[lib_name][addr])

    return raw_name


def parse_thread(
    thread: dict,
    libs: list,
    symbol_map: dict[str, dict[int, str]],
    filter_str: Optional[str],
) -> dict:
    stack_table = thread["stackTable"]
    samples = thread["samples"]

    stack_frames = stack_table["frame"]
    stack_prefixes = stack_table["prefix"]
    cpu_deltas = samples.get("threadCPUDelta", [])
    stack_ids = samples.get("stack", [])
    total_cpu = sum(x for x in cpu_deltas if x is not None)

    self_cpu: dict[str, float] = defaultdict(float)
    total_cpu_per_func: dict[str, float] = defaultdict(float)

    for i, stack_id in enumerate(stack_ids):
        if stack_id is None:
            continue
        cpu = (cpu_deltas[i] if cpu_deltas and cpu_deltas[i] is not None else 1)

        # Self: leaf frame only
        leaf_frame = stack_frames[stack_id]
        leaf_name = resolve_frame_name(leaf_frame, thread, libs, symbol_map)
        if filter_str is None or filter_str.lower() in leaf_name.lower():
            self_cpu[leaf_name] += cpu

        # Total: walk full stack, deduplicate per sample
        seen: set = set()
        sid = stack_id
        while sid is not None:
            fid = stack_frames[sid]
            if fid not in seen:
                seen.add(fid)
                fname = resolve_frame_name(fid, thread, libs, symbol_map)
                if filter_str is None or filter_str.lower() in fname.lower():
                    total_cpu_per_func[fname] += cpu
            sid = stack_prefixes[sid]

    return {
        "name": thread["name"],
        "tid": thread["tid"],
        "num_samples": len(stack_ids),
        "total_cpu_us": total_cpu,
        "self_cpu": dict(self_cpu),
        "total_cpu": dict(total_cpu_per_func),
    }


def format_us(us: float) -> str:
    if us >= 1_000_000:
        return f"{us / 1_000_000:.2f}s"
    if us >= 1_000:
        return f"{us / 1_000:.1f}ms"
    return f"{us:.0f}µs"


def print_report(profile: dict, top_n: int = 15, filter_str: Optional[str] = None) -> None:
    meta = profile["meta"]
    threads = profile["threads"]
    libs = profile.get("libs", [])

    print("=" * 72)
    print("SAMPLY PROFILE REPORT")
    print("=" * 72)
    print(f"Product:     {meta.get('product', '?')}")
    print(f"Sample rate: {meta.get('interval', '?')} ms")
    print(f"Threads:     {len(threads)}")
    if filter_str:
        print(f"Filter:      '{filter_str}'")

    print("\nSymbolicating... ", end="", flush=True)
    symbol_map = build_symbol_map(profile)
    symbolicated_libs = [n for n in symbol_map if not n.startswith("_")]
    print(f"done ({', '.join(symbolicated_libs) or 'none'})")
    print()

    results = [parse_thread(t, libs, symbol_map, filter_str) for t in threads]

    # Thread summary
    print("THREAD SUMMARY")
    print("-" * 72)
    total_all = sum(r["total_cpu_us"] for r in results)
    for r in sorted(results, key=lambda x: x["total_cpu_us"], reverse=True):
        pct = (r["total_cpu_us"] / total_all * 100) if total_all else 0
        print(f"  {r['name']:<42} {format_us(r['total_cpu_us']):>10}  {pct:5.1f}%  ({r['num_samples']} samples)")
    print(f"  {'TOTAL':<42} {format_us(total_all):>10}")
    print()

    # Per-thread breakdown
    for r in sorted(results, key=lambda x: x["total_cpu_us"], reverse=True):
        if r["num_samples"] == 0 or not r["self_cpu"]:
            continue

        print(f"THREAD: {r['name']} (tid={r['tid']}, {r['num_samples']} samples, {format_us(r['total_cpu_us'])} CPU)")
        print("-" * 72)
        thread_total = r["total_cpu_us"] or 1

        # Self time
        sorted_self = sorted(r["self_cpu"].items(), key=lambda x: x[1], reverse=True)
        print("  Self CPU  (hottest leaf — where the CPU actually is)")
        print(f"  {'Function':<54} {'Self':>10} {'%':>6}")
        print(f"  {'-'*54} {'-'*10} {'-'*6}")
        for name, cpu in sorted_self[:top_n]:
            display = name if len(name) <= 54 else "..." + name[-51:]
            print(f"  {display:<54} {format_us(cpu):>10} {cpu/thread_total*100:>5.1f}%")

        # Total time
        sorted_total = sorted(r["total_cpu"].items(), key=lambda x: x[1], reverse=True)
        print()
        print("  Total CPU  (inclusive — hot call paths)")
        print(f"  {'Function':<54} {'Total':>10} {'%':>6}")
        print(f"  {'-'*54} {'-'*10} {'-'*6}")
        for name, cpu in sorted_total[:top_n]:
            display = name if len(name) <= 54 else "..." + name[-51:]
            print(f"  {display:<54} {format_us(cpu):>10} {cpu/thread_total*100:>5.1f}%")
        print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("profile", help="Path to samply-profile.json")
    parser.add_argument("--top", type=int, default=15, help="Top N functions per thread (default: 15)")
    parser.add_argument("--filter", type=str, default=None, help="Only show functions containing this string")
    args = parser.parse_args()

    profile = load_profile(args.profile)
    print_report(profile, top_n=args.top, filter_str=args.filter)


if __name__ == "__main__":
    main()
