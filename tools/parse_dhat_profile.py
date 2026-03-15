#!/usr/bin/env python3
"""
Parse a DHAT heap profile (dhat-heap.json) and report allocation hotspots.

Unlike the samply parser, no external symbolication tool is needed — DHAT embeds
function names directly in its JSON output at the time of the run.

Usage:
    python3 tools/parse_dhat_profile.py dhat-heap.json
    python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim
    python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim --max-lifetime-ms 1.0
    python3 tools/parse_dhat_profile.py dhat-heap.json --sort blocks
    python3 tools/parse_dhat_profile.py dhat-heap.json --top 20

Profiling workflow (run from workspace root):
    cargo run --bin helios_research --profile profiling --features dhat-heap -- \\
      --headless --duration-secs 30 \\
      --scenario configs/scenarios/00_tutorial_showcase.toml

    # Output written to: dhat-heap.json (in the directory you ran from)
    # View in browser: https://nnethercote.github.io/dh_view/dh_view.html

Notes on frame filtering:
    Use --filter helios_sim  to see allocations originating in helios_sim code.
    Use --filter helios_runtime  for config/pipeline allocations.
    helios_core rarely allocates during simulation (it works on fixed-size matrices),
    so --filter helios_core will typically return no results — that is expected and good.
"""

import json
import argparse
import re
from typing import Optional


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_profile(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Frame parsing
# ---------------------------------------------------------------------------

# DHAT frame string formats:
#   "0x1234abcd: some::module::function (src/file.rs:42:10)"
#   "0x1234abcd: some::module::function (???:0:0)"   <- no source location
#   "[root]"
_FRAME_RE = re.compile(r"^0x[0-9a-f]+:\s+(.+?)(?:\s+\(([^)]+)\))?$")


def parse_frame(raw: str) -> tuple[str, str]:
    """Return (function_name, source_location). Location is '' if unknown."""
    if raw == "[root]":
        return "[root]", ""
    m = _FRAME_RE.match(raw)
    if not m:
        return raw, ""
    func = m.group(1).strip()
    loc = m.group(2) or ""
    if loc == "???:0:0":
        loc = ""
    # Strip Rust hash suffixes like ::h1a2b3c4d5e6f789
    func = re.sub(r"::h[0-9a-f]{16}$", "", func)
    return func, loc


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def fmt_bytes(b: int) -> str:
    if b >= 1_000_000_000:
        return f"{b / 1_000_000_000:.2f} GB"
    if b >= 1_000_000:
        return f"{b / 1_000_000:.2f} MB"
    if b >= 1_000:
        return f"{b / 1_000:.1f} KB"
    return f"{b} B"


def fmt_lifetime(us: float) -> str:
    if us >= 1_000_000:
        return f"{us / 1_000_000:.2f}s"
    if us >= 1_000:
        return f"{us / 1_000:.1f}ms"
    return f"{us:.0f}µs"


def fmt_count(n: int) -> str:
    if n >= 1_000_000:
        return f"{n / 1_000_000:.2f}M"
    if n >= 1_000:
        return f"{n / 1_000:.1f}k"
    return str(n)


def _truncate(s: str, width: int) -> str:
    return s if len(s) <= width else "..." + s[-(width - 3):]


# ---------------------------------------------------------------------------
# PP processing
# ---------------------------------------------------------------------------

def _find_match(fs: list[int], ftbl: list[str], filter_str: str) -> Optional[tuple[int, int]]:
    """Walk the call stack leaf→root. Return (stack_pos, frame_idx) for the
    deepest frame matching filter_str, or None."""
    for pos, fi in enumerate(fs):
        if filter_str.lower() in ftbl[fi].lower():
            return pos, fi
    return None


def build_records(
    profile: dict,
    filter_str: Optional[str],
    max_lifetime_ms: Optional[float],
) -> list[dict]:
    """Build a flat list of processed PP records, optionally filtered."""
    ftbl = profile["ftbl"]
    records = []

    for pp in profile["pps"]:
        tbk = pp.get("tbk", 0)
        if tbk == 0:
            continue

        tl = pp.get("tl", 0)
        avg_us = tl / tbk

        if max_lifetime_ms is not None and avg_us / 1000 > max_lifetime_ms:
            continue

        fs = pp.get("fs", [])

        if filter_str:
            result = _find_match(fs, ftbl, filter_str)
            if result is None:
                continue
            match_pos, match_fi = result
        else:
            match_pos = 0
            match_fi = fs[0] if fs else 0

        records.append({
            "tb": pp.get("tb", 0),
            "tbk": tbk,
            "mb": pp.get("mb", 0),
            "eb": pp.get("eb", 0),
            "avg_us": avg_us,
            "match_pos": match_pos,
            "match_fi": match_fi,
            "fs": fs,
        })

    return records


def aggregate_records(records: list[dict], ftbl: list[str]) -> list[dict]:
    """Merge PPs that share the same function at their match frame."""
    buckets: dict[str, dict] = {}

    for r in records:
        func, loc = parse_frame(ftbl[r["match_fi"]])
        key = func  # aggregate by function name

        if key not in buckets:
            buckets[key] = {
                "tb": 0, "tbk": 0, "mb": 0, "eb": 0,
                "total_tl": 0, "count": 0,
                "func": func, "loc": loc,
                "sample_fs": r["fs"],
                "sample_match_pos": r["match_pos"],
                "sample_match_fi": r["match_fi"],
            }

        b = buckets[key]
        b["tb"] += r["tb"]
        b["tbk"] += r["tbk"]
        b["mb"] += r["mb"]
        b["eb"] += r["eb"]
        b["total_tl"] += r["avg_us"] * r["tbk"]  # reconstruct tl
        b["count"] += 1
        # Keep the sample with the most blocks for stack context
        if r["tbk"] > buckets[key].get("_best_tbk", 0):
            b["_best_tbk"] = r["tbk"]
            b["sample_fs"] = r["fs"]
            b["sample_match_pos"] = r["match_pos"]
            b["sample_match_fi"] = r["match_fi"]

    result = []
    for b in buckets.values():
        b["avg_us"] = b["total_tl"] / b["tbk"] if b["tbk"] else 0
        result.append(b)
    return result


# ---------------------------------------------------------------------------
# Report rendering
# ---------------------------------------------------------------------------

def _sort_key(sort_by: str):
    if sort_by == "blocks":
        return lambda r: r["tbk"]
    if sort_by == "lifetime":
        return lambda r: r["avg_us"]
    return lambda r: r["tb"]  # bytes (default)


def _sort_reverse(sort_by: str) -> bool:
    return sort_by != "lifetime"


def print_report(
    profile: dict,
    top_n: int,
    filter_str: Optional[str],
    max_lifetime_ms: Optional[float],
    sort_by: str,
    aggregate: bool,
) -> None:
    ftbl = profile["ftbl"]
    all_pps = profile["pps"]

    total_tb = sum(pp.get("tb", 0) for pp in all_pps)
    total_tbk = sum(pp.get("tbk", 0) for pp in all_pps if pp.get("tbk", 0))
    peak_mb = max((pp.get("mb", 0) for pp in all_pps), default=0)
    total_eb = sum(pp.get("eb", 0) for pp in all_pps)
    tg = profile.get("tg", 0)
    te = profile.get("te", 0)

    print("=" * 72)
    print("DHAT HEAP PROFILE REPORT")
    print("=" * 72)
    print(f"Command:         {profile.get('cmd', '?')}")
    print(f"Mode:            {profile.get('mode', '?')}")
    print(f"Peak heap:       {fmt_bytes(peak_mb)}  at {tg / 1_000_000:.2f}s")
    print(f"Heap at exit:    {fmt_bytes(total_eb)}  at {te / 1_000_000:.2f}s")
    print(f"Total allocated: {fmt_bytes(total_tb)} in {fmt_count(total_tbk)} blocks")
    if filter_str:
        print(f"Filter:          '{filter_str}'")
    if max_lifetime_ms is not None:
        print(f"Max avg lifetime: {max_lifetime_ms:.1f}ms")
    print(f"Sort:            {sort_by}")
    print()

    records = build_records(profile, filter_str, max_lifetime_ms)

    if not records:
        print("No matching allocation sites found.")
        if filter_str:
            print(f"\nNote: '{filter_str}' not found in any call stack.")
            print("  helios_core rarely allocates during simulation (works with fixed-size")
            print("  matrices). Try --filter helios_sim or --filter helios_runtime.")
        return

    if aggregate:
        display_records = aggregate_records(records, ftbl)
        mode_label = "AGGREGATED "
    else:
        display_records = [
            dict(r, func=parse_frame(ftbl[r["match_fi"]])[0],
                 loc=parse_frame(ftbl[r["match_fi"]])[1],
                 sample_fs=r["fs"],
                 sample_match_pos=r["match_pos"],
                 sample_match_fi=r["match_fi"])
            for r in records
        ]
        mode_label = ""

    display_records.sort(key=_sort_key(sort_by), reverse=_sort_reverse(sort_by))

    lifetime_label = f"(avg lifetime < {max_lifetime_ms:.1f}ms) " if max_lifetime_ms is not None else ""
    print(
        f"TOP {min(top_n, len(display_records))} {mode_label}"
        f"{lifetime_label}ALLOCATION SITES  "
        f"({len(display_records)} {'groups' if aggregate else 'sites'} matched)"
    )
    print("-" * 72)

    for rec in display_records[:top_n]:
        func = rec["func"]
        loc = rec.get("loc", "")
        pct_bytes = rec["tb"] / total_tb * 100 if total_tb else 0
        loc_str = f"  ({loc})" if loc else ""

        print(
            f"  {fmt_count(rec['tbk']):>8} blocks  "
            f"{fmt_lifetime(rec['avg_us']):>10} avg  "
            f"{fmt_bytes(rec['tb']):>10} ({pct_bytes:4.1f}%)  "
            f"peak {fmt_bytes(rec['mb']):>9}  "
            f"exit {fmt_bytes(rec['eb']):>8}"
        )
        print(f"  {_truncate(func, 68)}{loc_str}")

        # Show call stack context around the match frame
        fs = rec["sample_fs"]
        match_pos = rec["sample_match_pos"]
        ctx_start = max(0, match_pos - 2)
        ctx_end = min(len(fs), match_pos + 4)
        for i, fi in enumerate(fs[ctx_start:ctx_end], start=ctx_start):
            fn, floc = parse_frame(ftbl[fi])
            marker = ">>>" if i == match_pos else "   "
            fn_str = _truncate(fn, 60)
            floc_str = f"  ({floc})" if floc else ""
            print(f"    {marker} {fn_str}{floc_str}")
        print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("profile", help="Path to dhat-heap.json")
    parser.add_argument(
        "--top", type=int, default=15,
        help="Show top N allocation sites (default: 15)",
    )
    parser.add_argument(
        "--filter", type=str, default=None,
        help="Only show sites where a call stack frame contains this string "
             "(e.g. 'helios_sim', 'helios_runtime'). "
             "The deepest matching frame is used as the call site label.",
    )
    parser.add_argument(
        "--max-lifetime-ms", type=float, default=None,
        metavar="MS",
        help="Only show sites with average block lifetime below this threshold (ms). "
             "Short lifetimes indicate per-tick allocation churn.",
    )
    parser.add_argument(
        "--sort", choices=["bytes", "blocks", "lifetime"], default="bytes",
        help="Sort order: bytes (default), blocks, or lifetime (ascending — shortest first)",
    )
    parser.add_argument(
        "--no-aggregate", action="store_true",
        help="Show each PP individually instead of grouping by function name. "
             "Useful when a single function has many distinct call paths.",
    )
    args = parser.parse_args()

    profile = load_profile(args.profile)
    print_report(
        profile,
        top_n=args.top,
        filter_str=args.filter,
        max_lifetime_ms=args.max_lifetime_ms,
        sort_by=args.sort,
        aggregate=not args.no_aggregate,
    )


if __name__ == "__main__":
    main()
