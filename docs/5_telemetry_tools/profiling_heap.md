# Heap Profiling with DHAT

DHAT measures heap allocation behavior — how much memory is allocated, where, how often,
and how long each allocation lives. Use it to find per-tick allocation churn (short-lived
allocations that create GC-like pressure) and unexpected long-lived memory.

DHAT is a Rust allocator, not an external tool. It is compiled into the binary via a
feature flag and writes its output on clean process exit.

---

## How it works

When built with `--features dhat-heap`, the binary replaces the system allocator with
`dhat::Alloc`, which intercepts every allocation and free. On exit it writes `dhat-heap.json`
to the working directory. The file contains every allocation call site, its total bytes,
block count, and the sum of all block lifetimes, plus a full call stack for each site.

Because DHAT is compiled in, there is no external tool to install and no separate build
step — you run the binary directly with `cargo run`.

---

## Prerequisites

No installation required. The `dhat-heap` feature flag and the `profiling` Cargo profile
are already configured in the workspace `Cargo.toml`.

**`--headless` is required.** DHAT only writes its output on a clean Rust process exit
(`AppExit::Success`). A windowed run exits via OS window close, which may not flush DHAT's
buffers. Use `--headless` combined with `--duration-secs` to guarantee a clean exit.

---

## Step 1 — Run

Run from the workspace root:

```sh
cargo run --bin helios_research --profile profiling --features dhat-heap -- \
    --headless --duration-secs 30 \
    --scenario configs/scenarios/00_tutorial_showcase.toml
```

When the duration elapses the app exits cleanly and writes `dhat-heap.json` in the
directory you ran from. You do not need to build separately first — `cargo run` handles it.

---

## Step 2 — Analyze with the script

`tools/parse_dhat_profile.py` reads `dhat-heap.json` and prints an aggregated report.
Unlike the samply script, no external symbolication is needed — DHAT embeds function names
in the JSON at runtime.

```sh
# Default: top 15 allocation sites across the whole run, grouped by function
python3 tools/parse_dhat_profile.py dhat-heap.json

# Only show helios_sim call sites (most useful filter)
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim

# Only show helios_runtime call sites (config/pipeline allocations)
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_runtime

# Short-lived allocation churn — avg block lifetime under 1ms
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim --max-lifetime-ms 1.0

# Sort by block count (high block count = high allocation frequency)
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim --sort blocks

# Sort by shortest average lifetime (most transient allocations first)
python3 tools/parse_dhat_profile.py dhat-heap.json --sort lifetime

# Show more entries
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim --top 30

# Disable aggregation — show each allocation site individually instead of grouped
python3 tools/parse_dhat_profile.py dhat-heap.json --filter helios_sim --no-aggregate
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `profile` | _(required)_ | Path to `dhat-heap.json` |
| `--filter TEXT` | _(none)_ | Only show sites where a frame in the call stack contains `TEXT`. The deepest matching frame is used as the call site label. |
| `--max-lifetime-ms MS` | _(none)_ | Only show sites with average block lifetime below `MS` milliseconds. Short lifetimes indicate per-tick allocation churn. |
| `--sort bytes\|blocks\|lifetime` | `bytes` | Sort by total bytes (default), block count, or average lifetime (ascending — shortest first). |
| `--top N` | `15` | Number of sites to show. |
| `--no-aggregate` | _(off)_ | Show each PP individually instead of grouping by function. Useful when one function has many distinct call paths. |

---

## Reading the output

### Summary header

```
Peak heap:       16.78 MB  at 8.44s    ← max live heap at any moment
Heap at exit:     3.55 MB  at 8.46s    ← what was still allocated when the app exited
Total allocated: 229.94 MB in 807.0k blocks  ← cumulative over the full run
```

- **Peak heap** is the key memory budget number.
- **Heap at exit** near zero means no leaks. Residual is framework infrastructure
  (Bevy, Avian3D) that intentionally holds allocations until process exit.
- **Total allocated** is cumulative. A large total with a small peak means high allocation
  churn — many short-lived allocations being constantly created and freed.

### Per-site entries

```
    4.9k blocks      25.4ms avg    997.2 KB ( 0.4%)  peak  123.0 KB  exit      0 B
  helios_sim::simulation::config::catalog::load_catalog_from_disk  (catalog.rs:54:54)
        figment::Figment::extract
        figment::Figment::provide
    >>> helios_sim::...::load_catalog_from_disk  (catalog.rs:54:54)   ← deepest match
        core::ops::function::FnMut::call_mut
        bevy_ecs::system::function_system::run
```

| Field | Meaning |
|-------|---------|
| `blocks` | Total number of allocations at this call site |
| `avg` | Average time each block was alive. Short (< 1ms) = per-tick churn. Long (> 1s) = held data. |
| `KB/MB (%)` | Total bytes allocated, as a percentage of all allocations |
| `peak` | Bytes from this site alive at the moment of peak heap usage |
| `exit` | Bytes still alive at process exit. Non-zero = long-lived or leaked. |
| `>>>` marker | The deepest call stack frame matching `--filter` |

### Lifetime as the key signal

| Avg lifetime | What it means | Action |
|---|---|---|
| < 1ms | Per-tick churn — allocate and free within a single frame | Pre-allocate outside the loop; use a scratch buffer |
| 1ms – 1s | Short-lived working data | Usually fine; check if it can be reused |
| > 1s | Long-lived data structure | Expected for caches, buffers; confirm it's intentional |
| Lives until exit | Held for the process lifetime | Expected for resources, plugins |

---

## Note on `--filter helios_core`

`--filter helios_core` will return no results. This is correct and expected.

`helios_core` works exclusively with fixed-size nalgebra matrices on the stack.
The EKF, UKF, and A* hot paths perform zero heap allocation per tick after the P1/P2
pre-allocation work. DHAT confirms this: `helios_core` does not appear in any allocation
call stack during simulation.

---

## Known findings (2026-03-14, tutorial showcase, 30s headless run)

| Metric | Value |
|--------|-------|
| Peak heap | 16.8 MB |
| Heap at exit | 3.5 MB (Bevy/Avian3D infrastructure — not Helios code) |
| Total allocated | 229.9 MB in 807k blocks |
| helios_core per-tick allocations | **0** |

| Top helios allocator | Blocks | Avg lifetime | Total | Note |
|---|---|---|---|---|
| `load_catalog_from_disk` | 4.9k | 25ms | 997 KB | Startup only, fully freed. Config deserialization. |
| `resolve_value_recursively` | 1.3k | 1.7ms | 214 KB | TOML `$ref` resolver cloning strings. Short-lived. |
| Foxglove `run_server` | 15 | 5.3s | 150 KB | WebSocket broadcast buffer. Long-lived by design. |

---

## Browser viewer (alternative)

The raw `dhat-heap.json` can also be loaded into the DHAT browser viewer:

> https://nnethercote.github.io/dh_view/dh_view.html

Drag and drop the file into the page. The browser viewer shows the full PP (profit point)
tree with expandable children. It is useful for exploring the full call hierarchy, but
does not filter to helios frames or aggregate by function — use `parse_dhat_profile.py`
for directed analysis.
