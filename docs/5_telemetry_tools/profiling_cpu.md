# CPU Profiling with Samply

Samply is an external CPU profiler that wraps the binary and samples call stacks using
the OS performance counter. Use it to find which functions consume the most CPU time.

---

## Prerequisites

Install samply once per machine (not in `Cargo.toml`):

```sh
cargo install samply
```

Linux only — if samply fails with a permissions error, run once:

```sh
sudo sysctl kernel.perf_event_paranoid=1
```

---

## Step 1 — Build

Always build the binary separately before profiling. Do not use `samply record cargo run`
— that profiles the cargo process, not your binary.

```sh
cargo build --profile profiling --bin helios_research
```

The `profiling` Cargo profile (`inherits = "release"`, `debug = true`, `strip = false`,
`split-debuginfo = "unpacked"`) gives release-level performance with debug symbols so
samply and `parse_samply_profile.py` can resolve function names.

---

## Step 2 — Record

Run from the workspace root:

```sh
BEVY_ASSET_ROOT=./helios_sim samply record --output samply-profile.json \
    ./target/profiling/helios_research \
    --duration-secs 60 --scenario configs/scenarios/00_tutorial_showcase.toml
```

**Two non-obvious requirements:**

- **`BEVY_ASSET_ROOT=./helios_sim`** — Bevy looks for `assets/` relative to the binary
  by default, which puts it in `target/profiling/`. Setting this variable redirects it to
  `helios_sim/assets/` in the workspace root where the files actually live.
- **Profile the binary directly** — pass the binary path to samply, not `cargo run`.
  Samply wraps the process it launches; wrapping cargo means you profile cargo's startup,
  not the simulation.

Samply writes the raw (unsymbolicated) JSON to `samply-profile.json` and opens Firefox
Profiler in your browser automatically. Close the browser tab — the file is what matters.

---

## Step 3 — Analyze with the script

`tools/parse_samply_profile.py` reads the raw JSON, symbolicates addresses using `atos`
(macOS) or `llvm-symbolizer`/`addr2line` (Linux), and prints a terminal report.

```sh
# Default: top 15 functions by CPU time across all threads
python3 tools/parse_samply_profile.py samply-profile.json

# Narrow to helios code only
python3 tools/parse_samply_profile.py samply-profile.json --filter helios

# Narrow to a specific crate
python3 tools/parse_samply_profile.py samply-profile.json --filter helios_core
python3 tools/parse_samply_profile.py samply-profile.json --filter helios_sim

# Show more entries
python3 tools/parse_samply_profile.py samply-profile.json --top 30

# Combine filters
python3 tools/parse_samply_profile.py samply-profile.json --filter helios_sim --top 20
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `profile` | _(required)_ | Path to `samply-profile.json` |
| `--filter TEXT` | _(none)_ | Only show functions whose name contains `TEXT` (case-insensitive) |
| `--top N` | `15` | Number of functions to show per thread |

---

## Reading the output

The report is organized per thread. Each thread shows two tables.

**Self CPU** — time spent *inside* the function itself, not in callees. This is where the
CPU actually is. A high self-CPU function is a direct optimization target.

**Total CPU** — time spent in the function *including* all functions it calls. High total
CPU with low self CPU means the function is a hot call path but the work is delegated
further down the stack.

```
THREAD: main (tid=12345, 1200 samples, 59.8s CPU)
------------------------------------------------------------------------
  Self CPU  (hottest leaf — where the CPU actually is)
  Function                                               Self        %
  ------------------------------------------------------ ---------- ------
  helios_sim::draw_occupancy_grid                         12.3s    20.5%
  nalgebra::matrix_multiply                                4.1s     6.8%
  ...

  Total CPU  (inclusive — hot call paths)
  Function                                               Total       %
  ------------------------------------------------------ ---------- ------
  helios_sim::AutonomyPlugin::run_estimation              22.1s    36.9%
  ...
```

---

## Known findings (2026-03-14, tutorial showcase, single agent)

| Function | Self CPU | Note |
|----------|----------|------|
| `draw_occupancy_grid` | ~20% | Iterates every cell every draw call. Primary optimization target (P2-20). |
| `TopicBus::publish` | ~4% | Measurable at single-agent scale. Will grow linearly with agent count. |
| EKF predict/update | <1% | Fast after P1 pre-allocation work. Not a concern. |
| A* replan | <1% | Fast after P2 `AStarSearchBuffers` pre-allocation. Not a concern. |

---

## Browser viewer (alternative)

Samply also opens Firefox Profiler automatically after recording. This gives an
interactive flame graph and timeline view. The `parse_samply_profile.py` script is better
for quick terminal comparisons and filtering to helios code; the browser is better for
exploring call hierarchies visually.
