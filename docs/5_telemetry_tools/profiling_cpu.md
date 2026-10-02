# CPU Profiling with Samply

Samply is an external CPU profiler that wraps a binary and samples its call
stacks. Use it to find which functions consume the most CPU time. It shows the
result in the Firefox Profiler in your browser: a flame graph, a call tree with
self and total time, and a timeline per thread.

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

Build the binary with the `profiling` Cargo profile (`inherits = "release"`,
`debug = true`, `strip = false`, `split-debuginfo = "unpacked"`): release speed,
with the debug symbols samply needs to name functions.

```sh
# The player, for a run paced like a real session:
cargo build --profile profiling -p helios_sim --bin helios_play

# The test harness, for a run that ends by itself:
cargo build --profile profiling -p helios_test --features sim --bin helios_test_sim
```

Profile the built binary, never `samply record cargo run`: samply profiles the
process it launches, so that would profile cargo.

---

## Step 2 — Record

Run from the workspace root. Both binaries find `helios_sim/assets/` on their
own; no environment variable is needed.

**Paced like a real session** (answers "how much CPU does the sim use?"):

```sh
samply record ./target/profiling/helios_play --headless --speed 1.0
```

A headless run exits after the scenario's `[simulation] duration_seconds` of
simulated time (180 s for the proving ground), and samply then saves the
profile and opens it in the browser. Press **Ctrl+C** to stop sooner; samply
still saves what it recorded, and 30–60 s is plenty. Leave out `--headless` to
profile the windowed app, rendering included; a window runs until you close
it.

**A fixed run that ends by itself** (answers "where does a tick's time go?"):

```sh
samply record ./target/profiling/helios_test_sim --run configs/test/runs/proving_ground.toml
```

The harness runs unpaced, as fast as the machine allows: the run's 60
simulated seconds take about 10 s of wall time. Good for finding hot
functions; not for CPU percentages, since an unpaced run uses all the CPU it
can.

Both default to the proving-ground scenario. Pass `--scenario <path>` to
`helios_play`, or another run file to `helios_test_sim`, to profile a
different one.

**Useful flags:**

| Flag | Effect |
|------|--------|
| `-o <file>` | Where to save the profile (default `profile.json.gz` in the current directory) |
| `--save-only` | Save without opening the browser |
| `-r <hz>` | Sampling rate (default 1000 Hz) |

To reopen a saved profile later:

```sh
samply load profile.json.gz
```

Symbols are resolved from the binary when the profile is opened, so keep the
binary unchanged (don't rebuild) until you are done reading the profile.

---

## Step 3 — Read the profile

In the Firefox Profiler:

- **Pick the thread** in the track list at the top. Bevy runs systems on the
  main thread and on its compute task pool threads; the work you care about
  may be spread across several.
- **Call Tree**, with **Invert call stack** ticked, lists functions by *self*
  time: time spent inside the function itself, not its callees. This is where
  the CPU actually is, and the list to optimize from.
- **Call Tree** un-inverted gives *total* time: the function plus everything it
  calls. High total with low self time means a hot path whose work happens
  further down.
- **Flame Graph** shows the same tree visually.
- The **search box** filters to matching functions, e.g. `helios_core` or
  `helios_sim`, to see only our code.

---

## Known findings

The latest profile (2026-09-28, raycast car with a 2D lidar, 400 Hz) is
written up in `docs/notes/sim_cpu_profile.md`. In short: the sim's CPU is
dominated by Bevy's scheduler waking and parking its worker threads, not by
physics, the lidar or the autonomy pipeline; helios code grows by about 0.55%
of a core per car, mostly the EKF.
