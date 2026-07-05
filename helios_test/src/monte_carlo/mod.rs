// https://rosettacode.org/wiki/Pseudo-random_numbers/Splitmix64
//! Deterministic seed derivation for Monte Carlo batches.
//!
//! A batch is launched with a single master seed. Every run needs its own seed
//! so the runs actually diverge, yet the whole batch must stay reproducible:
//! re-running with the same master seed reproduces every run's seed exactly.
//! [`seed_for_run`] is that pure mapping from `(master seed, run index)` to a
//! per-run seed.

/// The golden-ratio odd constant. Multiplying the run index by it spreads
/// consecutive indices far apart across the 64-bit space before mixing, so
/// runs 0, 1, 2 don't start out as near-neighbors.
const GOLDEN_GAMMA: u64 = 0x9E3779B97F4A7C15;

/// The two splitmix64 finalizer multipliers. These are the published,
/// statistically-tuned constants for the mix; they are not arbitrary and must
/// be used exactly.
const MIX_MULTIPLIER_1: u64 = 0xBF58476D1CE4E5B9;
const MIX_MULTIPLIER_2: u64 = 0x94D049BB133111EB;

/// Derive the seed for one run of a Monte Carlo batch from the batch's master
/// `base_seed` and the run's `run_index`.
///
/// Pure and total: the same inputs always yield the same output (no clock, no
/// OS entropy), so a batch is reproducible; different `run_index` values yield
/// well-separated outputs, so the runs diverge. This is a splitmix64 finalizer
/// adapted from a sequential generator into a hash of the two inputs — the
/// index is folded into the base rather than accumulated into internal state.
pub fn seed_for_run(base_seed: u64, run_index: u32) -> u64 {
    // Combine the two inputs into one word. Wrapping arithmetic is deliberate:
    // overflow is not an error here, it is part of the scrambling.
    let mut seed = base_seed.wrapping_add((run_index as u64).wrapping_mul(GOLDEN_GAMMA));

    // Finalizer: alternate xor-shifts (move high bits down) with multiplies
    // (smear bits back across the whole word). The back-and-forth is what makes
    // a one-bit change in the input flip roughly half the output bits.
    seed ^= seed >> 30;
    seed = seed.wrapping_mul(MIX_MULTIPLIER_1);
    seed ^= seed >> 27;
    seed = seed.wrapping_mul(MIX_MULTIPLIER_2);
    seed ^= seed >> 31;

    seed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_inputs_reproduce_the_same_seed() {
        // The reproducibility guarantee: identical inputs, identical output,
        // every time. This is what lets a batch be re-run to the same result.
        assert_eq!(seed_for_run(42, 3), seed_for_run(42, 3));
    }

    #[test]
    fn different_run_indices_give_distinct_seeds() {
        // Successive runs in one batch must not collide, or they'd draw the same
        // random stream and defeat the point of the batch.
        let s0 = seed_for_run(42, 0);
        let s1 = seed_for_run(42, 1);
        let s2 = seed_for_run(42, 2);
        assert_ne!(s0, s1);
        assert_ne!(s1, s2);
        assert_ne!(s0, s2);
    }

    #[test]
    fn different_base_seeds_give_distinct_seeds() {
        // Two batches launched with different master seeds must not produce the
        // same per-run seed for the same run index.
        assert_ne!(seed_for_run(1, 0), seed_for_run(2, 0));
    }

    #[test]
    fn output_is_pinned_to_the_algorithm() {
        // Locks the exact mapping so any change to a constant or a mix step is
        // caught: reproducibility across code versions, not just across calls.
        // (`(0, 0)` maps to `0`, so a non-trivial input is pinned instead.)
        assert_eq!(seed_for_run(42, 3), 5_139_283_748_462_763_858);
    }
}
