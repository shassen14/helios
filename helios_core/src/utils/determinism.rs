//! Deterministic seed derivation.
//!
//! Every subsystem that consumes randomness derives its own RNG stream from one
//! run-level master seed, rather than sharing a single generator. A shared
//! generator makes each draw depend on how many draws every *other* subsystem
//! happened to take first, so a scenario change in one place silently perturbs
//! the noise everywhere else. Deriving instead means a stream depends only on
//! the master seed and the deriving subsystem's own identity, which is what
//! makes a failing run reproducible from its seed alone.
//!
//! [`stable_hash`] is that derivation. [`derive_seed`] is the primitive beneath
//! it, exposed because callers that already hold a numeric counter (a Monte
//! Carlo run index) need the same mixing without going through a string.
//!
//! **The constants in this module are published, statistically-tuned values and
//! must not be adjusted.** Everything here is pure and total — no clock, no OS
//! entropy — so the same inputs yield the same output on every machine and
//! every run.
//!
//! ## Why this lives in `helios_core`
//!
//! Seed derivation is not robotics math, so it sits slightly outside this
//! crate's charter. It is here because it is the only location all three
//! consumers can reach: `helios_sim` seeds its sensors, `helios_test` derives
//! per-run seeds for Monte Carlo batches from a module compiled without the
//! `sim` feature, and a future `helios_hw` replaying a logged run needs the
//! identical mapping. It adds no dependencies.
//!
//! ## Never hash a seed input with `DefaultHasher`
//!
//! `std::collections::hash_map::DefaultHasher` carries no cross-version
//! stability guarantee — its algorithm is explicitly allowed to change between
//! Rust releases. Deriving seeds from it would mean a compiler upgrade silently
//! changes every random stream in the project, with no code change, no test
//! failure, and no way to replay a previously recorded seed. Anything hashed
//! for a seed must use an algorithm this module owns outright.

/// The golden-ratio odd constant. Multiplying the counter by it spreads
/// near-identical counter values far apart across the 64-bit space before
/// mixing, so closely-related inputs don't start out as near-neighbors —
/// consecutive Monte Carlo run indices 0, 1, 2 being the motivating case.
const GOLDEN_GAMMA: u64 = 0x9E3779B97F4A7C15;

/// The two splitmix64 finalizer multipliers. These are the published,
/// statistically-tuned constants for the mix; they are not arbitrary and must
/// be used exactly.
const MIX_MULTIPLIER_1: u64 = 0xBF58476D1CE4E5B9;
const MIX_MULTIPLIER_2: u64 = 0x94D049BB133111EB;

/// The published FNV-1a 64-bit parameters. The prime is odd, so multiplying by
/// it is invertible modulo 2^64 — folding a byte in never destroys information
/// already accumulated.
const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Separates `kind` from `stable_id` in the hash input. `0xFF` never appears in
/// well-formed UTF-8 and `&str` is guaranteed well-formed, so no two distinct
/// (kind, stable_id) pairs can produce the same byte sequence.
const FIELD_SEPARATOR: u8 = 0xFF;

/// Derives an independent seed for one named thing within a run.
///
/// `kind` names the subsystem (`"sensor"`, `"world"`, `"spawn"`); `stable_id`
/// names the individual within it (an agent-qualified sensor path such as
/// `"car_1/gps"`). Both feed the hash, so every `kind` gets a provably disjoint
/// family of streams — a sensor and a world generator that happen to share an
/// id still derive different seeds, and neither can be perturbed by the other's
/// draw count.
///
/// `stable_id` must be stable across runs, which rules out anything allocated
/// at runtime. An ECS entity index is not stable; a config-derived name is.
///
/// The two fields are separated rather than concatenated, so `("sensor", "a/b")`
/// and `("sensora", "/b")` cannot collide: a `0xFF` byte, which well-formed
/// UTF-8 can never contain, separates them in the hashed byte sequence.
pub fn stable_hash(master_seed: u64, kind: &str, stable_id: &str) -> u64 {
    let counter = fnv1a_64(kind, stable_id);
    derive_seed(master_seed, counter)
}

/// Folds `counter` into `base_seed` and mixes the result — the splitmix64
/// finalizer applied to a gamma-spread combination of the two inputs.
///
/// Pure and total: identical inputs always give identical output, and distinct
/// counters give well-separated outputs. This is the shared primitive under
/// both [`stable_hash`] and `helios_test`'s per-run seed derivation, so its
/// exact mapping is a compatibility surface — changing it invalidates every
/// seed ever recorded. The pinned test below is what guards that.
///
/// Reference: <https://rosettacode.org/wiki/Pseudo-random_numbers/Splitmix64>
/// (the finalizer, applied as a hash of two inputs rather than accumulated into
/// a sequential generator's internal state).
pub fn derive_seed(base_seed: u64, counter: u64) -> u64 {
    // Combine the two inputs into one word. Wrapping arithmetic is deliberate:
    // overflow is not an error here, it is part of the scrambling.
    let mut seed = base_seed.wrapping_add(counter.wrapping_mul(GOLDEN_GAMMA));

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

/// Hashes the two fields into one word with FNV-1a.
///
/// Each byte is folded in with two operations that do different jobs: the xor
/// *injects* the byte into the low 8 bits, and the multiply *diffuses* those
/// bits across all 64 positions before the next byte arrives.
///
/// The ordering is what distinguishes FNV-1a from FNV-1, and it is the variant
/// to use here: xor-then-multiply diffuses every byte within its own iteration,
/// including the last one, whereas FNV-1 leaves the final byte undiffused in the
/// low bits. On the short inputs this module sees — `"sensor"`, `"car_1/gps"` —
/// the final byte is a meaningful fraction of the whole input.
fn fnv1a_64(kind: &str, stable_id: &str) -> u64 {
    let mut accumulator = FNV_OFFSET_BASIS;

    for byte in kind
        .bytes()
        .chain([FIELD_SEPARATOR])
        .chain(stable_id.bytes())
    {
        accumulator ^= byte as u64;
        accumulator = accumulator.wrapping_mul(FNV_PRIME);
    }

    accumulator
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A representative subsystem/individual pair, so the tests read as the
    /// real call shape rather than as abstract strings.
    const KIND: &str = "sensor";
    const ID: &str = "car_1/gps";

    /// Backwards compatibility with `helios_test`'s `monte_carlo::seed_for_run`,
    /// which computes this same mapping and pins this same value.
    ///
    /// `seed_for_run` predates this module and has already produced seeds for
    /// recorded Monte Carlo batches. Once it delegates here, this assertion and
    /// its counterpart there must agree, or previously-recorded batches stop
    /// replaying to the same results. If this fails, the primitive drifted.
    #[test]
    fn derive_seed_reproduces_the_monte_carlo_mapping() {
        assert_eq!(derive_seed(42, 3), 5_139_283_748_462_763_858);
    }

    #[test]
    fn derive_seed_is_deterministic() {
        assert_eq!(derive_seed(7, 99), derive_seed(7, 99));
    }

    #[test]
    fn derive_seed_separates_consecutive_counters() {
        // Successive counters must not land near each other, or per-run streams
        // in a batch would start out correlated.
        let a = derive_seed(42, 0);
        let b = derive_seed(42, 1);
        let c = derive_seed(42, 2);
        assert_ne!(a, b);
        assert_ne!(b, c);
        assert_ne!(a, c);
    }

    #[test]
    fn stable_hash_is_deterministic() {
        assert_eq!(stable_hash(42, KIND, ID), stable_hash(42, KIND, ID));
    }

    // The next three vary exactly one argument each. Kept separate on purpose:
    // a single combined test would still pass if the function ignored one of
    // its three inputs entirely.

    #[test]
    fn stable_hash_varies_with_master_seed() {
        assert_ne!(stable_hash(42, KIND, ID), stable_hash(43, KIND, ID));
    }

    #[test]
    fn stable_hash_varies_with_kind() {
        assert_ne!(stable_hash(42, KIND, ID), stable_hash(42, "world", ID));
    }

    #[test]
    fn stable_hash_varies_with_stable_id() {
        assert_ne!(stable_hash(42, KIND, ID), stable_hash(42, KIND, "car_1/imu"));
    }

    /// The field separator's reason for existing. Under naive concatenation
    /// both of these hash the byte string `"sensora/b"` and collide, silently
    /// handing two different subsystems the same RNG stream. This is the test
    /// that fails if `FIELD_SEPARATOR` is ever dropped from the chain.
    #[test]
    fn stable_hash_separates_kind_from_stable_id() {
        assert_ne!(
            stable_hash(42, "sensor", "a/b"),
            stable_hash(42, "sensora", "/b")
        );
    }

    /// Empty fields are degenerate but must stay total — no panic — and must
    /// still distinguish which field was empty.
    #[test]
    fn stable_hash_handles_empty_fields() {
        assert_eq!(stable_hash(42, "", ""), stable_hash(42, "", ""));
        assert_ne!(stable_hash(42, "", "x"), stable_hash(42, "x", ""));
    }

    /// Locks the exact mapping. Every seed the project derives flows from this
    /// function, so a change to a constant, to the FNV variant, or to the field
    /// encoding would silently alter every random stream in every scenario.
    ///
    /// The expected value is definitionally whatever the algorithm produces —
    /// it was generated by running it. What is being pinned is that it never
    /// changes *again*. Do not update this number to make a failing test pass:
    /// a failure here means the derivation moved, and every previously recorded
    /// seed now replays to different results.
    #[test]
    fn stable_hash_is_pinned_to_the_algorithm() {
        assert_eq!(stable_hash(42, KIND, ID), 11_518_198_242_234_854_978);
    }
}
