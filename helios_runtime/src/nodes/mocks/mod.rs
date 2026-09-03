//! Runtime-native test doubles.
//!
//! These wrap no `helios_core` trait; they are fixtures, selected from TOML by
//! `kind` like a real estimator. They are not combinators (no fan-in), and they
//! live in `helios_runtime` (not `helios_test`) because both integration tests
//! and dev iteration use them.
//!
//! - `mock_oracle_estimator` — `MockOracleEstimatorNode`, publishes ground-truth
//!   pose/twist read from the oracle channels.
//! - `register` — registers the built-in mock factories.
//!
//! Only the `register` fn crosses the family boundary.

mod mock_oracle_estimator;
mod register;

pub(crate) use register::register;
