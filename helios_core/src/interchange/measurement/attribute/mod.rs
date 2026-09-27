//! Attribute columns: per-point (cloud) or per-cell (range field) values a
//! sensor reports beyond geometry and time, carried as named, typed data rather
//! than as part of the payload's type.
//!
//! Keeping attributes out of the type means one cloud type serves every
//! sensor: a consumer that needs fewer attributes than a producer offers reads
//! the same channel and never asks for the rest.
//!
//! - [`key`] — [`AttributeKey<T>`](key::AttributeKey), the typed name of one
//!   column, and its erased [`AttributeDescriptor`](key::AttributeDescriptor).
//! - [`canonical`] — the core measurement keys ([`INTENSITY`](canonical::INTENSITY),
//!   [`RING`](canonical::RING)).
//! - [`schema`] — [`AttributeSchema`](schema::AttributeSchema), the columns a
//!   payload carries, and the build-time checks over schemas.

pub mod canonical;
pub mod key;
pub mod schema;
