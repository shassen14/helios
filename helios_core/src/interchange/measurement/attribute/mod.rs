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
//! - [`column`](mod@column) — [`AttributeColumn`](column::AttributeColumn),
//!   one column's values with its element type as a variant.
//! - [`schema`] — [`AttributeSchema`](schema::AttributeSchema), the columns a
//!   payload carries, and the build-time checks over schemas.
//! - [`table`] — [`AttributeTable`](table::AttributeTable), a schema with its
//!   columns, read by key.

pub mod canonical;
pub mod column;
pub mod key;
pub mod schema;
pub mod table;
