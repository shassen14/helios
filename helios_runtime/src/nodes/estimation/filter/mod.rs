//! The filter component: the recursive algorithm a node runs (EKF, UKF), built
//! around a seeded state and a dynamics model.
//!
//! - `config` — each filter kind's own keys in a `filter` sub-table.
//! - `parts` — [`FilterParts`], what a filter factory receives, and the
//!   built-in filter kinds' factories.

mod config;
mod parts;

pub(super) use self::parts::register;
pub use self::parts::FilterParts;
