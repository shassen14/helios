//! The world layout as the simulation uses it: every placement checked, its
//! class resolved to a `SemanticClass`, its orientation turned into a
//! rotation, and its body into a static or dynamic body with a mass.
//!
//! Resolution is a pure function over `LoadedWorldLayout`, so a layout
//! written by hand and one produced by a generator pass the same checks.

mod error;
mod plugin;
mod resolved;

pub use error::LayoutError;
pub use plugin::WorldLayoutPlugin;
pub use resolved::{ResolvedBody, ResolvedPlacement, ResolvedPrefab, ResolvedWorldLayout};
