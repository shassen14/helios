//! The reference seam: turns the `[reference]` section into the `Selector`
//! that writes the guidance reference the controllers track.
//!
//! Each member writes its reference on a channel of its own. The seam reads
//! the section's member list, finds each member's reference output among the
//! built nodes, and adds one `Selector` from those channels onto the resolved
//! [`control::reference`] channel. A lone member is the selector's `base` with
//! no `preferred` inputs, which forwards it every tick, so the shape is the
//! same for any number of members.
//!
//! The seam type is `BodyTwistRef`, the only reference type today. When a
//! second appears, the section names it and this pass dispatches on it.
//!
//! - `config` — the section's config.
//! - `pass` — the pass that builds the seam's node.
//!
//! [`control::reference`]: crate::channels::control::reference

mod config;
mod pass;

pub use self::config::{ArbitrationPolicyConfig, ReferenceSeamConfig};
pub(in crate::assembly) use self::pass::reference_selector;
