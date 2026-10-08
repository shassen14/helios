//! The command seam: turns each `[command.<fold>]` table into a `Sum` that
//! writes the fold's command for an allocator to read.
//!
//! Each member writes its command on a channel of its own. For every fold, the
//! pass looks up the fold's `type` among the registered command types, finds
//! each member's output of that type among the built nodes, and adds a `Sum`
//! named after the fold, writing `T @ <fold name>`. Required members are the
//! sum's required inputs, so it publishes only once they all have; optional
//! members are folded in when present. A lone member is a one-input sum, which
//! forwards it, so the shape is the same for any number of members.
//!
//! A command type is an entry in [`CommandTypes`], a registry extension, not a
//! variant here, so any number of folds may share one and a type defined
//! outside this crate is one registration away.
//!
//! - `config` — [`CommandFoldConfig`], one `[command.<fold>]` table.
//! - `types` — [`CommandTypes`] and the built-in command types.
//! - `pass` — the pass that builds each fold's `Sum`.

mod config;
mod pass;
mod types;

pub use self::config::CommandFoldConfig;
pub(in crate::assembly) use self::pass::command_sums;
pub(crate) use self::types::register;
pub use self::types::{CommandTypes, DuplicateCommandType};
