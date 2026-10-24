//! Finalizing a run: each evaluation's series derived, each entry cut to
//! its window and judged or measured, and the report built.

pub mod entries;
mod reached;
pub mod report;
pub mod verdict;
pub mod windowed;

#[cfg(test)]
mod fixtures;

pub use report::HostFacts;
