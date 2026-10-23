//! Suites: evaluations written once over agent series, bound to each agent
//! they are applied to.

pub mod bind;
pub mod config;

pub use bind::bind;
pub use config::{AgentSelection, Suite, SuiteApplication, SuiteConfig};
