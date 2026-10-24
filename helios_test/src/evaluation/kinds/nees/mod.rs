//! `nees`: whether an estimator's covariance is honest about its error. At
//! each state, the truth interpolated to its stamp and placed in the state's
//! root, and the normalized estimation error squared of each block the truth
//! gives, and of them jointly. A pose truth grades the position and the
//! orientation; the binding names every other block and why it is ungraded.

pub mod config;
mod grading;
mod kind;

pub use kind::Nees;
