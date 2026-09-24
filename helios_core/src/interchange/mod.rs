//! The typed, frame-tagged data nouns that domains hand each other — the
//! *messages*, not the math. A type lives here if and only if two or more
//! domains name it: one produces it, another consumes it. A noun named by a
//! single domain stays inside that domain until a second domain needs it.
//!
//! - [`measurement`] — physical quantities a device transduces, named by the
//!   quantity rather than the producing device.
//! - [`perception`] — interpretations that required a world model, an ontology,
//!   or association across time (maps, detections, tracks).
//! - [`motion`] — estimator/control I/O ([`Odometry`](motion::Odometry),
//!   [`Twist`](motion::Twist)).
//! - [`path`] — planner output ([`Path`](path::Path),
//!   [`PlannerGoal`](path::PlannerGoal)) consumed by path following.
//!
//! The [`measurement`] ⊥ [`perception`] split partitions *world-derived* nouns
//! only; `motion` and `path` are not world-derived and stay flat at the root.

pub mod measurement;
pub mod motion;
pub mod path;
pub mod perception;
