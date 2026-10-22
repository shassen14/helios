//! Reductions of many values to one, at two levels: [`within_run`] reduces
//! one series' samples to the value an assertion judges or a measure
//! reports; [`across_runs`] reduces one entry's value from each run of a
//! batch. Both are closed sets parsed strictly, and the math they share is
//! written once, in [`numbers`].

pub mod across_runs;
pub mod numbers;
pub mod within_run;
