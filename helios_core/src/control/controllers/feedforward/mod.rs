//! Feedforward laws — precompute the command open-loop by inverting the plant. An
//! inverse map is inherently plant-specific, so each file names its plant rather
//! than the axis it acts on (e.g. [`road_load`] inverts a ground vehicle's
//! tractive resistance).

pub mod road_load;
