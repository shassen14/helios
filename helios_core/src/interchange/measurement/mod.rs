//! Physical quantities a device transduces — producer-agnostic, named by the
//! quantity rather than the device. A `Velocity` has one home here whether it
//! came from GPS, a DVL, an encoder, or optical flow; who produced a given
//! reading is a runtime fact carried in the envelope's provenance, orthogonal
//! to where the type is defined.
//!
//! - [`envelope`] — [`SensorReading<T>`](envelope::SensorReading), the
//!   frame-and-time stamp every measurement rides in.
//! - [`sensor`] — the [`SensorPayload`](sensor::SensorPayload) trait and the
//!   scalar/vector payload primitives that map onto a filter's `z`.
//! - [`cloud`] — [`PointCloud`](cloud::PointCloud), the SoA world-sensor cloud.
//! - [`range_field`] — the organized world-sensor grid: one range per beam,
//!   misses kept, addressed by a
//!   [`DirectionModel`](range_field::DirectionModel).

pub mod cloud;
pub mod envelope;
pub mod range_field;
pub mod sensor;
