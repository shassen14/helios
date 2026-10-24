//! `contacts`: how often a body hit something, and how long it stayed
//! touching. Read `collisions` with `final` for the episode count, and
//! `in_contact` with `time_above = 0.5` for the seconds in contact.

pub mod config;
mod kind;

pub use kind::Contacts;
