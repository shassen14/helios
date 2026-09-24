//! Interpretations of one or more measurements — nouns that required a world
//! model, an ontology, or association across time to produce. Named by the
//! concept ([`MapData`](map::MapData), `Detection`), never by the device or
//! algorithm that emitted them.
//!
//! The firewall against [`measurement`](super::measurement) is enforced at
//! *production*: a perception noun can be born only in a perception domain, so a
//! forward sensor model structurally cannot emit one.

pub mod map;
