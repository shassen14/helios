//! The tf panel's portable data model: a placed, health-annotated projection of
//! one agent's estimated transform tree.
//!
//! This is data only — no `bevy_ui` nodes, no widgets, no `Entity` — so it is the
//! seam between the gather half (which reads ECS and the tf buffer) and the render
//! half (which only walks these structs). Positions are abstract grid cells, not
//! pixels; health is an interpreted verdict, not raw sample stamps. Turning either
//! into screen coordinates or colours is the renderer's job and never enters here.
//!
//! `serde` is deliberately absent for now: [`AgentId`] and [`LayoutCell`] are not
//! yet `Serialize`, and a config/resolved-config dump is the feature that will
//! justify making the whole chain serializable at once.

use crate::viz::interaction::tf_panel::layout::LayoutCell;

use helios_core::{data::AgentId, frames::FrameId};

/// One agent's estimated tree, laid out and annotated — the unit the renderer
/// draws. The panel model is a list of these, one per selected agent, drawn
/// unjoined: helios has no shared estimated root across agents, so there is no
/// forest to merge, only N independent graphs side by side.
#[derive(Debug, Clone, PartialEq)]
pub struct AgentGraph {
    pub agent: AgentId,
    pub nodes: Vec<PanelNode>,
    pub edges: Vec<PanelEdge>,
}

/// A single frame placed on the panel grid. `label` is the display string — the
/// frame's leaf name alone, since the agent scope is shown once as the graph
/// header, not repeated per node. The full `frame` is kept alongside it so the
/// renderer can hit-test or cross-reference by identity without re-parsing the
/// string; `cell` is the abstract `(depth, slot)` the renderer maps to pixels.
#[derive(Debug, Clone, PartialEq)]
pub struct PanelNode {
    pub frame: FrameId,
    pub cell: LayoutCell,
    pub label: String,
}

/// A parent→child link to draw a connector for. `health` is `None` for a static
/// edge — a single-sourced extrinsic carries no rate or staleness signal — and
/// `Some` for a dynamic edge that has samples. A dynamic edge with no samples yet
/// (cold start) also reads as `None` here; distinguishing the two needs an
/// expected-topology oracle that does not exist yet.
#[derive(Debug, Clone, PartialEq)]
pub struct PanelEdge {
    pub child: FrameId,
    pub parent: FrameId,
    pub health: Option<EdgeHealth>,
}

/// The interpreted health of one dynamic edge at the snapshot instant: a publish
/// rate, how long since its newest sample, and the verdict those imply. Derived in
/// the gather step so the renderer stays free of thresholds and clock arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeHealth {
    pub rate_hz: f64,
    pub staleness_s: f64,
    pub verdict: HealthVerdict,
}

/// A dynamic edge's staleness bucket, driving the renderer's one-glance colour.
/// `Ok` is the calm default; `Stale` and `Dead` are the exceptions worth a signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthVerdict {
    Ok,
    Stale,
    Dead,
}
