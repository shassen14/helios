//! Builds the tf panel model from the selected agents' live tf buffers.
//!
//! The interpretation — join topology to sample stats, run the layout, bucket
//! staleness into a verdict — is lifted into [`build_agent_graph`], a pure
//! function over `helios_core` types with no `App`, so it is testable in
//! isolation. [`gather_tf_panel`] is the thin ECS shell around it: read the
//! selection and the clock, call the builder per agent, write the resource.

use crate::prelude::{AgentIdComponent, TfServiceComponent};
use crate::viz::interaction::selection::Selected;
use crate::viz::interaction::tf_panel::geometry::TfPanelHealthColors;
use crate::viz::interaction::tf_panel::layout::tree_layout;
use crate::viz::interaction::tf_panel::model::{AgentGraph, EdgeHealth, HealthVerdict, PanelEdge, PanelNode};
use crate::viz::interaction::tf_panel::TfPanelModel;
use crate::viz::interaction::tuning::{require_positive, InteractionTuningError};

use helios_core::data::{AgentId, MonotonicTime};
use helios_core::frames::transforms::tf::stamped::{DynamicEdgeStats, EdgeKindTag, FrameEdge};

use bevy::prelude::*;
use serde::Deserialize;
use std::collections::HashMap;

/// Sparse TOML overrides for the `[tf_panel.health]` section. Every field is optional;
/// anything omitted falls back to a compiled-in default. The section fans out into two
/// single-consumer resources on resolve: the staleness [`TfPanelHealthThresholds`] read
/// here in gather, and the [`TfPanelHealthColors`] read by the renderer.
#[derive(Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct TfPanelHealthTuningFile {
    pub stale_after: Option<f64>,
    pub dead_after: Option<f64>,
    pub ok_color: Option<[f32; 3]>,
    pub stale_color: Option<[f32; 3]>,
    pub dead_color: Option<[f32; 3]>,
    pub none_color: Option<[f32; 3]>,
}

/// Seconds since an edge's newest sample past which it reads as stale, then dead. A
/// `Resource` — one operator preference for the session — read by the verdict bucketing
/// in [`edge_health`]. Defaults reproduce the values compiled in before the tuning
/// surface existed.
#[derive(Resource, Debug, Clone)]
pub struct TfPanelHealthThresholds {
    pub stale_after: f64,
    pub dead_after: f64,
}

impl Default for TfPanelHealthThresholds {
    fn default() -> Self {
        Self {
            stale_after: 0.5,
            dead_after: 2.0,
        }
    }
}

impl TfPanelHealthThresholds {
    /// Resolves the whole `[tf_panel.health]` section, fanning it into its two
    /// single-consumer resources: the thresholds (read here) and the verdict colours
    /// (read by the renderer). Rejects a non-positive threshold and a `dead_after` that
    /// is not strictly greater than `stale_after` — a dead-before-stale ordering would
    /// bucket every edge wrong.
    pub(crate) fn resolve(
        overrides: &TfPanelHealthTuningFile,
    ) -> Result<(Self, TfPanelHealthColors), InteractionTuningError> {
        let mut thresholds = Self::default();
        if let Some(v) = overrides.stale_after {
            thresholds.stale_after = v;
        }
        if let Some(v) = overrides.dead_after {
            thresholds.dead_after = v;
        }

        require_positive("tf_panel.health.stale_after", thresholds.stale_after as f32)?;
        require_positive("tf_panel.health.dead_after", thresholds.dead_after as f32)?;
        if thresholds.dead_after <= thresholds.stale_after {
            return Err(InteractionTuningError::HealthThresholdOrder {
                stale: thresholds.stale_after,
                dead: thresholds.dead_after,
            });
        }

        let mut colors = TfPanelHealthColors::default();
        if let Some([r, g, b]) = overrides.ok_color {
            colors.ok = Color::srgb(r, g, b);
        }
        if let Some([r, g, b]) = overrides.stale_color {
            colors.stale = Color::srgb(r, g, b);
        }
        if let Some([r, g, b]) = overrides.dead_color {
            colors.dead = Color::srgb(r, g, b);
        }
        if let Some([r, g, b]) = overrides.none_color {
            colors.none = Color::srgb(r, g, b);
        }

        Ok((thresholds, colors))
    }
}

/// Rebuilds the panel model from every selected agent's estimated tree.
///
/// One [`AgentGraph`] per selected agent — the renderer draws them unjoined.
/// `iter()` already spans a multi-selection; under replace-on-click there is only
/// ever one, so the single-agent case falls out for free. The plugin gates this
/// off unless the panel is visible, so an unopened panel costs nothing. Assigning
/// the whole `Vec` each frame is the deliberate full rebuild: the tree is tiny, so
/// there is no dirty-tracking to justify.
pub fn gather_tf_panel(
    query: Query<(&TfServiceComponent, &AgentIdComponent), With<Selected>>,
    time: Res<Time>,
    thresholds: Res<TfPanelHealthThresholds>,
    mut model: ResMut<TfPanelModel>,
) {
    let now = MonotonicTime(time.elapsed_secs_f64());

    model.0 = query
        .iter()
        .map(|(tf_service, agent_id)| {
            let buffer = tf_service.0.buffer();
            build_agent_graph(
                agent_id.0.clone(),
                buffer.edges(),
                buffer.dynamic_edge_stats(),
                now,
                &thresholds,
            )
        })
        .collect();
}

/// Assembles one agent's placed, health-annotated graph from raw buffer output.
///
/// Layout takes topology alone, so the kind tags are stripped before the call;
/// they re-enter only through the join, which keys the dynamic stats onto their
/// edge by `FrameEdge` (`Eq + Hash`). An edge with a stats entry gets `Some`
/// health; a static edge, or a dynamic one with no samples yet, gets `None`.
pub fn build_agent_graph(
    agent: AgentId,
    edges: Vec<(FrameEdge, EdgeKindTag)>,
    stats: Vec<(FrameEdge, DynamicEdgeStats)>,
    now: MonotonicTime,
    thresholds: &TfPanelHealthThresholds,
) -> AgentGraph {
    let stats_by_edge: HashMap<FrameEdge, DynamicEdgeStats> = stats.into_iter().collect();

    let topology: Vec<FrameEdge> = edges.iter().map(|(edge, _tag)| edge.clone()).collect();

    let nodes = tree_layout(&topology)
        .into_iter()
        .map(|(frame, cell)| PanelNode {
            // Leaf only: every node in one graph shares the agent scope, which the
            // panel already prints once as the graph's header, so repeating
            // `agent/` on each node is pure noise (and eats the node's width).
            label: frame.leaf().as_str().to_owned(),
            frame,
            cell,
        })
        .collect();

    let edges = edges
        .into_iter()
        .map(|(edge, _tag)| PanelEdge {
            health: stats_by_edge
                .get(&edge)
                .map(|s| edge_health(s, now, thresholds)),
            child: edge.child,
            parent: edge.parent,
        })
        .collect();

    AgentGraph {
        agent,
        nodes,
        edges,
    }
}

/// Turns raw sample stamps into an interpreted [`EdgeHealth`]. `n` samples spanning
/// `T` seconds give `(n-1)/T` Hz; a lone sample has no defined interval and reports
/// `0`. Staleness is measured from the newest sample to `now`, then bucketed.
fn edge_health(
    stats: &DynamicEdgeStats,
    now: MonotonicTime,
    thresholds: &TfPanelHealthThresholds,
) -> EdgeHealth {
    let span = stats.newest.0 - stats.oldest.0;
    let rate_hz = if span > 0.0 {
        stats.sample_count.saturating_sub(1) as f64 / span
    } else {
        0.0
    };

    let staleness_s = now.0 - stats.newest.0;
    let verdict = if staleness_s >= thresholds.dead_after {
        HealthVerdict::Dead
    } else if staleness_s >= thresholds.stale_after {
        HealthVerdict::Stale
    } else {
        HealthVerdict::Ok
    };

    EdgeHealth {
        rate_hz,
        staleness_s,
        verdict,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::frames::FrameId;

    /// A dynamic-style edge `child -> parent`; kind supplied per test.
    fn edge(child: FrameId, parent: FrameId) -> FrameEdge {
        FrameEdge { child, parent }
    }

    fn stats(newest: f64, oldest: f64, sample_count: usize) -> DynamicEdgeStats {
        DynamicEdgeStats {
            newest: MonotonicTime(newest),
            oldest: MonotonicTime(oldest),
            sample_count,
        }
    }

    fn health_of<'a>(graph: &'a AgentGraph, child: &FrameId) -> &'a Option<EdgeHealth> {
        graph
            .edges
            .iter()
            .find_map(|e| (&e.child == child).then_some(&e.health))
            .expect("edge present")
    }

    /// The join is the load-bearing seam: a dynamic edge carries health, a static
    /// edge carries none, and every frame in the topology is placed as a node.
    #[test]
    fn build_joins_stats_to_dynamic_edges_only() {
        let agent = AgentId::new("robot_1");
        let base_link = FrameId::base_link(agent.clone());
        let odom = FrameId::odom(agent.clone());
        let imu = FrameId::sensor(agent.clone(), "imu");

        let graph = build_agent_graph(
            agent.clone(),
            vec![
                (edge(base_link.clone(), odom.clone()), EdgeKindTag::Dynamic),
                (edge(imu.clone(), base_link.clone()), EdgeKindTag::Static),
            ],
            vec![(edge(base_link.clone(), odom.clone()), stats(9.9, 9.0, 10))],
            MonotonicTime(10.0),
            &TfPanelHealthThresholds::default(),
        );

        assert_eq!(graph.nodes.len(), 3, "every frame in the topology is placed");
        assert!(
            health_of(&graph, &base_link).is_some(),
            "the dynamic edge carries health",
        );
        assert!(
            health_of(&graph, &imu).is_none(),
            "the static edge carries no health",
        );
    }

    /// A dynamic edge whose kind tag says dynamic but which has no stats entry (no
    /// samples yet) reads as `None`, the same as a static edge — the deferred
    /// cold-start case.
    #[test]
    fn build_leaves_sampleless_dynamic_edge_without_health() {
        let agent = AgentId::new("robot_1");
        let base_link = FrameId::base_link(agent.clone());
        let odom = FrameId::odom(agent.clone());

        let graph = build_agent_graph(
            agent,
            vec![(edge(base_link.clone(), odom.clone()), EdgeKindTag::Dynamic)],
            vec![],
            MonotonicTime(10.0),
            &TfPanelHealthThresholds::default(),
        );

        assert!(health_of(&graph, &base_link).is_none());
    }

    /// Rate is samples-minus-one over the window, so ten samples across 0.9 s read
    /// as 10 Hz — the interval count, not the sample count, sets the rate.
    #[test]
    fn edge_health_reports_rate_over_the_window() {
        let h = edge_health(
            &stats(9.9, 9.0, 10),
            MonotonicTime(9.9),
            &TfPanelHealthThresholds::default(),
        );
        assert!((h.rate_hz - 10.0).abs() < 1e-9);
    }

    /// A single sample spans no interval, so its rate is defined as zero rather
    /// than dividing by a zero window.
    #[test]
    fn edge_health_reports_zero_rate_for_a_single_sample() {
        let h = edge_health(
            &stats(9.0, 9.0, 1),
            MonotonicTime(9.0),
            &TfPanelHealthThresholds::default(),
        );
        assert_eq!(h.rate_hz, 0.0);
    }

    /// The three staleness buckets, pinned at their boundaries: fresh is `Ok`, at
    /// or past the stale threshold is `Stale`, at or past the dead threshold is
    /// `Dead`.
    #[test]
    fn edge_health_buckets_staleness() {
        let th = TfPanelHealthThresholds::default();
        let fresh = edge_health(&stats(9.9, 9.0, 2), MonotonicTime(10.0), &th);
        assert_eq!(fresh.verdict, HealthVerdict::Ok);

        let stale = edge_health(&stats(9.0, 8.0, 2), MonotonicTime(10.0), &th);
        assert_eq!(stale.verdict, HealthVerdict::Stale);

        let dead = edge_health(&stats(7.0, 6.0, 2), MonotonicTime(10.0), &th);
        assert_eq!(dead.verdict, HealthVerdict::Dead);
    }

    /// An empty file resolves both fanned-out resources to their defaults.
    #[test]
    fn health_empty_file_resolves_to_defaults() {
        let (thresholds, colors) =
            TfPanelHealthThresholds::resolve(&TfPanelHealthTuningFile::default()).unwrap();
        let d = TfPanelHealthThresholds::default();
        assert_eq!(thresholds.stale_after, d.stale_after);
        assert_eq!(thresholds.dead_after, d.dead_after);
        assert_eq!(colors.ok, TfPanelHealthColors::default().ok);
    }

    /// Overrides pass through and colours pack into sRGB.
    #[test]
    fn health_overrides_pass_through() {
        let file = TfPanelHealthTuningFile {
            stale_after: Some(1.0),
            dead_after: Some(4.0),
            dead_color: Some([0.1, 0.2, 0.3]),
            ..Default::default()
        };
        let (thresholds, colors) = TfPanelHealthThresholds::resolve(&file).unwrap();
        assert_eq!(thresholds.stale_after, 1.0);
        assert_eq!(thresholds.dead_after, 4.0);
        assert_eq!(colors.dead, Color::srgb(0.1, 0.2, 0.3));
    }

    /// A `dead_after` not strictly greater than `stale_after` buckets every edge wrong
    /// and is rejected.
    #[test]
    fn health_rejects_dead_not_past_stale() {
        let file = TfPanelHealthTuningFile {
            stale_after: Some(2.0),
            dead_after: Some(1.0),
            ..Default::default()
        };
        assert!(matches!(
            TfPanelHealthThresholds::resolve(&file).unwrap_err(),
            InteractionTuningError::HealthThresholdOrder { .. }
        ));
    }

    /// A non-positive threshold is rejected.
    #[test]
    fn health_rejects_nonpositive_threshold() {
        let file = TfPanelHealthTuningFile {
            stale_after: Some(0.0),
            ..Default::default()
        };
        assert!(matches!(
            TfPanelHealthThresholds::resolve(&file).unwrap_err(),
            InteractionTuningError::NonPositive { .. }
        ));
    }
}
