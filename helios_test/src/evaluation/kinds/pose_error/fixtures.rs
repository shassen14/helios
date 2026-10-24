//! Test fixtures for binding: the placement a built pipeline gives an
//! estimator's root, pinned through a GPS at a world frame or not at all.

use helios_core::prelude::{AgentId, FrameId, TfProvider};
use helios_core::spatial::transforms::Convention;
use helios_runtime::{
    pipeline::FramePlacement,
    port::{ChannelKey, Pinned, PortBus, PortDescriptor, PortDescriptorBuilder, Provenance},
    prelude::{PipelineNode, TickContext},
    BodyCapabilities, BodyChannel, HostDeclaration, PipelineBuilder,
};

use std::collections::HashMap;

pub(crate) const CAR: &str = "car";

/// The world frame the truth is expressed in.
pub(crate) fn datum() -> FrameId {
    FrameId::global("datum")
}

/// A second world frame the host declares, which the truth is not in.
pub(crate) fn site() -> FrameId {
    FrameId::global("site")
}

/// Writes nothing; carries the estimator's descriptor.
struct Estimator(PortDescriptor);

impl PipelineNode for Estimator {
    fn name(&self) -> &str {
        "estimator"
    }

    fn port_descriptor(&self) -> &PortDescriptor {
        &self.0
    }

    fn execute(&self, _: &PortBus, _: &dyn TfProvider, _: TickContext) {}
}

/// The placement the build gives `root`, which an estimator defines in
/// `axes`, pinning its position through a GPS referenced to `pinned_at`, or
/// reading no input when `None`. The host declares [`datum`] and [`site`]
/// in ENU.
pub(crate) fn placement(
    root: FrameId,
    axes: Convention,
    pinned_at: Option<FrameId>,
) -> FramePlacement {
    let gps = ChannelKey::measurement::<f64>("gps");
    let (ports, publishes) = match pinned_at {
        Some(reference) => (
            PortDescriptorBuilder::algorithm()
                .optional(gps.clone())
                .pins(root.clone(), Pinned::Position, &gps),
            vec![BodyChannel::new(gps, "gps", Provenance::Simulated).with_reference(reference)],
        ),
        None => (PortDescriptorBuilder::algorithm(), vec![]),
    };
    let pipeline = PipelineBuilder::new(AgentId::new(CAR))
        .with_body_capabilities(BodyCapabilities {
            publishes,
            ..Default::default()
        })
        .with_host(HostDeclaration {
            tick_rate_hz: 100.0,
            world_frames: HashMap::from([(datum(), Convention::Enu), (site(), Convention::Enu)]),
        })
        .add_node(Box::new(Estimator(ports.defines_frame(root, axes).build())))
        .build()
        .unwrap_or_else(|errors| panic!("the estimator builds: {errors:?}"));
    pipeline.frame_placements()[0].clone()
}
