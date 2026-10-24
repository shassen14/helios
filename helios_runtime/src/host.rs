//! [`HostDeclaration`]: what the host running the pipeline declares about
//! itself, apart from the body it is attached to.

use helios_core::spatial::{transforms::Convention, FrameId};

use std::collections::HashMap;

/// The host's own facts, declared once when the pipeline is built: how often
/// it ticks, and the frames of the world it places the body in.
#[derive(Clone, Debug, PartialEq)]
pub struct HostDeclaration {
    /// How many times a second the host calls
    /// [`tick`](crate::AutonomyPipeline::tick), nominally. A node that is not
    /// rate-gated writes its outputs at this rate.
    pub tick_rate_hz: f64,
    /// Each world frame the host's values name, such as the datum ground
    /// truth is published in, with its axis convention. The host owns each;
    /// a node defining one, or a body naming one, fails the build.
    pub world_frames: HashMap<FrameId, Convention>,
}
