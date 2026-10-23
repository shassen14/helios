//! [`EstimatorPlan`]: a declared estimator waiting for the pipeline to be
//! wired, and its construction into a [`RecursiveEstimatorNode`].
//!
//! Constructing builds the measurement models, the augmentation blocks, the
//! seeded state, the input builder and the filter, checking each against the
//! others. Each model, block and the input builder is built for the mounts its
//! channels' writers state.

use super::aiding::Aiding;
use super::config::InitialPoseConfig;
use super::nis_health::NisWindow;
use super::node::RecursiveEstimatorNode;
use super::register::{AIDING_KEY, DYNAMICS_KEY};

use crate::assembly::{BuildContext, BuildFailure, DeferredComponent};
use crate::nodes::estimation::{
    DynamicsComponent, FilterParts, MeasurementSource, MeasurementWiring,
};
use crate::pipeline::{node::PipelineNode, InputMounts};
use crate::port::{ChannelKey, Pinned, PortDescriptor};

use helios_core::estimation::augmentation::AugmentationSpec;
use helios_core::estimation::schema::{
    check_measurement_state_agreement, MeasurementSchema, StateSchema, StateSchemaBlock,
};
use helios_core::estimation::GaussianStateEstimator;
use helios_core::prelude::AgentId;
use helios_core::spatial::conventions::{Enu, Flu};
use helios_core::spatial::primitives::MonotonicTime;
use helios_core::spatial::state::Quantity;
use helios_core::spatial::transforms::{Convention, Transform};
use helios_core::spatial::{FrameAwareState, FrameId};

use nalgebra::{DMatrix, Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::sync::Arc;

/// Everything a declared estimator needs to be built: the parts checked,
/// parsed or built at declaration, the root it holds its state in, and the
/// starting pose, read only at construction.
pub(super) struct EstimatorPlan {
    pub(super) agent: AgentId,
    pub(super) root: FrameId,
    pub(super) node_name: String,
    pub(super) kind: String,
    pub(super) dynamics: DynamicsComponent,
    pub(super) filter: DeferredComponent<FilterParts, Box<dyn GaussianStateEstimator>>,
    pub(super) aiding: Vec<AidingPlan>,
    pub(super) augmentation: Vec<AugmentationPlan>,
    pub(super) initial_pose: InitialPoseConfig,
}

impl EstimatorPlan {
    /// Builds each aiding source for its channel's mount in `mounts`, then
    /// the state they agree on, seeded with the starting pose, then the
    /// input builder for its channels' mounts and the filter around the
    /// state, and the node with the ports `descriptor` declared for it.
    pub(super) fn construct(
        self,
        descriptor: PortDescriptor,
        mounts: InputMounts,
    ) -> Result<Box<dyn PipelineNode>, BuildFailure> {
        let EstimatorPlan {
            agent,
            root,
            node_name,
            kind,
            dynamics,
            filter,
            aiding,
            augmentation,
            initial_pose,
        } = self;
        let ctx = BuildContext::new(agent, &node_name, &kind, root);

        let mut sources = Vec::with_capacity(aiding.len());
        // Each entry's name, which names its leaves, with its NIS window.
        let mut entry_windows = Vec::with_capacity(aiding.len());
        for entry in aiding {
            let wiring = MeasurementWiring {
                input: entry.input,
                mount: mount_of(&mounts, &entry.channel)?,
                noise: entry.noise,
            };
            let source = entry.model.build(&ctx, wiring)?;
            check_source_channel(&entry.name, &entry.channel, source.channel())?;
            check_source_pin(
                &entry.name,
                entry.pins,
                &source.model().schema(),
                &ctx.frames().root,
            )?;
            sources.push(source);
            entry_windows.push((entry.name, entry.nis_window));
        }
        let augmentation = augmentation_blocks(&augmentation, &mounts)?;

        let initial_state = seeded_state(&dynamics, augmentation, &sources, &initial_pose, &ctx)?;
        let (process, input) = dynamics.into_parts();
        let input = input
            .build(&mounts)
            .map_err(|err| format!("{DYNAMICS_KEY}: {err}"))?;
        let filter = filter.build(&ctx, FilterParts::new(initial_state, process))?;

        let aiding = sources
            .into_iter()
            .zip(entry_windows)
            .map(|(source, (name, window))| match window {
                Some(window) => Aiding::new(&name, source).with_nis_window(window),
                None => Aiding::new(&name, source),
            })
            .collect();
        let node = RecursiveEstimatorNode::new(ctx.node_name(), filter, input, aiding, descriptor);
        Ok(Box::new(node))
    }
}

/// One aiding entry as declared: the channel its model's kind reads, named
/// `input` in config, the axes the model reads that channel's mount in,
/// what of the root the model pins to that channel's world frame, if anything,
/// its NIS window, its R, and the source still to build.
pub(super) struct AidingPlan {
    pub(super) name: String,
    pub(super) input: String,
    pub(super) channel: ChannelKey,
    pub(super) mount_convention: Option<Convention>,
    pub(super) pins: Option<Pinned>,
    pub(super) nis_window: Option<NisWindow>,
    pub(super) noise: DMatrix<f64>,
    pub(super) model: DeferredComponent<MeasurementWiring, Box<dyn MeasurementSource>>,
}

/// One augmentation entry as declared: the checked block, and the channel of
/// the sensor it calibrates, whose mount it is tied to when built.
pub(super) struct AugmentationPlan {
    pub(super) channel: ChannelKey,
    pub(super) spec: AugmentationSpec,
}

/// Fails unless aiding entry `name`'s built source reads `built`, the channel
/// its kind `declared`. The node's ports carry the declared channel, so a
/// source reading another would find no bus slot and never be updated.
fn check_source_channel(
    name: &str,
    declared: &ChannelKey,
    built: &ChannelKey,
) -> Result<(), String> {
    if built != declared {
        return Err(format!(
            "{AIDING_KEY}.{name}: the source was built reading {built}, but its kind \
             declared {declared}"
        ));
    }
    Ok(())
}

/// Fails unless aiding entry `name`'s built model pins what its kind
/// `declared`: a model whose schema holds the position of `root` pins it, and
/// one whose schema doesn't pins nothing. The node's ports carry the declared
/// pin, so a model disagreeing would leave the root unanchored where its
/// predictions place it, or anchor it where they don't.
fn check_source_pin(
    name: &str,
    declared: Option<Pinned>,
    schema: &MeasurementSchema,
    root: &FrameId,
) -> Result<(), String> {
    let position = Quantity::Position(root.clone());
    let predicted = schema
        .blocks()
        .iter()
        .any(|block| block.quantity() == &position)
        .then_some(Pinned::Position);
    match (declared, predicted) {
        (Some(Pinned::Position), Some(Pinned::Position)) | (None, None) => Ok(()),
        (Some(Pinned::Position), None) => Err(format!(
            "{AIDING_KEY}.{name}: its kind registered that it pins the position of {root}, \
             but the model predicts none"
        )),
        (None, Some(Pinned::Position)) => Err(format!(
            "{AIDING_KEY}.{name}: the model predicts the position of {root}, which pins it, \
             but its kind registered no pin"
        )),
    }
}

/// The mount of `channel`, one of the node's inputs, or why it has none.
fn mount_of(mounts: &InputMounts, channel: &ChannelKey) -> Result<FrameId, String> {
    mounts
        .mount(channel)
        .cloned()
        .map_err(|err| err.to_string())
}

/// Turns every declared augmentation into a state block tagged with its
/// measurement channel's mount. The aiding source reading that channel is built
/// for the same mount, so the block's frame is the one its model reads back.
fn augmentation_blocks(
    plans: &[AugmentationPlan],
    mounts: &InputMounts,
) -> Result<Vec<StateSchemaBlock>, String> {
    plans
        .iter()
        .map(|plan| Ok(plan.spec.block(mount_of(mounts, &plan.channel)?)))
        .collect()
}

/// The filter's starting state: the dynamics' state extended by the
/// augmentation blocks, with P₀ and Q from that schema and the mean pose from
/// `pose`.
///
/// Fails if an aiding model's measurement disagrees with the state (a frame
/// the state can't anchor, a convention it doesn't use), naming the sensor,
/// or if the state can't hold the pose. The state's root was checked when the
/// node was declared ([`state_root_convention`]).
///
/// Built valid at time zero; the node restamps it with the pipeline clock on
/// its first tick.
fn seeded_state(
    dynamics: &DynamicsComponent,
    augmentation: Vec<StateSchemaBlock>,
    aiding: &[Box<dyn MeasurementSource>],
    pose: &InitialPoseConfig,
    ctx: &BuildContext<'_>,
) -> Result<FrameAwareState, String> {
    let base = dynamics.dynamics().schema();
    let schema = if augmentation.is_empty() {
        base
    } else {
        Arc::new(base.extended(augmentation))
    };

    for source in aiding {
        check_measurement_state_agreement(&schema, &source.model().schema(), source.sensor())
            .map_err(|err| format!("aiding channel {}: {err}", source.channel()))?;
    }

    let mut state = FrameAwareState::from_schema(schema, MonotonicTime(0.0));
    dynamics
        .seed_pose(&mut state, ctx.frames(), body_pose(pose))
        .map_err(|err| format!("initial pose: {err}"))?;
    Ok(state)
}

/// The axes the state holds the root the node defines, `ctx`'s, in. Fails
/// unless the state is held in that root: a position, if the state holds one,
/// is in that root, the agent's `odom` appears in the state only when it is
/// that root, and some block names the root.
///
/// The node declares its root from the context, but the dynamics build the
/// state. A dynamics that names `odom` instead of the context's root would
/// publish a shadow estimate in the authoritative estimate's frame while the
/// build believed it held elsewhere. A state with no position (attitude or
/// velocity only) is checked by the second rule alone.
pub(super) fn state_root_convention(
    schema: &StateSchema,
    ctx: &BuildContext<'_>,
) -> Result<Convention, String> {
    let root = &ctx.frames().root;
    if let Some(position) = schema.reference_frame() {
        if &position != root {
            return Err(format!(
                "the dynamics hold the state's position in {position}, but this node holds \
                 its estimate in {root}; dynamics build the state in the frames the build \
                 context gives"
            ));
        }
    }
    let odom = FrameId::odom(ctx.agent().clone());
    if &odom != root && schema.convention_of(&odom).is_some() {
        return Err(format!(
            "the dynamics hold the state in {odom}, the stack estimate's root, but this node \
             holds its estimate in {root}; dynamics build the state in the frames the build \
             context gives"
        ));
    }
    schema.convention_of(root).ok_or_else(|| {
        format!(
            "the dynamics hold nothing in {root}, the root this node holds its estimate in; \
             dynamics build the state in the frames the build context gives"
        )
    })
}

/// The pose of `base_link` in the estimate's root that `pose` describes: a translation and
/// a rotation about up by the heading.
fn body_pose(pose: &InitialPoseConfig) -> Transform<Flu, Enu> {
    let yaw = pose.heading_deg.to_radians();
    Transform::<Flu, Enu>::from_isometry(Isometry3::from_parts(
        Translation3::new(pose.x, pose.y, pose.z),
        UnitQuaternion::from_quaternion(Quaternion::new(
            (yaw / 2.0).cos(),
            0.0,
            0.0,
            (yaw / 2.0).sin(),
        )),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::ChannelKey;

    use helios_core::estimation::schema::MeasurementSchemaBlock;
    use helios_core::interchange::measurement::envelope::SensorReading;
    use helios_core::interchange::measurement::sensor::{GpsPosition, MagneticField};

    /// A source built reading another channel than its kind declared is
    /// refused, since the node's ports carry only the declared one.
    #[test]
    fn a_source_reading_an_undeclared_channel_is_refused() {
        let declared = ChannelKey::measurement::<Vec<SensorReading<GpsPosition>>>("gps");
        let built = ChannelKey::measurement::<Vec<SensorReading<MagneticField>>>("gps");

        assert!(check_source_channel("gps", &declared, &declared).is_ok());
        let err = check_source_channel("gps", &declared, &built).expect_err("channels differ");
        assert!(err.starts_with("aiding.gps:"), "{err}");
    }

    fn root() -> FrameId {
        FrameId::odom(AgentId::new("car"))
    }

    /// A schema predicting `quantity`, in ENU.
    fn predicting(quantity: Quantity) -> MeasurementSchema {
        MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(quantity, Convention::Enu)])
    }

    /// A model predicting the root's position pins it, so its kind must say
    /// so, and one predicting anything else must not.
    #[test]
    fn a_model_pins_what_its_kind_registered() {
        let position = predicting(Quantity::Position(root()));
        let elsewhere = predicting(Quantity::Position(FrameId::global("datum")));

        assert!(check_source_pin("gps", Some(Pinned::Position), &position, &root()).is_ok());
        assert!(check_source_pin("mocap", None, &elsewhere, &root()).is_ok());
    }

    /// A kind that forgets the pin would leave the root unanchored where its
    /// model places it; one that claims a pin would anchor it where its model
    /// doesn't. Both are refused, naming the entry.
    #[test]
    fn a_model_disagreeing_with_its_kinds_pin_is_refused() {
        let position = predicting(Quantity::Position(root()));
        let elsewhere = predicting(Quantity::Position(FrameId::global("datum")));

        let unregistered =
            check_source_pin("gps", None, &position, &root()).expect_err("pin unregistered");
        assert_eq!(
            unregistered,
            "aiding.gps: the model predicts the position of agent.car.odom, which pins it, \
             but its kind registered no pin"
        );
        let unpredicted = check_source_pin("gps", Some(Pinned::Position), &elsewhere, &root())
            .expect_err("pin unpredicted");
        assert_eq!(
            unpredicted,
            "aiding.gps: its kind registered that it pins the position of agent.car.odom, \
             but the model predicts none"
        );
    }
}
