//! Layer 1.5 — the gatherers that build an [`InspectorModel`] from the selection.
//!
//! Each *source* is a pair: a pure `*_section` fn that does the packing (Bevy-free,
//! Tier-1 testable) and a thin system that owns the ECS query — and whose query
//! filter *is* the applies-predicate. [`begin_inspection`] resets the model shell
//! from the selection each frame; every `gather_*` system then appends its
//! section(s). Component-intersection does the routing: a subject that lacks a
//! source's component never matches that gatherer's query, so it silently
//! contributes nothing — no `enum SubjectKind`, no branching. Adding a source is a
//! new pure fn plus a new system in the plugin's chain.

use super::{model::*, CurrentInspection};
use crate::{
    core::transforms::EnuBodyPose, prelude::AutonomyPipelineComponent,
    viz::interaction::selection::Selected,
};

use helios_core::{
    control::commands::BodyTwist,
    data::MonotonicTime,
    frames::{
        conventions::Enu,
        quantities::{FreeVector, Point},
        FrameAwareState,
    },
};
use helios_runtime::channels::control;

use bevy::prelude::*;
use nalgebra::Vector3;
use std::sync::Arc;

// TODO(multi-select): `begin_inspection` takes the first of possibly many Selected
// entities (`iter().next()`), while `gather_pose` uses `single()`, which returns Err
// when more than one entity is Selected. Under single-select (replace-on-click) only
// one is ever Selected, so they agree; once several can be Selected at once, a second
// selection would get a shell here but no pose there. Both must then iterate the
// Selected set the same way.

/// Establishes the model shell — subject id and timestamp, no sections — from the
/// current selection, or clears it to `None`. Runs first in the chain so every
/// downstream `gather_*` has an [`InspectorModel`] to append into.
///
/// `SubjectId` is derived from `Name` here rather than stored as a component, so no
/// Bevy coupling leaks into the pure model layer. `time` is a placeholder — swap it
/// for the same clock `oracle/pose` is stamped with so the model's `at` lines up
/// with bus payload timestamps.
pub fn begin_inspection(
    selected: Query<&Name, With<Selected>>,
    time: Res<Time>,
    mut out: ResMut<CurrentInspection>,
) {
    out.0 = selected.iter().next().map(|name| InspectorModel {
        subject: SubjectId(Arc::from(name.as_str())),
        at: MonotonicTime(time.elapsed_secs_f64()),
        sections: Vec::new(),
    })
}

/// Packs the subject's identity section. Pure — the section-building math, lifted
/// out of [`gather_identity`] so it can be tested without an `App`.
pub fn identity_section(subject: &SubjectId) -> Section {
    Section {
        id: SubsystemPath(Arc::from(format!("subject.{}", subject.0))),
        title: "Identity".into(),
        rows: vec![Row {
            label: "name".into(),
            value: Value::Label(subject.0.to_string()),
        }],
    }
}

/// Appends the identity section. Applies to *any* subject — it reads the shell's
/// own `subject`, so there is no query beyond the model already existing.
pub fn gather_identity(mut out: ResMut<CurrentInspection>) {
    let Some(model) = &mut out.0 else { return };
    let section = identity_section(&model.subject);
    model.sections.push(section);
}

/// Packs the subject's ENU pose section. Pure — takes an already-converted
/// [`EnuBodyPose`], never a Bevy type, so the frame conversion stays in the caller
/// and this fn is the hardware-portable, testable unit.
pub fn pose_section(pose: EnuBodyPose) -> Section {
    let p = pose.0.translation.vector;

    Section {
        id: SubsystemPath(Arc::from("subject.pose")),
        title: "Pose".into(),
        rows: vec![Row {
            label: "position".into(),
            value: Value::Vector(FramedVec3 {
                value: [p.x, p.y, p.z],
                frame: Frame::Enu,
            }),
        }],
    }
}

/// Appends the pose section for the selected subject. Applies to anything with a
/// transform (everything). Owns the Bevy→ENU conversion so [`pose_section`] never
/// touches `GlobalTransform`.
pub fn gather_pose(
    selected: Query<&GlobalTransform, With<Selected>>,
    mut out: ResMut<CurrentInspection>,
) {
    let (Some(model), Ok(gt)) = (&mut out.0, selected.single()) else {
        return;
    };

    model.sections.push(pose_section(EnuBodyPose::from(gt)));
}

/// Packs the ego state estimate into a Section. Pure — the estimate's kinematics
/// are already in its reference (odom) ENU frame, so unlike [`pose_section`] there
/// is no frame conversion here; the components pass straight through tagged
/// [`Frame::Enu`]. Position and velocity are each optional: a layout that omits one
/// simply drops that row.
pub fn estimator_section(state: &FrameAwareState) -> Section {
    let frame = state.reference_frame();
    let position = frame
        .clone()
        .and_then(|f| state.position::<Enu>(f))
        .map(Point::into_inner);
    let velocity = frame
        .and_then(|f| state.velocity::<Enu>(f))
        .map(FreeVector::into_inner);

    let mut rows = Vec::new();
    if let Some(p) = position {
        rows.push(Row {
            label: "position".into(),
            value: Value::Vector(FramedVec3 {
                value: [p.x, p.y, p.z],
                frame: Frame::Enu,
            }),
        });
    }
    if let Some(v) = velocity {
        rows.push(Row {
            label: "velocity".into(),
            value: Value::Vector(FramedVec3 {
                value: [v.x, v.y, v.z],
                frame: Frame::Enu,
            }),
        });
    }

    Section {
        id: SubsystemPath(Arc::from("estimator")),
        title: "Estimator".into(),
        rows,
    }
}

/// Appends the estimator section for the selected subject, when it has one. The query
/// filter `With<AutonomyPipelineComponent>` is the inter-subject predicate — a subject
/// with no pipeline never matches. Whether that pipeline actually contains an estimator
/// is the intra-pipeline predicate, answered at runtime by the pipeline's `read_state`:
/// `None` (no estimator node, or one that has not yet produced a state) contributes
/// nothing, so a mis-wired or cold-start estimator is silent rather than empty.
pub fn gather_estimator(
    selected: Query<&AutonomyPipelineComponent, With<Selected>>,
    mut out: ResMut<CurrentInspection>,
) {
    let (Some(model), Ok(pipeline)) = (&mut out.0, selected.single()) else {
        return;
    };

    let Some(stamped) = pipeline.0.read_state() else {
        return;
    };

    model.sections.push(estimator_section(&stamped.value));
}

/// Packs the controller's latest command into a Section. Pure. A [`BodyTwist`] is a
/// pair of body-FLU vectors, so — unlike the ENU estimate — these rows are tagged
/// [`Frame::Flu`]. A leading `mode` row names the command type, keeping the panel
/// self-describing as more command types (wrench, rate/thrust) join the read surface.
pub fn control_section(control: &BodyTwist) -> Section {
    let rows = vec![
        Row {
            label: "mode".into(),
            value: Value::Label("body twist".into()),
        },
        flu_row("linear", control.linear().raw()),
        flu_row("angular", control.angular().raw()),
    ];

    Section {
        id: SubsystemPath(Arc::from("controller")),
        title: "Controller".into(),
        rows,
    }
}

/// A body-frame 3-vector row, tagged [`Frame::Flu`] — the shape each half of a
/// [`BodyTwist`] (linear, angular) reduces to.
fn flu_row(label: &'static str, v: &Vector3<f64>) -> Row {
    Row {
        label: label.into(),
        value: Value::Vector(FramedVec3 {
            value: [v.x, v.y, v.z],
            frame: Frame::Flu,
        }),
    }
}

/// Appends the controller section for the selected subject, when it has produced a
/// command. Mirrors [`gather_estimator`]: `With<AutonomyPipelineComponent>` is the
/// inter-subject predicate, and a by-name read of the `command` channel is the
/// intra-pipeline one — `None` (no controller node, or one that has not yet fired)
/// contributes nothing.
pub fn gather_controller(
    selected: Query<&AutonomyPipelineComponent, With<Selected>>,
    mut out: ResMut<CurrentInspection>,
) {
    let (Some(model), Ok(pipeline)) = (&mut out.0, selected.single()) else {
        return;
    };

    let Some(stamped) = pipeline
        .0
        .bus()
        .read::<BodyTwist>(control::command::<BodyTwist>().into())
    else {
        return;
    };

    model.sections.push(control_section(&stamped.value));
}

#[cfg(test)]
mod tests {
    use super::*;

    use helios_core::estimation::schema::{Quantity, SchemaBlock, StateSchema};
    use helios_core::frames::quantities::FluVector;
    use helios_core::frames::{FrameId, StateVariable};
    use nalgebra::{DMatrix, DVector};
    use helios_runtime::{
        channels::control,
        pipeline::node::HOST_PRODUCER_ID,
        port::{InternalChannel, PortBus},
        prelude::{
            AgentRuntime, Health, PipelineBuilder, PipelineNode, PortDescriptor, Stamped,
            TickContext,
        },
    };

    use nalgebra::Isometry3;

    /// A bare app carrying just the inspector's resources, ready to run one gather
    /// system against a hand-built world.
    fn inspector_app() -> App {
        let mut app = App::new();
        app.init_resource::<CurrentInspection>();
        app.init_resource::<Time>();
        app
    }

    /// An empty model shell for a known subject — what `begin_inspection` leaves for
    /// the gatherers to fill.
    fn sample_shell() -> InspectorModel {
        InspectorModel {
            subject: SubjectId(Arc::from("robot_3")),
            at: MonotonicTime(0.0),
            sections: Vec::new(),
        }
    }

    /// Tier 1: the identity section names the subject and addresses it under the
    /// `subject.<id>` path. Total over every subject — a tree hits this same fn.
    #[test]
    fn identity_section_carries_the_subject_name() {
        let section = identity_section(&SubjectId(Arc::from("robot_3")));

        assert_eq!(section.id, SubsystemPath(Arc::from("subject.robot_3")));
        assert_eq!(
            section.rows,
            vec![Row {
                label: "name".into(),
                value: Value::Label("robot_3".into()),
            }],
        );
    }

    /// Tier 1: the pose section copies the ENU translation straight through, tagged
    /// `Frame::Enu`. The conversion is the caller's job, so this fn is graded purely
    /// on faithful packing — `[x, y, z]` in, `[x, y, z]` out.
    #[test]
    fn pose_section_packs_the_enu_translation() {
        let pose = EnuBodyPose(Isometry3::translation(1.0, 2.0, 3.0));

        let section = pose_section(pose);

        assert_eq!(
            section.rows[0].value,
            Value::Vector(FramedVec3 {
                value: [1.0, 2.0, 3.0],
                frame: Frame::Enu,
            }),
        );
    }

    /// Tier 2: a selected, named entity produces a shell — subject set, sections
    /// left empty for the gatherers.
    #[test]
    fn begin_inspection_builds_a_shell_from_the_selection() {
        let mut app = inspector_app();
        app.world_mut().spawn((Name::new("robot_3"), Selected));
        app.add_systems(Update, begin_inspection);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .expect("a selection builds a shell");
        assert_eq!(model.subject, SubjectId(Arc::from("robot_3")));
        assert!(
            model.sections.is_empty(),
            "begin leaves sections to the gatherers"
        );
    }

    /// Tier 2: with nothing selected, `begin_inspection` clears any stale model —
    /// the "no waste when idle" invariant, enforced as an ECS effect.
    #[test]
    fn begin_inspection_clears_when_nothing_is_selected() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.add_systems(Update, begin_inspection);

        app.update();

        assert!(app.world().resource::<CurrentInspection>().0.is_none());
    }

    /// Tier 2: `gather_identity` appends the identity section to an existing shell,
    /// reading the subject off the model rather than the world.
    #[test]
    fn gather_identity_appends_the_identity_section() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.add_systems(Update, gather_identity);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert_eq!(model.sections.len(), 1);
        assert_eq!(
            model.sections[0].id,
            SubsystemPath(Arc::from("subject.robot_3")),
        );
    }

    /// Tier 2: `gather_pose` appends a pose section for the selected entity. The
    /// query filter (`With<Selected>`) plus the Bevy→ENU conversion are the wiring
    /// under test — the numeric conversion itself is covered in `transforms`, so
    /// this asserts the section arrives, not its coordinates.
    #[test]
    fn gather_pose_appends_a_pose_section_for_the_selection() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.world_mut().spawn((
            Selected,
            GlobalTransform::from_translation(Vec3::new(1.0, 2.0, 3.0)),
        ));
        app.add_systems(Update, gather_pose);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert_eq!(model.sections.len(), 1);
        assert_eq!(
            model.sections[0].id,
            SubsystemPath(Arc::from("subject.pose"))
        );
    }

    /// A minimal state carrying only World-frame position and velocity — enough to
    /// exercise the estimator section without the full 16-state INS layout.
    fn state_with(position: [f64; 3], velocity: [f64; 3]) -> FrameAwareState {
        let flat = |quantity: Quantity| {
            SchemaBlock::new(quantity, None, DVector::zeros(3), DMatrix::zeros(3, 3))
        };
        let schema = StateSchema::compose(vec![
            flat(Quantity::Position(FrameId::World)),
            flat(Quantity::Velocity(FrameId::World)),
        ]);
        let mut state = FrameAwareState::from_schema(Arc::new(schema), 0.0);
        state.set_variable(&StateVariable::Px(FrameId::World), position[0]);
        state.set_variable(&StateVariable::Py(FrameId::World), position[1]);
        state.set_variable(&StateVariable::Pz(FrameId::World), position[2]);
        state.set_variable(&StateVariable::Vx(FrameId::World), velocity[0]);
        state.set_variable(&StateVariable::Vy(FrameId::World), velocity[1]);
        state.set_variable(&StateVariable::Vz(FrameId::World), velocity[2]);
        state
    }

    /// A stand-in estimator node: it declares the canonical `FrameAwareState` output so
    /// the bus allocates that slot, but computes nothing. A test writes the state onto
    /// the bus directly — standing in for a real estimator having produced one, which is
    /// exactly what `gather_estimator` reads back through `read_state`.
    struct FakeEstimatorNode {
        descriptor: PortDescriptor,
    }

    impl FakeEstimatorNode {
        fn new() -> Self {
            Self {
                descriptor: PortDescriptor {
                    required_inputs: Vec::new(),
                    optional_inputs: Vec::new(),
                    outputs: vec![InternalChannel::of::<FrameAwareState>().into()],
                    rate: None,
                },
            }
        }
    }

    impl PipelineNode for FakeEstimatorNode {
        fn name(&self) -> &str {
            "fake_estimator"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _runtime: &dyn AgentRuntime, _tick: TickContext) {}
    }

    /// An `AutonomyPipelineComponent` whose pipeline carries the estimator slot,
    /// optionally with a state already written to it (`Some` = a produced estimate;
    /// `None` = the cold-start / no-value case).
    fn pipeline_with_estimate(state: Option<FrameAwareState>) -> AutonomyPipelineComponent {
        let pipeline = PipelineBuilder::new()
            .add_node(Box::new(FakeEstimatorNode::new()))
            .build()
            .expect("a single-node pipeline builds");

        if let Some(state) = state {
            pipeline
                .bus()
                .write(
                    InternalChannel::of::<FrameAwareState>().into(),
                    Stamped {
                        value: state,
                        timestamp: MonotonicTime(0.0),
                        health: Health::Ok,
                        producer: HOST_PRODUCER_ID,
                    },
                )
                .expect("the estimator slot exists on the bus");
        }

        AutonomyPipelineComponent(pipeline)
    }

    /// Tier 1: the estimator section copies World-frame position and velocity straight
    /// through, each tagged `Frame::Enu`. Like `pose_section`, the frame is the caller's
    /// guarantee (the estimate is already ENU), so this fn is graded on faithful packing.
    #[test]
    fn estimator_section_packs_position_and_velocity() {
        let state = state_with([7.30, -10.45, 0.37], [-1.54, 0.94, -0.02]);

        let section = estimator_section(&state);

        assert_eq!(section.id, SubsystemPath(Arc::from("estimator")));
        assert_eq!(
            section.rows,
            vec![
                Row {
                    label: "position".into(),
                    value: Value::Vector(FramedVec3 {
                        value: [7.30, -10.45, 0.37],
                        frame: Frame::Enu,
                    }),
                },
                Row {
                    label: "velocity".into(),
                    value: Value::Vector(FramedVec3 {
                        value: [-1.54, 0.94, -0.02],
                        frame: Frame::Enu,
                    }),
                },
            ],
        );
    }

    /// Tier 2, "with estimate" direction: a selected subject whose estimator has produced
    /// a state gets exactly the estimator section — the intra-pipeline predicate resolving
    /// positive at runtime.
    #[test]
    fn gather_estimator_appends_a_section_when_a_state_is_present() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.world_mut().spawn((
            Selected,
            pipeline_with_estimate(Some(state_with([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]))),
        ));
        app.add_systems(Update, gather_estimator);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SubsystemPath(Arc::from("estimator")));
    }

    /// Tier 2, "no estimate" direction: a pipeline whose estimator has not produced a
    /// state (`read_state` is `None`) contributes no section — the guard against a
    /// silently empty panel. A pipeline with no estimator node at all hits this same
    /// branch, so this one test covers both absence cases Part A treats alike.
    #[test]
    fn gather_estimator_is_silent_without_a_state() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.world_mut()
            .spawn((Selected, pipeline_with_estimate(None)));
        app.add_systems(Update, gather_estimator);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert!(model.sections.is_empty());
    }

    /// A stand-in controller node: it declares the canonical `BodyTwist` output so
    /// the bus allocates that slot but computes nothing. A test writes the command onto
    /// the bus directly, standing in for a real controller having produced one — exactly
    /// what `gather_controller` reads back by name off the `command` channel.
    struct FakeControllerNode {
        descriptor: PortDescriptor,
    }

    impl FakeControllerNode {
        fn new() -> Self {
            Self {
                descriptor: PortDescriptor {
                    required_inputs: Vec::new(),
                    optional_inputs: Vec::new(),
                    outputs: vec![control::command::<BodyTwist>().into()],
                    rate: None,
                },
            }
        }
    }

    impl PipelineNode for FakeControllerNode {
        fn name(&self) -> &str {
            "fake_controller"
        }

        fn port_descriptor(&self) -> &PortDescriptor {
            &self.descriptor
        }

        fn execute(&self, _bus: &PortBus, _runtime: &dyn AgentRuntime, _tick: TickContext) {}
    }

    /// An `AutonomyPipelineComponent` whose pipeline carries the controller slot,
    /// optionally with a command already written to it (`Some` = a produced command;
    /// `None` = the no-command case a cold-start or controller-less stack presents).
    fn pipeline_with_control(control: Option<BodyTwist>) -> AutonomyPipelineComponent {
        let pipeline = PipelineBuilder::new()
            .add_node(Box::new(FakeControllerNode::new()))
            .build()
            .expect("a single-node pipeline builds");

        if let Some(control) = control {
            pipeline
                .bus()
                .write(
                    control::command::<BodyTwist>().into(),
                    Stamped {
                        value: control,
                        timestamp: MonotonicTime(0.0),
                        health: Health::Ok,
                        producer: HOST_PRODUCER_ID,
                    },
                )
                .expect("the controller slot exists on the bus");
        }

        AutonomyPipelineComponent(pipeline)
    }

    /// Tier 1: a body twist packs a `mode` label plus its linear and angular
    /// vectors, each tagged `Frame::Flu` — the control frame, distinct from the ENU
    /// estimate. The `mode` row keeps the command self-describing once other command
    /// types (wrench, rate/thrust) share the panel.
    #[test]
    fn control_section_packs_body_twist() {
        let control = BodyTwist::new(FluVector::new(1.5, 0.0, 0.0), FluVector::new(0.0, 0.0, 0.3));

        let section = control_section(&control);

        assert_eq!(section.id, SubsystemPath(Arc::from("controller")));
        assert_eq!(
            section.rows,
            vec![
                Row {
                    label: "mode".into(),
                    value: Value::Label("body twist".into()),
                },
                Row {
                    label: "linear".into(),
                    value: Value::Vector(FramedVec3 {
                        value: [1.5, 0.0, 0.0],
                        frame: Frame::Flu,
                    }),
                },
                Row {
                    label: "angular".into(),
                    value: Value::Vector(FramedVec3 {
                        value: [0.0, 0.0, 0.3],
                        frame: Frame::Flu,
                    }),
                },
            ],
        );
    }

    /// Tier 2, "with command" direction: a selected subject whose controller has produced
    /// a command gets exactly the controller section — the intra-pipeline predicate
    /// resolving positive at runtime.
    #[test]
    fn gather_controller_appends_a_section_when_control_is_present() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.world_mut().spawn((
            Selected,
            pipeline_with_control(Some(BodyTwist::new(
                FluVector::new(1.0, 0.0, 0.0),
                FluVector::zeros(),
            ))),
        ));
        app.add_systems(Update, gather_controller);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SubsystemPath(Arc::from("controller")));
    }

    /// Tier 2, "no command" direction: a pipeline whose controller has not produced a
    /// command (no `command` on the bus) contributes no section. A pipeline with no
    /// controller node at all hits this same branch, so this covers both absence cases
    /// Part A treats alike.
    #[test]
    fn gather_controller_is_silent_without_control() {
        let mut app = inspector_app();
        app.world_mut().resource_mut::<CurrentInspection>().0 = Some(sample_shell());
        app.world_mut()
            .spawn((Selected, pipeline_with_control(None)));
        app.add_systems(Update, gather_controller);

        app.update();

        let model = app
            .world()
            .resource::<CurrentInspection>()
            .0
            .clone()
            .unwrap();
        assert!(model.sections.is_empty());
    }
}
