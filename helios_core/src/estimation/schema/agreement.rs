use crate::{
    estimation::schema::{
        InputSchema, InputSchemaBlock, MeasurementSchema, MeasurementSchemaBlock, StateSchema,
    },
    spatial::state::Quantity,
    spatial::{transforms::Convention, FrameId},
};

/// Checks that a measurement schema is expressed compatibly with the state it
/// aids, one frame at a time.
///
/// For each frame a measurement block references, the state either assigns that
/// frame a convention — which the measurement must match — or it does not, in
/// which case the frame must be the sensor's own device frame, whose convention
/// is the sensor model's concern and is accepted here as declared; anything else
/// is unanchorable. This one rule subsumes both a direct readout (the state
/// tracks the same quantity, so its frame is in the map) and a derived
/// measurement (the state tracks only the frame): both reduce to looking the
/// frame up in the state's convention map. A disagreement is a construction-time
/// config error, returned rather than panicked so the assembler can name the
/// offending estimator. Does no coordinate math: the frame crossing is the
/// measurement model's own concern, and this only validates the declarations
/// line up.
pub fn check_measurement_state_agreement(
    state: &StateSchema,
    measurement: &MeasurementSchema,
) -> Result<(), MeasurementAgreementError> {
    for block in measurement.blocks() {
        check_block(block, state)?;
    }
    Ok(())
}

/// Checks one measurement block: every frame it references must agree with the
/// state's convention for that frame, or be a sensor's own device frame.
fn check_block(
    block: &MeasurementSchemaBlock,
    state: &StateSchema,
) -> Result<(), MeasurementAgreementError> {
    for (frame, convention) in &block.conventions {
        match state.convention_of(frame) {
            // The state anchors this frame under a different convention. This one
            // arm covers both a direct readout (the state tracks the same quantity)
            // and a derived measurement (the state tracks only the frame): if the
            // state anchors the frame at all, the map holds the convention to match.
            Some(state_convention) if state_convention != *convention => {
                return Err(MeasurementAgreementError::FrameConventionMismatch {
                    quantity: block.quantity().clone(),
                    frame: frame.clone(),
                    state: state_convention,
                    measurement: *convention,
                });
            }
            // The state anchors this frame and the conventions agree.
            Some(_) => {}
            // Not anchored by the state, but the sensor's own device frame — its
            // convention is the sensor model's to declare, not ours to check.
            None if frame.is_sensor() => {}
            // Neither anchored by the state nor a sensor frame: nothing to check
            // against.
            None => {
                return Err(MeasurementAgreementError::UnanchorableFrame {
                    quantity: block.quantity().clone(),
                    frame: frame.clone(),
                });
            }
        }
    }
    Ok(())
}

/// Why a measurement schema fails to agree with the state it aids. Every variant
/// is a construction-time config error; none can arise once a pipeline is built.
#[derive(Debug)]
pub enum MeasurementAgreementError {
    /// The state expresses the measurement's frame, but under a different
    /// convention than the measurement claims for it.
    FrameConventionMismatch {
        quantity: Quantity,
        frame: FrameId,
        state: Convention,
        measurement: Convention,
    },
    /// The measurement's frame is neither tracked by the state nor a sensor
    /// frame, so no convention exists to check it against.
    UnanchorableFrame { quantity: Quantity, frame: FrameId },
    /// The measurement schema's innovation length disagrees with the size of the
    /// noise matrix `R` it is paired with.
    DimensionMismatch { schema_dim: usize, expected: usize },
}

impl std::fmt::Display for MeasurementAgreementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FrameConventionMismatch {
                quantity,
                frame,
                state,
                measurement,
            } => write!(
                f,
                "measurement of {quantity} in {frame} is declared {measurement}, \
                 but the state expresses {frame} in {state}. A measurement \
                 and the state must agree on a frame's axis convention."
            ),
            Self::UnanchorableFrame { quantity, frame } => write!(
                f,
                "measurement of {quantity} references {frame}, which the state neither tracks \
                 nor recognizes as a sensor frame; there is no convention to check it against. \
                 Add this frame to the estimator's state, or fix the sensor's target frame."
            ),
            Self::DimensionMismatch {
                schema_dim,
                expected,
            } => write!(
                f,
                "measurement schema has innovation dimension {schema_dim}, but the noise matrix R \
                 is {expected}x{expected}; the measurement and its R must have the same size."
            ),
        }
    }
}

impl std::error::Error for MeasurementAgreementError {}

/// Checks that the input an input builder assembles is the input the dynamics
/// consume: the same blocks, in the same order, each with the same quantity,
/// frame and conventions.
///
/// `u` is positional, so agreement is exact equality block by block — no
/// reordering, no conversion. Length alone is not enough: an accelerometer and
/// a gyro block are both three long, so a swap passes a length check and then
/// integrates angular rate as specific force. Returned rather than panicked so
/// the assembler can name the offending estimator. Does no coordinate math:
/// moving an input into the frame the dynamics expect is the input builder's
/// job, and this only validates that the declarations line up.
pub fn check_input_agreement(
    expected: &InputSchema,
    supplied: &InputSchema,
) -> Result<(), InputAgreementError> {
    for (index, (want, got)) in expected.blocks().iter().zip(supplied.blocks()).enumerate() {
        if want != got {
            return Err(InputAgreementError::BlockMismatch {
                index,
                expected: Box::new(want.clone()),
                supplied: Box::new(got.clone()),
            });
        }
    }

    if expected.blocks().len() != supplied.blocks().len() {
        return Err(InputAgreementError::BlockCountMismatch {
            expected: expected.blocks().len(),
            supplied: supplied.blocks().len(),
        });
    }

    Ok(())
}

/// Why a supplied input fails to agree with the input the dynamics consume.
/// Every variant is a construction-time config error.
#[derive(Debug)]
pub enum InputAgreementError {
    /// The block at `index` differs in quantity, frame, or convention. Boxed
    /// to keep the `Result` small; this only arises at construction.
    BlockMismatch {
        index: usize,
        expected: Box<InputSchemaBlock>,
        supplied: Box<InputSchemaBlock>,
    },
    /// Every shared position agrees, but one side has more blocks.
    BlockCountMismatch { expected: usize, supplied: usize },
}

impl std::fmt::Display for InputAgreementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BlockMismatch {
                index,
                expected,
                supplied,
            } => write!(
                f,
                "input block {index}: the dynamics consume {}, but the input builder \
                 supplies {}. The input vector is positional, so both sides must declare \
                 the same quantities, frames and conventions in the same order.",
                describe_block(expected),
                describe_block(supplied),
            ),
            Self::BlockCountMismatch { expected, supplied } => write!(
                f,
                "the dynamics consume {expected} input blocks, but the input builder \
                 supplies {supplied}."
            ),
        }
    }
}

impl std::error::Error for InputAgreementError {}

/// A block as `quantity [frame: convention, …]`, for error messages.
fn describe_block(block: &InputSchemaBlock) -> String {
    let conventions: Vec<String> = block
        .conventions()
        .iter()
        .map(|(frame, convention)| format!("{frame}: {convention}"))
        .collect();
    format!("{} [{}]", block.quantity(), conventions.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimation::schema::StateSchemaBlock;
    use crate::kernel::manifold::TangentNoise;
    use crate::prelude::AgentId;

    use nalgebra::{DMatrix, DVector};

    // A single agent and a sensor mounted on it. Distinct leaves so a frame
    // mixup would surface as a mismatched id rather than an accidental match.
    fn agent() -> AgentId {
        AgentId::new("test_agent")
    }
    fn sensor() -> FrameId {
        FrameId::sensor(agent(), "sensor0")
    }

    fn noise(var: f64) -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, var)).unwrap())
    }

    /// A state that anchors two frames' conventions: `Odom(agent) → ENU` via the
    /// position block, and `Body(agent) → FLU` via the body↦odom orientation.
    /// `World` is deliberately left untracked so it can stand in for the
    /// unanchorable case. Neither block observes `SpecificForce`, so any
    /// measurement of it references a frame the state anchors without the state
    /// tracking the same quantity.
    fn anchored_state() -> StateSchema {
        StateSchema::compose(vec![
            StateSchemaBlock::new(
                Quantity::Position(FrameId::odom(agent())),
                Convention::Enu,
                noise(0.1),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            StateSchemaBlock::orientation(
                FrameId::base_link(agent()),
                FrameId::odom(agent()),
                Convention::Flu,
                Convention::Enu,
                noise(0.1),
                DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0]),
                DMatrix::identity(3, 3),
            ),
        ])
    }

    /// A one-block measurement schema — the public check runs per block, so a
    /// single block isolates exactly one outcome and its error becomes the
    /// function's return verbatim.
    fn one(quantity: Quantity, convention: Convention) -> MeasurementSchema {
        MeasurementSchema::compose(vec![MeasurementSchemaBlock::new(quantity, convention)])
    }

    // ── Direct readout: measurement quantity is tracked by the state ──────────

    #[test]
    fn direct_readout_with_matching_convention_agrees() {
        let state = anchored_state();
        let m = one(Quantity::Position(FrameId::odom(agent())), Convention::Enu);
        assert!(check_measurement_state_agreement(&state, &m).is_ok());
    }

    #[test]
    fn direct_readout_with_wrong_convention_is_a_frame_convention_mismatch() {
        // Same quantity the state tracks (position in odom), but declared FLU
        // where the state holds ENU: the innovation would mix axis conventions.
        let state = anchored_state();
        let m = one(Quantity::Position(FrameId::odom(agent())), Convention::Flu);

        let err = check_measurement_state_agreement(&state, &m).unwrap_err();
        assert!(matches!(
            err,
            MeasurementAgreementError::FrameConventionMismatch { .. }
        ));
    }

    // ── Derived, frame tracked by the state ───────────────────────────────────

    #[test]
    fn derived_measurement_in_a_tracked_frame_agrees_on_matching_convention() {
        // SpecificForce is untracked (derived), but its frame — Body(agent) — is
        // anchored FLU by the state's orientation block, and the measurement
        // declares FLU too.
        let state = anchored_state();
        let m = one(
            Quantity::SpecificForce(FrameId::base_link(agent())),
            Convention::Flu,
        );
        assert!(check_measurement_state_agreement(&state, &m).is_ok());
    }

    #[test]
    fn derived_measurement_in_a_tracked_frame_with_wrong_convention_is_a_frame_convention_mismatch()
    {
        // Body(agent) is anchored FLU by the state; the measurement claims ENU.
        // The state tracks the frame but not this quantity, yet it lands on the
        // same `FrameConventionMismatch` as a direct readout — which is the point
        // of folding the two cases into one frame lookup.
        let state = anchored_state();
        let m = one(
            Quantity::SpecificForce(FrameId::base_link(agent())),
            Convention::Enu,
        );

        match check_measurement_state_agreement(&state, &m).unwrap_err() {
            MeasurementAgreementError::FrameConventionMismatch {
                frame,
                state,
                measurement,
                ..
            } => {
                assert_eq!(frame, FrameId::base_link(agent()));
                assert_eq!(state, Convention::Flu);
                assert_eq!(measurement, Convention::Enu);
            }
            other => panic!("expected FrameConventionMismatch, got {other:?}"),
        }
    }

    // ── Derived, frame absent from the state ──────────────────────────────────

    #[test]
    fn derived_measurement_in_its_own_sensor_frame_is_accepted_in_any_convention() {
        // The state tracks nothing in the sensor's frame, so the sensor model —
        // not this check — owns its convention. The measurement is accepted
        // whether it declares FLU (an IMU) or otherwise (a camera's optical
        // frame); there is nothing here to compare it against.
        let state = anchored_state();
        for convention in [Convention::Flu, Convention::Enu] {
            let m = one(Quantity::SpecificForce(sensor()), convention);
            assert!(check_measurement_state_agreement(&state, &m).is_ok());
        }
    }

    #[test]
    fn derived_measurement_in_an_untracked_non_sensor_frame_is_unanchorable() {
        // World is neither tracked by this state nor a sensor frame, so there is
        // no convention to check against.
        let state = anchored_state();
        let m = one(Quantity::SpecificForce(FrameId::world()), Convention::Enu);

        match check_measurement_state_agreement(&state, &m).unwrap_err() {
            MeasurementAgreementError::UnanchorableFrame { frame, .. } => {
                assert_eq!(frame, FrameId::world());
            }
            other => panic!("expected UnanchorableFrame, got {other:?}"),
        }
    }

    // ── The public loop over many blocks ──────────────────────────────────────

    #[test]
    fn a_fully_agreeing_multi_block_measurement_passes() {
        let state = anchored_state();
        let m = MeasurementSchema::compose(vec![
            MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::odom(agent())),
                Convention::Enu,
            ),
            MeasurementSchemaBlock::new(Quantity::SpecificForce(sensor()), Convention::Flu),
        ]);
        assert!(check_measurement_state_agreement(&state, &m).is_ok());
    }

    #[test]
    fn one_bad_block_fails_the_whole_measurement() {
        // The first block disagrees; `?` short-circuits, so the second (valid)
        // block is never reached and the first block's error surfaces.
        let state = anchored_state();
        let m = MeasurementSchema::compose(vec![
            MeasurementSchemaBlock::new(
                Quantity::Position(FrameId::odom(agent())),
                Convention::Flu,
            ),
            MeasurementSchemaBlock::new(Quantity::SpecificForce(sensor()), Convention::Flu),
        ]);

        assert!(matches!(
            check_measurement_state_agreement(&state, &m).unwrap_err(),
            MeasurementAgreementError::FrameConventionMismatch { .. }
        ));
    }

    // ── Input agreement: builder vs dynamics ──────────────────────────────────

    fn specific_force(convention: Convention) -> InputSchemaBlock {
        InputSchemaBlock::new(
            Quantity::SpecificForce(FrameId::base_link(agent())),
            convention,
        )
    }

    fn angular_velocity(convention: Convention) -> InputSchemaBlock {
        InputSchemaBlock::new(
            Quantity::AngularVelocity(FrameId::base_link(agent())),
            convention,
        )
    }

    /// The IMU input: specific force then angular velocity, both body FLU.
    fn imu_input() -> InputSchema {
        InputSchema::compose(vec![
            specific_force(Convention::Flu),
            angular_velocity(Convention::Flu),
        ])
    }

    #[test]
    fn identical_inputs_agree() {
        assert!(check_input_agreement(&imu_input(), &imu_input()).is_ok());
    }

    #[test]
    fn an_accel_gyro_swap_is_a_block_mismatch_at_the_first_block() {
        // Same length, same frames, same conventions — only the order differs.
        // A length check would pass this and integrate rate as force.
        let swapped = InputSchema::compose(vec![
            angular_velocity(Convention::Flu),
            specific_force(Convention::Flu),
        ]);
        assert_eq!(swapped.dim(), imu_input().dim());

        match check_input_agreement(&imu_input(), &swapped).unwrap_err() {
            InputAgreementError::BlockMismatch {
                index,
                expected,
                supplied,
            } => {
                assert_eq!(index, 0);
                assert_eq!(*expected, specific_force(Convention::Flu));
                assert_eq!(*supplied, angular_velocity(Convention::Flu));
            }
            other => panic!("expected BlockMismatch, got {other:?}"),
        }
    }

    #[test]
    fn a_convention_mismatch_is_a_block_mismatch() {
        let enu_gyro = InputSchema::compose(vec![
            specific_force(Convention::Flu),
            angular_velocity(Convention::Enu),
        ]);

        assert!(matches!(
            check_input_agreement(&imu_input(), &enu_gyro).unwrap_err(),
            InputAgreementError::BlockMismatch { index: 1, .. }
        ));
    }

    #[test]
    fn an_input_in_the_sensor_frame_is_a_block_mismatch() {
        // Raw IMU data lives in the sensor frame; the dynamics consume it in
        // base_link. Supplying it unrotated is exactly what this check refuses.
        let raw = InputSchema::compose(vec![
            InputSchemaBlock::new(Quantity::SpecificForce(sensor()), Convention::Flu),
            InputSchemaBlock::new(Quantity::AngularVelocity(sensor()), Convention::Flu),
        ]);

        assert!(matches!(
            check_input_agreement(&imu_input(), &raw).unwrap_err(),
            InputAgreementError::BlockMismatch { index: 0, .. }
        ));
    }

    #[test]
    fn a_missing_trailing_block_is_a_block_count_mismatch() {
        let accel_only = InputSchema::compose(vec![specific_force(Convention::Flu)]);

        match check_input_agreement(&imu_input(), &accel_only).unwrap_err() {
            InputAgreementError::BlockCountMismatch { expected, supplied } => {
                assert_eq!((expected, supplied), (2, 1));
            }
            other => panic!("expected BlockCountMismatch, got {other:?}"),
        }
    }

    #[test]
    fn a_block_mismatch_message_names_both_quantities() {
        let swapped = InputSchema::compose(vec![
            angular_velocity(Convention::Flu),
            specific_force(Convention::Flu),
        ]);
        let message = check_input_agreement(&imu_input(), &swapped)
            .unwrap_err()
            .to_string();

        assert!(message.contains("input block 0"), "{message}");
        assert!(
            message.contains(&Quantity::SpecificForce(FrameId::base_link(agent())).to_string()),
            "{message}"
        );
        assert!(
            message.contains(&Quantity::AngularVelocity(FrameId::base_link(agent())).to_string()),
            "{message}"
        );
    }
}
