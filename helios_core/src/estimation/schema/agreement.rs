use crate::{
    estimation::schema::{
        BlockConvention, MeasurementSchema, MeasurementSchemaBlock, StateSchema, StateSchemaBlock,
    },
    frames::{transforms::Convention, FrameId},
    state::Quantity,
};

use std::collections::HashMap;

/// Checks that a measurement schema is expressed compatibly with the state it
/// aids, one block at a time.
///
/// For each measurement block, either the state tracks the same quantity — in
/// which case their conventions must match — or it does not, in which case the
/// block's frame must either be one the state assigns a convention, which the
/// measurement must then match, or the sensor's own device frame, whose
/// convention is the sensor model's concern and is accepted here as declared. A
/// disagreement is a construction-time config error, returned rather than
/// panicked so the assembler can name the offending estimator. Does no
/// coordinate math: the frame crossing is the measurement model's own concern,
/// and this only validates the declarations line up.
pub fn check_measurement_state_agreement(
    state: &StateSchema,
    measurement: &MeasurementSchema,
) -> Result<(), MeasurementAgreementError> {
    let by_frame = frame_conventions(state);
    for block in measurement.blocks() {
        check_block(block, state, &by_frame)?
    }
    Ok(())
}

fn frame_conventions(state: &StateSchema) -> HashMap<FrameId, Convention> {
    let mut by_frame = HashMap::new();

    for block in state.blocks() {
        match (block.quantity(), block.convention()) {
            (q, BlockConvention::Single(conv)) => {
                let f = q.frame().expect("Single convention ⇒ flat quantity");
                by_frame.insert(f.clone(), *conv);
            }
            (
                Quantity::Orientation { from, to },
                BlockConvention::Pair {
                    from: from_conv,
                    to: to_conv,
                },
            ) => {
                by_frame.insert(from.clone(), *from_conv);
                by_frame.insert(to.clone(), *to_conv);
            }
            _ => unreachable!("constructors keep quantity arity and convention arity in lockstep"),
        };
    }

    by_frame
}

fn check_block(
    block: &MeasurementSchemaBlock,
    state: &StateSchema,
    by_frame: &HashMap<FrameId, Convention>,
) -> Result<(), MeasurementAgreementError> {
    match find_state_twin(state, block.quantity()) {
        Some(twin) => {
            if conventions_agree(block.convention(), twin.convention()) {
                Ok(())
            } else {
                Err(MeasurementAgreementError::ConventionMismatch {
                    quantity: block.quantity().clone(),
                    state: twin.convention().clone(),
                    measurement: block.convention().clone(),
                })
            }
        }
        None => {
            // Derived: no state block observes this quantity, so there is no twin
            // to compare against. Fall back to checking the block's frame — the
            // two extractions below are total because a composed measurement block
            // is always flat (orientation measurements are refused at compose).
            let Some(m_frame) = block.quantity().frame() else {
                unreachable!("a composed measurement block is flat; a flat quantity has a frame")
            };
            let BlockConvention::Single(m_convention) = block.convention() else {
                unreachable!("a composed measurement block is flat; its convention is Single")
            };

            match by_frame.get(m_frame) {
                // The state expresses this frame: the measurement must use the
                // same convention the state assigned it.
                Some(state_convention) => {
                    if m_convention == state_convention {
                        Ok(())
                    } else {
                        Err(MeasurementAgreementError::FrameEndpointMismatch {
                            quantity: block.quantity().clone(),
                            frame: m_frame.clone(),
                            state_frame_convention: *state_convention,
                            measurement_convention: *m_convention,
                        })
                    }
                }
                // The state never anchors this frame. A sensor's own device frame is
                // the sensor model's to describe; anything else has nothing to check
                // against and cannot be anchored.
                None => {
                    if matches!(m_frame, FrameId::Sensor(_)) {
                        // Its convention is the sensor model's to declare and verify —
                        // FLU for an IMU, optical for a camera — so there is nothing to
                        // compare here. Accept it.
                        Ok(())
                    } else {
                        Err(MeasurementAgreementError::UnanchorableFrame {
                            quantity: block.quantity().clone(),
                            frame: m_frame.clone(),
                        })
                    }
                }
            }
        }
    }
}

fn find_state_twin<'a>(
    state: &'a StateSchema,
    quantity: &Quantity,
) -> Option<&'a StateSchemaBlock> {
    state.blocks().iter().find(|b| quantity == b.quantity())
}

fn conventions_agree(measurement: &BlockConvention, state: &BlockConvention) -> bool {
    match (measurement, state) {
        (BlockConvention::Single(a), BlockConvention::Single(b)) => a == b,
        (
            BlockConvention::Pair { from: mf, to: mt },
            BlockConvention::Pair { from: sf, to: st },
        ) => mf == sf && mt == st,
        _ => false,
    }
}

/// Why a measurement schema fails to agree with the state it aids. Every variant
/// is a construction-time config error; none can arise once a pipeline is built.
#[derive(Debug)]
pub enum MeasurementAgreementError {
    /// The state tracks this exact quantity, but declares it in a different
    /// convention than the measurement does — a direct-readout mismatch.
    ConventionMismatch {
        quantity: Quantity,
        state: BlockConvention,
        measurement: BlockConvention,
    },
    /// The state expresses the measurement's frame, but under a different
    /// convention than the measurement claims for it.
    FrameEndpointMismatch {
        quantity: Quantity,
        frame: FrameId,
        state_frame_convention: Convention,
        measurement_convention: Convention,
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
            Self::ConventionMismatch {
                quantity,
                state,
                measurement,
            } => write!(
                f,
                "measurement of {quantity} is declared in {measurement}, but the state \
                 tracks it in {state}; the innovation z - h(x) would combine components in \
                 two different axis conventions. Align the measurement model's schema with the \
                 state's convention for this quantity."
            ),
            Self::FrameEndpointMismatch {
                quantity,
                frame,
                state_frame_convention,
                measurement_convention,
            } => write!(
                f,
                "measurement of {quantity} in {frame} is declared {measurement_convention}, \
                 but the state expresses {frame} in {state_frame_convention}. A measurement \
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::primitives::FrameHandle;
    use crate::manifold::TangentNoise;

    use nalgebra::{DMatrix, DVector};

    // A single agent and a sensor mounted on it. Distinct handles so a frame
    // mixup would surface as a mismatched id rather than an accidental match.
    fn agent() -> FrameHandle {
        FrameHandle(1)
    }
    fn sensor() -> FrameHandle {
        FrameHandle(3)
    }

    fn noise(var: f64) -> Option<TangentNoise> {
        Some(TangentNoise::from_variances(DVector::from_element(3, var)).unwrap())
    }

    /// A state that anchors two frames' conventions: `Odom(agent) → ENU` via the
    /// position block, and `Body(agent) → FLU` via the body↦odom orientation.
    /// `World` is deliberately left untracked so it can stand in for the
    /// unanchorable case. Neither block observes `SpecificForce`, so any
    /// measurement of it exercises the derived (no-twin) branch.
    fn anchored_state() -> StateSchema {
        StateSchema::compose(vec![
            StateSchemaBlock::new(
                Quantity::Position(FrameId::Odom(agent())),
                Convention::Enu,
                noise(0.1),
                DVector::zeros(3),
                DMatrix::identity(3, 3),
            ),
            StateSchemaBlock::orientation(
                FrameId::Body(agent()),
                FrameId::Odom(agent()),
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

    // ── frame_conventions: the map the derived branch reads ───────────────────

    #[test]
    fn frame_conventions_splits_an_orientation_into_both_endpoints() {
        // The position block contributes one entry; the orientation `Pair`
        // contributes two, its `from`/`to` conventions landing on the matching
        // frames. Odom appears in both blocks with the same convention, so the
        // map holds exactly the two distinct frames.
        let by_frame = frame_conventions(&anchored_state());

        assert_eq!(by_frame.len(), 2);
        assert_eq!(by_frame[&FrameId::Odom(agent())], Convention::Enu);
        assert_eq!(by_frame[&FrameId::Body(agent())], Convention::Flu);
    }

    // ── Direct readout: measurement quantity is tracked by the state ──────────

    #[test]
    fn direct_readout_with_matching_convention_agrees() {
        let state = anchored_state();
        let m = one(Quantity::Position(FrameId::Odom(agent())), Convention::Enu);
        assert!(check_measurement_state_agreement(&state, &m).is_ok());
    }

    #[test]
    fn direct_readout_with_wrong_convention_is_a_convention_mismatch() {
        // Same quantity the state tracks (position in odom), but declared FLU
        // where the state holds ENU: the innovation would mix axis conventions.
        let state = anchored_state();
        let m = one(Quantity::Position(FrameId::Odom(agent())), Convention::Flu);

        let err = check_measurement_state_agreement(&state, &m).unwrap_err();
        assert!(matches!(
            err,
            MeasurementAgreementError::ConventionMismatch { .. }
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
            Quantity::SpecificForce(FrameId::Body(agent())),
            Convention::Flu,
        );
        assert!(check_measurement_state_agreement(&state, &m).is_ok());
    }

    #[test]
    fn derived_measurement_in_a_tracked_frame_with_wrong_convention_is_a_frame_endpoint_mismatch() {
        // Body(agent) is anchored FLU by the state; the measurement claims ENU.
        let state = anchored_state();
        let m = one(
            Quantity::SpecificForce(FrameId::Body(agent())),
            Convention::Enu,
        );

        match check_measurement_state_agreement(&state, &m).unwrap_err() {
            MeasurementAgreementError::FrameEndpointMismatch {
                frame,
                state_frame_convention,
                measurement_convention,
                ..
            } => {
                assert_eq!(frame, FrameId::Body(agent()));
                assert_eq!(state_frame_convention, Convention::Flu);
                assert_eq!(measurement_convention, Convention::Enu);
            }
            other => panic!("expected FrameEndpointMismatch, got {other:?}"),
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
            let m = one(
                Quantity::SpecificForce(FrameId::Sensor(sensor())),
                convention,
            );
            assert!(check_measurement_state_agreement(&state, &m).is_ok());
        }
    }

    #[test]
    fn derived_measurement_in_an_untracked_non_sensor_frame_is_unanchorable() {
        // World is neither tracked by this state nor a sensor frame, so there is
        // no convention to check against.
        let state = anchored_state();
        let m = one(Quantity::SpecificForce(FrameId::World), Convention::Enu);

        match check_measurement_state_agreement(&state, &m).unwrap_err() {
            MeasurementAgreementError::UnanchorableFrame { frame, .. } => {
                assert_eq!(frame, FrameId::World);
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
                Quantity::Position(FrameId::Odom(agent())),
                Convention::Enu,
            ),
            MeasurementSchemaBlock::new(
                Quantity::SpecificForce(FrameId::Sensor(sensor())),
                Convention::Flu,
            ),
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
                Quantity::Position(FrameId::Odom(agent())),
                Convention::Flu,
            ),
            MeasurementSchemaBlock::new(
                Quantity::SpecificForce(FrameId::Sensor(sensor())),
                Convention::Flu,
            ),
        ]);

        assert!(matches!(
            check_measurement_state_agreement(&state, &m).unwrap_err(),
            MeasurementAgreementError::ConventionMismatch { .. }
        ));
    }

    // ── conventions_agree: the direct-branch predicate, in isolation ──────────
    //
    // Reached through the public check only for flat (`Single`) blocks today, so
    // its `Pair` and mixed-arity arms are pinned here directly — an AHRS
    // orientation measurement will exercise them through the front door later.

    #[test]
    fn conventions_agree_on_equal_singles_and_disagrees_otherwise() {
        assert!(conventions_agree(
            &BlockConvention::Single(Convention::Enu),
            &BlockConvention::Single(Convention::Enu),
        ));
        assert!(!conventions_agree(
            &BlockConvention::Single(Convention::Enu),
            &BlockConvention::Single(Convention::Flu),
        ));
    }

    #[test]
    fn conventions_agree_compares_both_pair_endpoints() {
        let matching = BlockConvention::Pair {
            from: Convention::Flu,
            to: Convention::Enu,
        };
        assert!(conventions_agree(&matching, &matching));

        // A single endpoint differing is enough to disagree.
        let one_endpoint_off = BlockConvention::Pair {
            from: Convention::Flu,
            to: Convention::Flu,
        };
        assert!(!conventions_agree(&matching, &one_endpoint_off));
    }

    #[test]
    fn conventions_of_different_arity_never_agree() {
        assert!(!conventions_agree(
            &BlockConvention::Single(Convention::Flu),
            &BlockConvention::Pair {
                from: Convention::Flu,
                to: Convention::Enu,
            },
        ));
    }
}
