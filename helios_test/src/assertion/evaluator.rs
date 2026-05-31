use crate::assertion::{
    condition::Condition,
    extract::ExtractorTable,
    target::{AgentId, TargetRegistry},
    Assertion, AssertionResult, AssertionValue, FailureKind,
};

use helios_runtime::port::PortBus;

/// Judge one assertion against the current bus state, producing a single
/// `AssertionResult`. Pure and read-only — it observes the bus, never writes
/// it — which keeps the runner's "observe, don't inject" invariant intact and
/// makes the evaluator unit-testable against a hand-built bus.
///
/// Four stages, each with its own failure so the report can say *why* rather
/// than a bare "failed": resolve the target to a channel, read that channel
/// without naming its Rust type, extract the payload into a comparable
/// `AssertionValue`, then compare.
pub fn evaluate(
    assertion: &Assertion,
    agent: &AgentId,
    registry: &TargetRegistry,
    bus: &PortBus,
    extractors: &ExtractorTable,
) -> AssertionResult {
    // Stage 1 — resolve the TOML target string to a bus channel. No mapping
    // means the author named a channel no node in the graph declares.
    let Some(key) = registry.resolve(agent, assertion.target()) else {
        return AssertionResult::Failed {
            kind: FailureKind::UnresolvedTarget,
            actual: None,
        };
    };

    // Stage 2 — read the channel without naming its Rust type (we hold only a
    // TypeId, never `T`). An empty slot is `Pending`, not a failure: the
    // channel simply hasn't published yet, which differs from holding a value
    // that fails the check.
    let Some(erased) = bus.read_erased(key) else {
        return AssertionResult::Pending;
    };

    // Stage 3 — project the erased payload into the comparable `AssertionValue`
    // space. `payload_type_id()` keys the lookup on the payload `T`, not the
    // `Stamped<T>` envelope that `Any::type_id()` would report. A present value
    // we have no extractor for is a real failure, distinct from stage 2's "no
    // value yet".
    let Some(actual) = extractors.extract(erased.payload_type_id(), erased.payload()) else {
        return AssertionResult::Failed {
            kind: FailureKind::NoExtractor,
            actual: None,
        };
    };

    // Stage 4 — compare. `check` decides pass vs which failure kind; we attach
    // the observed value here so the report can show actual-vs-expected.
    // `Ok(false)` is "compared and didn't hold"; `Err(kind)` is "couldn't
    // compare at all" (bad bounds or incompatible types).
    match check(assertion, &actual) {
        Ok(true) => AssertionResult::Passed,
        Ok(false) => AssertionResult::Failed {
            kind: FailureKind::ConditionFailed,
            actual: Some(actual),
        },
        Err(kind) => AssertionResult::Failed {
            kind,
            actual: Some(actual),
        },
    }
}

/// Run the assertion's condition against the observed value. `Ok(held)` means
/// the comparison ran (held or not); `Err(kind)` means it couldn't run.
///
/// `WithinRange` is dispatched separately because its bounds live on the
/// assertion struct, not in the value pair that `apply` compares — and a range
/// missing a bound is `InvalidBounds`, a failure kind `ConditionError` can't
/// express. Both `ConditionError` variants fold to `TypeMismatch` for now;
/// split if a report ever needs to tell a type mismatch from a not-comparable.
fn check(assertion: &Assertion, actual: &AssertionValue) -> Result<bool, FailureKind> {
    if *assertion.condition() == Condition::WithinRange {
        if let Some((low, high)) = assertion.bounds() {
            Condition::apply_range(actual, low, high).map_err(|_| FailureKind::TypeMismatch)
        } else {
            Err(FailureKind::InvalidBounds)
        }
    } else {
        assertion
            .condition()
            .apply(actual, assertion.expected())
            .map_err(|_| FailureKind::TypeMismatch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::extract::standard_extractors;
    use crate::assertion::terminal::TerminalAssertion;
    use crate::assertion::AssertionTarget;

    use helios_core::data::primitives::MonotonicTime;
    use helios_runtime::port::InternalChannel;
    use helios_runtime::prelude::{ChannelKey, Health, PortDescriptor, Stamped};

    const TARGET: &str = "agent.car.x";

    // Build a registry + bus that share one `InternalChannel<T>` slot, with the
    // registry resolving `TARGET` for agent "car" to that channel. The slot
    // starts empty; tests call `publish` to fill it (or skip it to exercise the
    // `Pending` path).
    fn fixture<T: 'static>() -> (AgentId, TargetRegistry, PortBus, ChannelKey) {
        let channel: ChannelKey = InternalChannel::of::<T>().into();
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![channel.clone()],
            rate: None,
        };
        let bus = PortBus::new(&[descriptor]);

        let agent = AgentId::new("car");
        let mut registry = TargetRegistry::new();
        registry.register(agent.clone(), AssertionTarget::new(TARGET), channel.clone());

        (agent, registry, bus, channel)
    }

    fn publish<T: 'static + Send + Sync>(bus: &PortBus, channel: &ChannelKey, value: T) {
        bus.write(
            channel.clone(),
            Stamped {
                value,
                timestamp: MonotonicTime(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .unwrap();
    }

    fn terminal(condition: Condition, value: AssertionValue) -> Assertion {
        Assertion::Terminal(TerminalAssertion {
            target: AssertionTarget::new(TARGET),
            condition,
            value,
            low: None,
            high: None,
        })
    }

    fn terminal_range(low: Option<f64>, high: Option<f64>) -> Assertion {
        Assertion::Terminal(TerminalAssertion {
            target: AssertionTarget::new(TARGET),
            condition: Condition::WithinRange,
            value: AssertionValue::Float(0.0),
            low,
            high,
        })
    }

    #[test]
    fn passes_when_condition_holds() {
        let (agent, reg, bus, ch) = fixture::<f64>();
        publish(&bus, &ch, 5.0_f64);
        let a = terminal(Condition::Equals, AssertionValue::Float(5.0));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Passed
        );
    }

    #[test]
    fn condition_failed_carries_observed_value() {
        let (agent, reg, bus, ch) = fixture::<f64>();
        publish(&bus, &ch, 5.0_f64);
        let a = terminal(Condition::Equals, AssertionValue::Float(9.0));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(5.0)),
            }
        );
    }

    #[test]
    fn pending_when_slot_empty() {
        // Slot exists but nothing has been published — not a failure.
        let (agent, reg, bus, _ch) = fixture::<f64>();
        let a = terminal(Condition::Equals, AssertionValue::Float(5.0));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Pending
        );
    }

    #[test]
    fn unresolved_target_when_not_registered() {
        // Bus has the value, but no registry entry maps the target → channel.
        let (agent, _reg, bus, _ch) = fixture::<f64>();
        let empty = TargetRegistry::new();
        let a = terminal(Condition::Equals, AssertionValue::Float(5.0));
        assert_eq!(
            evaluate(&a, &agent, &empty, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::UnresolvedTarget,
                actual: None,
            }
        );
    }

    #[test]
    fn no_extractor_for_unregistered_payload_type() {
        // `String` has no entry in `standard_extractors`, so a published String
        // value is present-but-uncomparable: NoExtractor, not Pending.
        let (agent, reg, bus, ch) = fixture::<String>();
        publish(&bus, &ch, "hello".to_string());
        let a = terminal(Condition::Equals, AssertionValue::String("hello".into()));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::NoExtractor,
                actual: None,
            }
        );
    }

    #[test]
    fn within_range_passes_inside_bounds() {
        let (agent, reg, bus, ch) = fixture::<f64>();
        publish(&bus, &ch, 2.0_f64);
        let a = terminal_range(Some(1.0), Some(3.0));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Passed
        );
    }

    #[test]
    fn within_range_fails_outside_bounds() {
        let (agent, reg, bus, ch) = fixture::<f64>();
        publish(&bus, &ch, 5.0_f64);
        let a = terminal_range(Some(1.0), Some(3.0));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::ConditionFailed,
                actual: Some(AssertionValue::Float(5.0)),
            }
        );
    }

    #[test]
    fn invalid_bounds_when_range_missing_a_bound() {
        let (agent, reg, bus, ch) = fixture::<f64>();
        publish(&bus, &ch, 2.0_f64);
        // `within_range` with only one bound set → `bounds()` returns None.
        let a = terminal_range(Some(1.0), None);
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::InvalidBounds,
                actual: Some(AssertionValue::Float(2.0)),
            }
        );
    }

    #[test]
    fn type_mismatch_when_condition_cannot_compare() {
        // `less_than` on a bool is not an ordering comparison; the resulting
        // ConditionError folds to TypeMismatch.
        let (agent, reg, bus, ch) = fixture::<bool>();
        publish(&bus, &ch, true);
        let a = terminal(Condition::LessThan, AssertionValue::Bool(false));
        assert_eq!(
            evaluate(&a, &agent, &reg, &bus, &standard_extractors()),
            AssertionResult::Failed {
                kind: FailureKind::TypeMismatch,
                actual: Some(AssertionValue::Bool(true)),
            }
        );
    }
}
