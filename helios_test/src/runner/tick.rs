use crate::assertion::target::AgentId;
use crate::assertion::{Assertion, AssertionResult};
use crate::run::termination::TerminationReason;

use super::{termination_check, Runner};

use helios_core::data::MonotonicTime;
use helios_runtime::port::PortBus;

impl Runner {
    /// Advance the run by one observation at simulated time `t`, judging every
    /// assertion against the agents' buses and reporting whether to stop. Pure
    /// w.r.t. the buses — it reads them, never writes — so the host stays the
    /// sole source of simulation state.
    pub fn tick(&mut self, t: MonotonicTime, blackboards: &[(AgentId, &PortBus)]) -> TickAction {
        // The run's clock origin: stamped on the first tick and fixed thereafter,
        // so the time budget is measured from when the run actually began, not
        // from zero. Copied out so the borrow of `start_time` ends here.
        let start = *self.start_time.get_or_insert(t);

        // Pass 1 — judge every assertion this tick into an owned vector. Terminal
        // assertions are evaluated too, because the stop-decision reads them for
        // `on_assertion` triggers; only their latch is left untouched (pass 2).
        let mut results: Vec<AssertionResult> = Vec::with_capacity(self.run.assertions().len());
        for assertion in self.run.assertions() {
            let result = self.result_for(assertion, &blackboards);

            results.push(result);
        }

        // Pass 2 — fold the continuous verdicts into their latches. Terminals are
        // skipped: they're judged once at finalize, never latched per tick.
        // `self.run` and `self.states` are disjoint fields, so the immutable loop
        // borrow and the mutable `record` borrow coexist.
        for (i, assertion) in self.run.assertions().iter().enumerate() {
            if !matches!(assertion, Assertion::Continuous(_)) {
                continue;
            }
            match &results[i] {
                AssertionResult::Passed { actual } => {
                    self.states[i].record(Some(actual.clone()), Ok(()), t);
                }
                AssertionResult::Failed { kind, actual } => {
                    self.states[i].record(actual.clone(), Err(kind.to_string()), t);
                }
                // No observation yet — leave the latch alone.
                AssertionResult::Pending => {}
            }
        }

        // Pass 3 — decide. `check` applies the precedence (assertion outcome
        // before time budget) and returns the reason, if any.
        match termination_check::check(t, start, self.run.termination(), &results) {
            Some(reason) => TickAction::Terminate { reason },
            None => TickAction::Continue,
        }
    }
}

/// _Per-step._ The outcome of one `tick`: keep going, or stop with a reason.
#[derive(Debug, Clone, PartialEq)]
pub enum TickAction {
    Continue,
    Terminate { reason: TerminationReason },
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::target::{AssertionTarget, TargetRegistry};
    use crate::assertion::{AssertionValue, FailureKind};
    use crate::run::Run;
    use crate::runner::state::ContinuousStatus;

    use helios_runtime::port::InternalChannel;
    use helios_runtime::prelude::{ChannelKey, Health, PortDescriptor, Stamped};

    fn at(secs: f64) -> MonotonicTime {
        MonotonicTime(secs)
    }

    // Build a `Run` from a termination body and zero or more assertion blocks.
    // Fields are private, so we construct through TOML like the rest of the
    // suite. `termination` is an inline-table body (comma-separated keys).
    fn run(termination: &str, assertions: &[&str]) -> Run {
        let mut text = format!(
            "schema_version = \"1.0\"\n\
             scenario = {{ from = \"s\" }}\n\
             observation = {{ from = \"o\" }}\n\
             termination = {{ {termination} }}\n"
        );
        for block in assertions {
            text.push_str("\n[[assert]]\n");
            text.push_str(block);
            text.push('\n');
        }
        toml::from_str(&text).expect("valid run toml")
    }

    // Build a `Runner` through the real constructor, then swap in a pre-seeded
    // registry. In production `on_pipeline_built` populates the registry by
    // walking the pipeline; these tests skip that step, so they seed a registry
    // directly and inject it. `new` supplies everything else — a validated run,
    // index-aligned states, and the standard extractors so f64 channels compare.
    fn runner(run: Run, registry: TargetRegistry) -> Runner {
        let mut r = Runner::new(run).expect("valid run");
        r.registry = registry;
        r
    }

    // A single-output bus over one unnamed f64 channel, plus the key to write it.
    fn bus() -> (PortBus, ChannelKey) {
        let channel: ChannelKey = InternalChannel::of::<f64>().into();
        let descriptor = PortDescriptor {
            required_inputs: vec![],
            optional_inputs: vec![],
            outputs: vec![channel.clone()],
            rate: None,
        };
        (PortBus::new(&[descriptor]), channel)
    }

    fn publish(bus: &PortBus, channel: &ChannelKey, value: f64) {
        bus.write(
            channel.clone(),
            Stamped {
                value,
                timestamp: at(0.0),
                health: Health::Ok,
                producer: 0,
            },
        )
        .unwrap();
    }

    fn seed(registry: &mut TargetRegistry, agent: &AgentId, target: &str, channel: &ChannelKey) {
        registry.register(agent.clone(), AssertionTarget::new(target), channel.clone());
    }

    // `{value:?}` forces a decimal point so TOML reads it as a float, not an int.
    fn cont(target: &str, value: f64) -> String {
        format!(
            "when = \"continuous\"\n\
             target = \"{target}\"\n\
             condition = \"equals\"\n\
             value = {value:?}"
        )
    }

    fn term(target: &str, value: f64) -> String {
        format!(
            "when = \"at_completion\"\n\
             target = \"{target}\"\n\
             condition = \"equals\"\n\
             value = {value:?}"
        )
    }

    #[test]
    fn continuous_pass_holds_and_continues() {
        // Value matches: no trigger set and time budget unmet, so the run goes on
        // and the latch advances Pending -> Holding.
        let agent = AgentId::new("car");
        let (b, ch) = bus();
        publish(&b, &ch, 5.0);
        let mut reg = TargetRegistry::new();
        seed(&mut reg, &agent, "agent.car.x", &ch);

        let a = cont("agent.car.x", 5.0);
        let mut r = runner(run("max_simulated_seconds = 10.0", &[&a]), reg);

        assert_eq!(r.tick(at(0.0), &[(agent, &b)]), TickAction::Continue);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Holding);
    }

    #[test]
    fn continuous_failure_terminates_and_latches() {
        // Value violates the condition; `any_failed` stops the run, and the
        // returned reason carries the real failing result (kind + observed
        // value). The latch records when and why it first broke — the reason
        // string comes from `FailureKind`'s Display.
        let agent = AgentId::new("car");
        let (b, ch) = bus();
        publish(&b, &ch, 9.0);
        let mut reg = TargetRegistry::new();
        seed(&mut reg, &agent, "agent.car.x", &ch);

        let a = cont("agent.car.x", 5.0);
        let mut r = runner(run("on_assertion = \"any_failed\"", &[&a]), reg);

        assert_eq!(
            r.tick(at(2.0), &[(agent, &b)]),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion {
                    result: AssertionResult::Failed {
                        kind: FailureKind::ConditionFailed,
                        actual: Some(AssertionValue::Float(9.0)),
                    },
                },
            }
        );
        assert_eq!(
            r.states[0].status(),
            &ContinuousStatus::FailedAt {
                t: at(2.0),
                reason: "condition failed".into(),
            }
        );
    }

    #[test]
    fn terminal_pass_terminates_without_latching() {
        // A terminal assertion drives the stop-decision (`any_passed` fires on
        // its pass) but is never folded into the latch — terminals are judged at
        // finalize, so the status stays Pending.
        let agent = AgentId::new("car");
        let (b, ch) = bus();
        publish(&b, &ch, 5.0);
        let mut reg = TargetRegistry::new();
        seed(&mut reg, &agent, "agent.car.x", &ch);

        let a = term("agent.car.x", 5.0);
        let mut r = runner(run("on_assertion = \"any_passed\"", &[&a]), reg);

        assert_eq!(
            r.tick(at(1.0), &[(agent, &b)]),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion {
                    result: AssertionResult::Passed {
                        actual: AssertionValue::Float(5.0),
                    },
                },
            }
        );
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn pending_observation_continues_without_latching() {
        // The channel exists but nothing has published — `Pending`, not a
        // failure. Even with `any_failed` armed, the run continues and the latch
        // is left alone.
        let agent = AgentId::new("car");
        let (b, ch) = bus();
        let mut reg = TargetRegistry::new();
        seed(&mut reg, &agent, "agent.car.x", &ch);

        let a = cont("agent.car.x", 5.0);
        let mut r = runner(
            run(
                "max_simulated_seconds = 10.0, on_assertion = \"any_failed\"",
                &[&a],
            ),
            reg,
        );

        assert_eq!(r.tick(at(0.0), &[(agent, &b)]), TickAction::Continue);
        assert_eq!(r.states[0].status(), &ContinuousStatus::Pending);
    }

    #[test]
    fn time_budget_measured_from_first_tick() {
        // A pure smoke run (no assertions). The clock origin is stamped on the
        // first tick at t=10, so elapsed is measured from there, not zero: the
        // budget fires at t=15, not immediately.
        let mut r = runner(
            run("max_simulated_seconds = 5.0", &[]),
            TargetRegistry::new(),
        );

        assert_eq!(r.tick(at(10.0), &[]), TickAction::Continue);
        assert_eq!(r.tick(at(14.0), &[]), TickAction::Continue);
        assert_eq!(
            r.tick(at(15.0), &[]),
            TickAction::Terminate {
                reason: TerminationReason::MaxSimulatedSeconds,
            }
        );
    }

    #[test]
    fn missing_agent_degrades_to_unresolved_failure() {
        // The target is registered, but its agent's bus isn't in this tick's
        // blackboards — so `find` misses. Rather than panic, the assertion
        // degrades to an `UnresolvedTarget` failure, which `any_failed` then
        // stops on, and which latches with the matching reason.
        let agent = AgentId::new("car");
        let (_b, ch) = bus();
        let mut reg = TargetRegistry::new();
        seed(&mut reg, &agent, "agent.car.x", &ch);

        let a = cont("agent.car.x", 5.0);
        let mut r = runner(run("on_assertion = \"any_failed\"", &[&a]), reg);

        assert_eq!(
            r.tick(at(3.0), &[]),
            TickAction::Terminate {
                reason: TerminationReason::OnAssertion {
                    result: AssertionResult::Failed {
                        kind: FailureKind::UnresolvedTarget,
                        actual: None,
                    },
                },
            }
        );
        assert_eq!(
            r.states[0].status(),
            &ContinuousStatus::FailedAt {
                t: at(3.0),
                reason: "unresolved target".into(),
            }
        );
        // No value was observed, so the cache stays empty.
        assert_eq!(r.states[0].last_value(), &None);
    }

    #[test]
    fn states_track_each_continuous_assertion_by_index() {
        // Two agents, one bus each; `find` must pair each assertion with its own
        // agent's bus. No trigger and the budget unmet, so the run continues and
        // both latches update independently: index 0 holds, index 1 fails.
        let car_a = AgentId::new("car_a");
        let car_b = AgentId::new("car_b");
        let (bus_a, ch_a) = bus();
        let (bus_b, ch_b) = bus();
        publish(&bus_a, &ch_a, 5.0);
        publish(&bus_b, &ch_b, 1.0);

        let mut reg = TargetRegistry::new();
        seed(&mut reg, &car_a, "agent.car_a.x", &ch_a);
        seed(&mut reg, &car_b, "agent.car_b.x", &ch_b);

        let a = cont("agent.car_a.x", 5.0);
        let b = cont("agent.car_b.x", 5.0);
        let mut r = runner(run("max_simulated_seconds = 100.0", &[&a, &b]), reg);

        assert_eq!(
            r.tick(at(1.0), &[(car_a, &bus_a), (car_b, &bus_b)]),
            TickAction::Continue
        );
        assert_eq!(r.states[0].status(), &ContinuousStatus::Holding);
        assert_eq!(
            r.states[1].status(),
            &ContinuousStatus::FailedAt {
                t: at(1.0),
                reason: "condition failed".into(),
            }
        );
    }
}
