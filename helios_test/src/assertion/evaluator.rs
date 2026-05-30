use crate::assertion::{
    extract::ExtractorTable,
    target::{AgentId, TargetRegistry},
    Assertion, AssertionResult,
};

use helios_runtime::port::PortBus;

pub fn evaluate(
    assertion: &Assertion,
    agent: &AgentId,
    registry: &TargetRegistry,
    bus: &PortBus,
    extractors: &ExtractorTable,
) -> AssertionResult {
    let _ = (assertion, agent, registry, bus, extractors);
    todo!()
}
