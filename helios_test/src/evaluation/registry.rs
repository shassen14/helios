//! [`EvaluatorRegistry`]: the evaluator kinds a run file's `kind` can name.

use crate::evaluation::{
    kind::EvaluatorKind,
    kinds::{pose_error::PoseError, twist_tracking::TwistTracking},
    typed::{TwoInputsKind, Typed},
};

use std::{collections::BTreeMap, sync::Arc};

/// _Pre-step._ The evaluator kinds a run can use, by name. Open: a kind is
/// code someone adds, registered beside the standard ones.
#[derive(Clone, Default)]
pub struct EvaluatorRegistry {
    kinds: BTreeMap<&'static str, Arc<dyn EvaluatorKind>>,
}

impl EvaluatorRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// The kind named `name`, if one is registered.
    pub fn get(&self, name: &str) -> Option<&dyn EvaluatorKind> {
        self.kinds.get(name).map(AsRef::as_ref)
    }

    /// Every registered kind's name, in order, for a message listing them.
    pub fn names(&self) -> Vec<&'static str> {
        self.kinds.keys().copied().collect()
    }

    /// Registers `kind` under its name. A second kind of one name replaces
    /// the first, so a test can install its own over the standard set.
    pub fn register(&mut self, kind: impl EvaluatorKind + 'static) {
        self.kinds.insert(kind.name(), Arc::new(kind));
    }
}

/// The kinds every run can name.
pub fn standard_evaluators() -> EvaluatorRegistry {
    let mut registry = EvaluatorRegistry::new();
    registry.register(Typed(PoseError));
    registry.register(TwoInputsKind::<TwistTracking>::new());
    registry
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_standard_set_has_pose_error_and_twist_tracking() {
        let registry = standard_evaluators();

        assert!(registry.get("pose_error").is_some());
        assert!(registry.get("twist_tracking").is_some());
        assert!(registry.get("goal_distance").is_none());
        assert_eq!(registry.names(), vec!["pose_error", "twist_tracking"]);
    }
}
