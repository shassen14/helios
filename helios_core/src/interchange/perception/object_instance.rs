//! [`ObjectInstanceId`]: which world object, as ground truth names it.

use std::fmt::{self, Display};
use std::sync::Arc;

/// What joins a layout's name to a placement's in an [`ObjectInstanceId`].
pub const INSTANCE_ID_SEPARATOR: char = '/';

/// A world object's identity in ground truth, `<layout>/<placement>` (e.g.
/// `yard/crate_a`): stable across runs and list order, and the key datasets
/// are joined on. The layout and placement names are checked where the
/// layout is resolved; this type only joins them. Backed by `Arc<str>`, so
/// a copy per tick is cheap.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObjectInstanceId(Arc<str>);

impl ObjectInstanceId {
    /// The object `placement` of the layout `layout`.
    pub fn new(layout: &str, placement: &str) -> Self {
        Self(format!("{layout}{INSTANCE_ID_SEPARATOR}{placement}").into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Display for ObjectInstanceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_instance_is_its_layout_and_placement_joined() {
        let crate_a = ObjectInstanceId::new("yard", "crate_a");

        assert_eq!(crate_a.as_str(), "yard/crate_a");
        assert_eq!(crate_a.to_string(), "yard/crate_a");
        assert_ne!(crate_a, ObjectInstanceId::new("yard", "crate_b"));
    }
}
