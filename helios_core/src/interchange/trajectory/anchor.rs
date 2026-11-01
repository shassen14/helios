//! Where a spatial geometry key is: its [`Anchor`], and the [`Root`] the
//! anchor is measured from.

use crate::spatial::id::FrameName;

/// The point a spatial geometry key describes: a frame leaf (`base_link`,
/// `tool0`) or a named point (`com`, `zmp`). Both are spelled as frame
/// leaves, and nothing here tells them apart.
///
/// Its own type, not a bare [`FrameName`], because the two promise different
/// things: an anchor is held by the sample, a named frame is bound by the
/// producer's config. One can't be passed where the other is meant.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Anchor(FrameName);

impl Anchor {
    pub fn new(name: FrameName) -> Self {
        Self(name)
    }

    pub fn name(&self) -> &FrameName {
        &self.0
    }
}

/// What an anchor is measured from.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Root {
    /// The one frame all geometry is finally rooted at, named once for the
    /// whole trajectory.
    Reference,
    /// Another anchor of the same sample (a tool rooted at the base it is
    /// mounted on). That anchor must have an orientation.
    Anchor(Anchor),
}
