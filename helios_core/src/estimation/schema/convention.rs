use crate::frames::transforms::Convention;

/// The axis convention(s) a schema block's components are expressed in.
///
/// A flat block (position, velocity, a bias) lives in one convention; an
/// orientation is a rotation *between* two frames and so carries one convention
/// per endpoint. The variant is fixed by the block's [`Quantity`]: every kind is
/// [`Single`](Self::Single) except [`Orientation`](Quantity::Orientation), which
/// is [`Pair`](Self::Pair). The `Pair`'s `from` / `to` align with the quantity's
/// `from` / `to` frames, so a checked extractor reading `orientation::<A, B>`
/// verifies `A` against `from` and `B` against `to`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BlockConvention {
    Single(Convention),
    Pair { from: Convention, to: Convention },
}
