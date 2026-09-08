//! Attribute columns: the per-point payload a point cloud carries alongside its
//! geometry, expressed struct-of-arrays.
//!
//! Two traits split the lifecycle. [`Attributes`] is the frozen, shareable
//! columns a finished cloud holds; [`AttributeColumns`] is the growable builder
//! that accumulates them one row at a time and freezes them at the end. A
//! concrete attribute set — [`LidarColumns`] here — is one real sensor's output
//! contract: a named bundle with a fixed set of columns, not a bag of optional
//! fields. New sensors add a new bundle (a small set, bounded by supported
//! hardware); columns no single device emits are reached by composing bundles,
//! never by bolting `Option` columns onto an existing one. `()` is the
//! degenerate "bare geometry, no attributes" case.

use std::sync::Arc;

/// The frozen attribute columns a finished cloud carries.
///
/// The associated types tie a bundle to its builder: `Row` is the per-point
/// value pushed while building, and `Builder` is the [`AttributeColumns`] that
/// accumulates rows and produces `Self`. The bounds `Clone + Send + Sync +
/// 'static` are what let a cloud generic over `A` cross the bus and clone by
/// sharing its columns.
pub trait Attributes: Clone + Send + Sync + 'static {
    /// The per-point value handed to the builder's `push`.
    type Row;
    /// The builder that accumulates `Row`s into `Self`.
    type Builder: AttributeColumns<Row = Self::Row, Output = Self> + Default;

    /// The number of rows across the columns, or `None` when there are no
    /// columns (`()`). The builder's length gate compares this against the
    /// geometry; `None` opts out — bare geometry has no attribute column that
    /// could disagree.
    fn column_len(&self) -> Option<usize>;

    /// Keeps the rows whose mask entry is true, in row order, filtering every
    /// column by the same mask so they stay aligned. `keep` has one entry per
    /// row; the caller (via [`PointCloud::select`](super::PointCloud::select))
    /// guarantees its length.
    fn select(&self, keep: &[bool]) -> Self;
}

/// The growable builder half of an attribute bundle: accumulates one row per
/// point, then freezes into the [`Attributes`] columns.
///
/// `Default` gives the empty starting state, so a cloud builder generic over `A`
/// can spin one up without knowing the concrete type.
pub trait AttributeColumns: Default {
    /// The per-point value appended by `push` — matches `Attributes::Row`.
    type Row;
    /// The frozen columns produced by `finish` — matches the owning `Attributes`.
    type Output;

    /// Appends one point's attributes. The cloud builder calls this in lockstep
    /// with pushing the point's geometry, so the columns never drift apart.
    fn push(&mut self, row: Self::Row);

    /// The number of rows pushed so far — read by the length gate at finalize.
    fn rows(&self) -> usize;

    /// Freezes the accumulated columns into their shareable form.
    fn finish(self) -> Self::Output;
}

/// The empty attribute set: a cloud of bare geometry, no per-point payload.
///
/// `column_len` is `None` (nothing to length-check against the geometry) and
/// `select` is a no-op, so `PointCloud<F, ()>` pays nothing for the attribute
/// machinery it does not use.
impl Attributes for () {
    type Row = ();
    type Builder = ();

    fn column_len(&self) -> Option<usize> {
        None
    }

    fn select(&self, _keep: &[bool]) -> Self {}
}

impl AttributeColumns for () {
    type Row = ();
    type Output = ();

    fn push(&mut self, _row: Self::Row) {}

    fn rows(&self) -> usize {
        0
    }

    fn finish(self) -> Self::Output {}
}

/// One lidar return's attributes: the per-point row pushed while building a
/// [`LidarColumns`] bundle.
pub struct LidarAttrs {
    pub intensity: f32,
    pub ring: u16,
}

/// The attribute bundle for a spinning lidar: return `intensity` and the beam
/// `ring` index, each its own contiguous column behind an [`Arc`] so a derived
/// cloud shares them by pointer.
#[derive(Clone)]
pub struct LidarColumns {
    intensity: Arc<[f32]>,
    ring: Arc<[u16]>,
}

impl LidarColumns {
    /// The return-intensity column, one value per point.
    pub fn intensity(&self) -> &[f32] {
        &self.intensity
    }

    /// The beam-ring column, one value per point.
    pub fn ring(&self) -> &[u16] {
        &self.ring
    }
}

impl Attributes for LidarColumns {
    type Row = LidarAttrs;
    type Builder = LidarColumnsBuilder;

    fn column_len(&self) -> Option<usize> {
        Some(self.intensity.len())
    }

    fn select(&self, keep: &[bool]) -> Self {
        debug_assert_eq!(keep.len(), self.intensity.len());

        // Both columns walk the same mask in the same order, so the kept rows
        // stay aligned across them.
        let intensity: Vec<f32> = self
            .intensity
            .iter()
            .zip(keep)
            .filter_map(|(i, k)| k.then_some(*i))
            .collect();

        let ring: Vec<u16> = self
            .ring
            .iter()
            .zip(keep)
            .filter_map(|(r, k)| k.then_some(*r))
            .collect();

        LidarColumns {
            intensity: intensity.into(),
            ring: ring.into(),
        }
    }
}

/// The builder for [`LidarColumns`]: a growable `Vec` per column, frozen to
/// `Arc<[_]>` at `finish`.
#[derive(Default)]
pub struct LidarColumnsBuilder {
    intensity: Vec<f32>,
    ring: Vec<u16>,
}

impl AttributeColumns for LidarColumnsBuilder {
    type Row = LidarAttrs;
    type Output = LidarColumns;

    fn push(&mut self, row: Self::Row) {
        self.intensity.push(row.intensity);
        self.ring.push(row.ring);
    }

    fn rows(&self) -> usize {
        self.intensity.len()
    }

    fn finish(self) -> Self::Output {
        LidarColumns {
            intensity: self.intensity.into(),
            ring: self.ring.into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a `LidarColumns` from `(intensity, ring)` rows through its builder
    /// — the only write path.
    fn lidar(rows: &[(f32, u16)]) -> LidarColumns {
        let mut builder = LidarColumnsBuilder::default();
        for &(intensity, ring) in rows {
            builder.push(LidarAttrs { intensity, ring });
        }
        builder.finish()
    }

    #[test]
    fn builder_accumulates_then_freezes_each_column() {
        let columns = lidar(&[(0.1, 0), (0.2, 1), (0.3, 2)]);

        assert_eq!(columns.intensity(), &[0.1, 0.2, 0.3]);
        assert_eq!(columns.ring(), &[0, 1, 2]);
    }

    #[test]
    fn builder_rows_tracks_pushed_count() {
        let mut builder = LidarColumnsBuilder::default();
        assert_eq!(builder.rows(), 0);

        builder.push(LidarAttrs {
            intensity: 1.0,
            ring: 0,
        });
        builder.push(LidarAttrs {
            intensity: 2.0,
            ring: 1,
        });

        assert_eq!(builder.rows(), 2);
    }

    #[test]
    fn column_len_reports_the_row_count() {
        assert_eq!(lidar(&[(0.1, 0), (0.2, 1)]).column_len(), Some(2));
    }

    #[test]
    fn select_keeps_masked_rows_in_lockstep() {
        let kept = lidar(&[(0.1, 0), (0.2, 1), (0.3, 2), (0.4, 3)]).select(&[
            true, false, true, false,
        ]);

        assert_eq!(kept.intensity(), &[0.1, 0.3]);
        assert_eq!(kept.ring(), &[0, 2]);
    }

    #[test]
    fn select_all_false_empties_every_column() {
        let kept = lidar(&[(0.1, 0), (0.2, 1)]).select(&[false, false]);

        assert!(kept.intensity().is_empty());
        assert!(kept.ring().is_empty());
    }

    #[test]
    fn unit_attributes_have_no_columns_and_select_is_a_no_op() {
        assert_eq!(().column_len(), None);
        assert_eq!(().select(&[true, false]), ());
    }

    #[test]
    fn unit_builder_is_inert() {
        let mut builder = <()>::default();

        AttributeColumns::push(&mut builder, ());

        assert_eq!(AttributeColumns::rows(&builder), 0);
        assert_eq!(builder.finish(), ());
    }
}
