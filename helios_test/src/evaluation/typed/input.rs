//! [`Base`] and [`Input`]: the roles of a typed kind, and [`Inputs`], a tuple
//! of one to three of them, the base first, each role's values kept in a
//! [`RoleBuffer`].

use super::buffer::{Lined, LinedUp, Lining, RoleBuffer};
use crate::evaluation::kind::{InputRejection, Role};

use helios_core::prelude::MonotonicTime;

use std::any::Any;

/// A value for a role the evaluation doesn't have.
const NO_SUCH_ROLE: InputRejection =
    InputRejection("a value for a role the evaluation doesn't have");

/// _Pre-step._ The base role of evaluator `E`: its name, and what it keeps of
/// a payload `P` as a value `K`. Each of its values is one sample, so it
/// comes first in a kind's inputs, and a kind has exactly one.
pub struct Base<E, P, K> {
    name: &'static str,
    project: Project<E, P, K>,
}

impl<E, P: 'static, K> Base<E, P, K> {
    /// The base `name`, keeping what `project` reads off each payload.
    pub fn new(name: &'static str, project: Project<E, P, K>) -> Self {
        Self { name, project }
    }

    fn role(&self) -> Role {
        Role::of::<P>(self.name)
    }
}

/// _Pre-step._ A role of evaluator `E` other than the base: its name, what it
/// keeps of a payload `P` as a value `K`, and how it lines up with the base.
/// Its constructor fixes how it lines up, and an interpolated role can't be
/// declared without its blend.
pub struct Input<E, P, K> {
    name: &'static str,
    project: Project<E, P, K>,
    lining: Lining<K>,
}

impl<E, P: 'static, K> Input<E, P, K> {
    /// A role whose value holds until the next: a goal, a command.
    pub fn held(name: &'static str, project: Project<E, P, K>) -> Self {
        Self {
            name,
            project,
            lining: Lining::Hold,
        }
    }

    /// A role interpolated to each base stamp by `blend`: the kept value
    /// `fraction` of the way from the first to the second, `None` when the
    /// two can't be blended.
    pub fn interpolated(
        name: &'static str,
        project: Project<E, P, K>,
        blend: fn(&K, &K, f64) -> Option<K>,
    ) -> Self {
        Self {
            name,
            project,
            lining: Lining::Interpolate(blend),
        }
    }

    fn role(&self) -> Role {
        Role::of::<P>(self.name)
    }
}

/// What a role keeps of one payload, as its evaluator reads it, or why the
/// payload is refused.
type Project<E, P, K> = fn(&E, &P) -> Result<K, InputRejection>;

/// _Pre-step._ The roles of evaluator `E`: a tuple of a [`Base`] and up to two
/// [`Input`]s, which keeps each role's values typed and lines them up into
/// one sample's [`Row`](Self::Row). Sealed: only the tuples of one to three
/// roles implement it.
pub trait Inputs<E>: sealed::Sealed + Send + Sync + 'static {
    /// One sample's values, one reference per role, in role order.
    type Row<'a>: Copy;

    /// Every role's kept values, each with the stamp of its payload.
    type Kept: Default + Send + Sync;

    /// One sample's values as lined up: the base's and each held value
    /// borrowed, each blended one owned.
    type LinedRow<'a>;

    /// Each role, erased, in order.
    fn roles(&self) -> Vec<Role>;

    /// What `role` keeps of `payload`, stamped `at`, added to `kept`, or why
    /// it is refused.
    fn keep(
        &self,
        evaluator: &E,
        kept: &mut Self::Kept,
        role: usize,
        at: MonotonicTime,
        payload: &dyn Any,
    ) -> Result<(), InputRejection>;

    /// Every value in `kept` stamped at or before `until`, each role taken
    /// in stamp order, so the order the values arrived in never matters.
    /// Each base value is one sample, each other role lined up at its stamp
    /// as its lining says; a base value some role has no value for is
    /// uncovered.
    fn line_up<'a>(
        &self,
        kept: &'a Self::Kept,
        until: MonotonicTime,
    ) -> LinedUp<Self::LinedRow<'a>>;

    /// One sample's values, from its row as lined up.
    fn row<'a>(lined: &'a Self::LinedRow<'_>) -> Self::Row<'a>;
}

/// The supertrait that seals [`Inputs`]: public, so it may bound a public
/// trait, but in a private module, so no other crate can name it.
mod sealed {
    pub trait Sealed {}
}

/// [`Inputs`], and its seal, for a tuple of a [`Base`] and [`Input`]s, the base given as its
/// payload and kept types, each other role as its index, its payload and
/// kept types, and a name for its value.
macro_rules! inputs_for_tuple {
    (($base_payload:ident, $base_kept:ident) $(, ($index:tt, $payload:ident, $kept:ident, $value:ident))*) => {
        impl<
                E: 'static,
                $base_payload: 'static,
                $base_kept: Send + Sync + 'static,
                $($payload: 'static, $kept: Send + Sync + 'static),*
            > sealed::Sealed for (Base<E, $base_payload, $base_kept>, $(Input<E, $payload, $kept>,)*)
        {
        }

        impl<
                E: 'static,
                $base_payload: 'static,
                $base_kept: Send + Sync + 'static,
                $($payload: 'static, $kept: Send + Sync + 'static),*
            > Inputs<E> for (Base<E, $base_payload, $base_kept>, $(Input<E, $payload, $kept>,)*)
        {
            type Row<'a> = (&'a $base_kept, $(&'a $kept,)*);
            type Kept = (RoleBuffer<$base_kept>, $(RoleBuffer<$kept>,)*);
            type LinedRow<'a> = (&'a $base_kept, $(Lined<'a, $kept>,)*);

            fn roles(&self) -> Vec<Role> {
                vec![self.0.role() $(, self.$index.role())*]
            }

            fn keep(
                &self,
                evaluator: &E,
                kept: &mut Self::Kept,
                role: usize,
                at: MonotonicTime,
                payload: &dyn Any,
            ) -> Result<(), InputRejection> {
                match role {
                    0 => kept.0.keep(at, payload, |payload| (self.0.project)(evaluator, payload)),
                    $($index => kept.$index.keep(at, payload, |payload| {
                        (self.$index.project)(evaluator, payload)
                    }),)*
                    _ => Err(NO_SUCH_ROLE),
                }
            }

            fn line_up<'a>(
                &self,
                kept: &'a Self::Kept,
                until: MonotonicTime,
            ) -> LinedUp<Self::LinedRow<'a>> {
                let base = kept.0.up_to(until);
                let mut lined = LinedUp {
                    base_samples: base.len(),
                    uncovered: 0,
                    rows: Vec::new(),
                };
                for (at, base) in base {
                    match (Some(base), $(kept.$index.at(*at, until, &self.$index.lining),)*) {
                        (Some(base), $(Some($value),)*) => {
                            lined.rows.push((*at, (base, $($value,)*)))
                        }
                        _ => lined.uncovered += 1,
                    }
                }
                lined
            }

            fn row<'a>(lined: &'a Self::LinedRow<'_>) -> Self::Row<'a> {
                (lined.0, $(lined.$index.value(),)*)
            }
        }
    };
}

inputs_for_tuple!((P0, K0));
inputs_for_tuple!((P0, K0), (1, P1, K1, value1));
inputs_for_tuple!((P0, K0), (1, P1, K1, value1), (2, P2, K2, value2));

#[cfg(test)]
mod tests {
    use super::*;
    use crate::evaluation::typed::buffer::NOT_ITS_PAYLOAD;

    /// Keeps a finite `f64` as it is; refuses a negative one.
    fn kept(_: &(), value: &f64) -> Result<f64, InputRejection> {
        match *value {
            value if value < 0.0 => Err(InputRejection("a negative value")),
            value => Ok(value),
        }
    }

    fn blend(before: &f64, after: &f64, fraction: f64) -> Option<f64> {
        Some(before + (after - before) * fraction)
    }

    /// A base, an interpolated role and a held role reading a label.
    type ThreeRoles = (
        Base<(), f64, f64>,
        Input<(), f64, f64>,
        Input<(), String, f64>,
    );

    fn inputs() -> ThreeRoles {
        (
            Base::new("base", kept),
            Input::interpolated("smooth", kept, blend),
            Input::held("label", |_, label: &String| Ok(label.len() as f64)),
        )
    }

    fn at(t: f64) -> MonotonicTime {
        MonotonicTime(t)
    }

    #[test]
    fn each_role_is_erased_with_its_payload() {
        let roles = inputs().roles();

        assert_eq!(
            roles,
            vec![
                Role::of::<f64>("base"),
                Role::of::<f64>("smooth"),
                Role::of::<String>("label"),
            ]
        );
    }

    #[test]
    fn a_value_is_kept_as_its_roles_function_reads_it_or_refused() {
        let inputs = inputs();
        let mut kept = <ThreeRoles as Inputs<()>>::Kept::default();

        let label = inputs.keep(&(), &mut kept, 2, at(1.0), &"four".to_owned());
        let negative = inputs.keep(&(), &mut kept, 0, at(1.0), &-1.0);
        let not_a_label = inputs.keep(&(), &mut kept, 2, at(1.0), &1.0);
        let no_role = inputs.keep(&(), &mut kept, 3, at(1.0), &1.0);

        assert_eq!(label, Ok(()));
        assert_eq!(kept.2.up_to(at(1.0)), [(at(1.0), 4.0)]);
        assert_eq!(negative, Err(InputRejection("a negative value")));
        assert!(kept.0.up_to(at(1.0)).is_empty());
        assert_eq!(not_a_label, Err(NOT_ITS_PAYLOAD));
        assert_eq!(no_role, Err(NO_SUCH_ROLE));
    }

    #[test]
    fn each_base_value_is_one_row_of_the_held_value_and_the_blend_at_its_stamp() {
        let inputs = inputs();
        let mut kept = <ThreeRoles as Inputs<()>>::Kept::default();
        for (role, t, value) in [(1, 0.0, 0.0), (1, 4.0, 4.0), (0, 1.0, 10.0), (0, 5.0, 50.0)] {
            inputs
                .keep(&(), &mut kept, role, at(t), &value)
                .expect("kept");
        }
        inputs
            .keep(&(), &mut kept, 2, at(0.5), &"four".to_owned())
            .expect("kept");

        let lined = inputs.line_up(&kept, at(10.0));
        let rows: Vec<(f64, (&f64, &f64, &f64))> = lined
            .rows
            .iter()
            .map(|(at, row)| (at.0, <ThreeRoles as Inputs<()>>::row(row)))
            .collect();

        assert_eq!(rows, vec![(1.0, (&10.0, &1.0, &4.0))]);
        assert_eq!((lined.base_samples, lined.uncovered), (2, 1));
    }
}
