//! [`Input`]: one role of a typed kind, and [`Inputs`], a tuple of one to
//! three of them. The one place a payload or a kept value is downcast.

use crate::evaluation::kind::{Alignment, InputRejection, Projection, Role};

use std::any::Any;

/// A value for a role the evaluation doesn't have.
const NO_SUCH_ROLE: InputRejection =
    InputRejection("a value for a role the evaluation doesn't have");

/// A value of another payload type than its role reads. Setup binds each
/// role to a series of its type, so this is never reached.
const NOT_ITS_PAYLOAD: InputRejection =
    InputRejection("a value of another type than its role reads");

/// _Pre-step._ One role of evaluator `E`: its name, what it keeps of a
/// payload `P` as a value `K`, and how it lines up in time. Its constructor
/// fixes the alignment, and an interpolated role can't be declared without
/// its blend.
pub struct Input<E, P, K> {
    name: &'static str,
    project: fn(&E, &P) -> Result<K, InputRejection>,
    lining: Lining<K>,
}

impl<E, P: 'static, K: Send + Sync + 'static> Input<E, P, K> {
    /// The base: each of its values is one sample.
    pub fn base(name: &'static str, project: fn(&E, &P) -> Result<K, InputRejection>) -> Self {
        Self {
            name,
            project,
            lining: Lining::Base,
        }
    }

    /// A role whose value holds until the next: a goal, a command.
    pub fn held(name: &'static str, project: fn(&E, &P) -> Result<K, InputRejection>) -> Self {
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
        project: fn(&E, &P) -> Result<K, InputRejection>,
        blend: fn(&K, &K, f64) -> Option<K>,
    ) -> Self {
        Self {
            name,
            project,
            lining: Lining::Interpolate(blend),
        }
    }

    fn role(&self) -> Role {
        let alignment = match self.lining {
            Lining::Base => Alignment::Base,
            Lining::Hold => Alignment::Hold,
            Lining::Interpolate(_) => Alignment::Interpolate,
        };
        Role::of::<P>(self.name, alignment)
    }

    fn project(&self, evaluator: &E, payload: &dyn Any) -> Result<Projection, InputRejection> {
        let payload = payload.downcast_ref::<P>().ok_or(NOT_ITS_PAYLOAD)?;
        Ok(Box::new((self.project)(evaluator, payload)?))
    }

    fn blend(&self, before: &Projection, after: &Projection, fraction: f64) -> Option<Projection> {
        let Lining::Interpolate(blend) = self.lining else {
            return None;
        };
        let blended = blend(
            before.downcast_ref::<K>()?,
            after.downcast_ref::<K>()?,
            fraction,
        )?;
        Some(Box::new(blended))
    }
}

/// How a role lines up, with an interpolated role's blend.
enum Lining<K> {
    Base,
    Hold,
    Interpolate(fn(&K, &K, f64) -> Option<K>),
}

/// _Pre-step._ The roles of evaluator `E`: a tuple of one to three
/// [`Input`]s, whose kept values make one sample's [`Row`](Self::Row).
pub trait Inputs<E>: Send + Sync + 'static {
    /// One sample's kept values, one reference per role, in role order.
    type Row<'a>: Copy;

    /// Each role, erased, in order.
    fn roles(&self) -> Vec<Role>;

    /// What `role` keeps of `payload`, as `evaluator` projects it.
    fn project(
        &self,
        evaluator: &E,
        role: usize,
        payload: &dyn Any,
    ) -> Result<Projection, InputRejection>;

    /// `role`'s value `fraction` of the way from `before` to `after`, or
    /// `None` for a role that isn't interpolated or two it can't blend.
    fn blend(
        &self,
        role: usize,
        before: &Projection,
        after: &Projection,
        fraction: f64,
    ) -> Option<Projection>;

    /// One sample's values, typed, from every role's projection lined up
    /// there; `None` when one isn't a value its role kept.
    fn row<'a>(&self, aligned: &[&'a Projection]) -> Option<Self::Row<'a>>;
}

/// [`Inputs`] for a tuple of `Input`s, each given as its index, its payload
/// type and its kept type.
macro_rules! inputs_for_tuple {
    ($(($index:tt, $payload:ident, $kept:ident)),+) => {
        impl<E: 'static, $($payload: 'static, $kept: Send + Sync + 'static),+> Inputs<E>
            for ($(Input<E, $payload, $kept>,)+)
        {
            type Row<'a> = ($(&'a $kept,)+);

            fn roles(&self) -> Vec<Role> {
                vec![$(self.$index.role()),+]
            }

            fn project(
                &self,
                evaluator: &E,
                role: usize,
                payload: &dyn Any,
            ) -> Result<Projection, InputRejection> {
                match role {
                    $($index => self.$index.project(evaluator, payload),)+
                    _ => Err(NO_SUCH_ROLE),
                }
            }

            fn blend(
                &self,
                role: usize,
                before: &Projection,
                after: &Projection,
                fraction: f64,
            ) -> Option<Projection> {
                match role {
                    $($index => self.$index.blend(before, after, fraction),)+
                    _ => None,
                }
            }

            fn row<'a>(&self, aligned: &[&'a Projection]) -> Option<Self::Row<'a>> {
                Some(($(aligned.get($index)?.downcast_ref::<$kept>()?,)+))
            }
        }
    };
}

inputs_for_tuple!((0, P0, K0));
inputs_for_tuple!((0, P0, K0), (1, P1, K1));
inputs_for_tuple!((0, P0, K0), (1, P1, K1), (2, P2, K2));

#[cfg(test)]
mod tests {
    use super::*;

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
        Input<(), f64, f64>,
        Input<(), f64, f64>,
        Input<(), String, f64>,
    );

    fn inputs() -> ThreeRoles {
        (
            Input::base("base", kept),
            Input::interpolated("smooth", kept, blend),
            Input::held("label", |_, label: &String| Ok(label.len() as f64)),
        )
    }

    #[test]
    fn each_role_is_erased_with_its_alignment_and_payload() {
        let roles = inputs().roles();

        assert_eq!(
            roles,
            vec![
                Role::of::<f64>("base", Alignment::Base),
                Role::of::<f64>("smooth", Alignment::Interpolate),
                Role::of::<String>("label", Alignment::Hold),
            ]
        );
    }

    #[test]
    fn a_value_is_projected_by_its_roles_function_and_typed_back_in_a_row() {
        let inputs = inputs();
        let base = inputs.project(&(), 0, &2.0).expect("a positive value");
        let smooth = inputs.project(&(), 1, &4.0).expect("a positive value");
        let label = inputs.project(&(), 2, &"four".to_owned()).expect("a label");

        let row = inputs
            .row(&[&base, &smooth, &label])
            .expect("its own values");

        assert_eq!(row, (&2.0, &4.0, &4.0));
        assert_eq!(
            inputs.project(&(), 0, &-1.0).err(),
            Some(InputRejection("a negative value"))
        );
        assert_eq!(inputs.project(&(), 2, &1.0).err(), Some(NOT_ITS_PAYLOAD));
        assert_eq!(inputs.project(&(), 3, &1.0).err(), Some(NO_SUCH_ROLE));
    }

    #[test]
    fn only_an_interpolated_role_blends() {
        let inputs = inputs();
        let (before, after) = (
            inputs.project(&(), 1, &0.0).expect("kept"),
            inputs.project(&(), 1, &4.0).expect("kept"),
        );

        let blended = inputs.blend(1, &before, &after, 0.25);

        assert_eq!(
            blended.and_then(|blended| blended.downcast_ref::<f64>().copied()),
            Some(1.0)
        );
        assert!(inputs.blend(0, &before, &after, 0.25).is_none());
    }
}
