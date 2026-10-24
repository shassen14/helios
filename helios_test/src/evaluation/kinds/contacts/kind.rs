//! [`Contacts`]: the `contacts` kind, counting contact episodes (of the
//! body as a whole, and per side it touched) and time in contact from one
//! body's touches.

use super::config::ContactsConfig;
use crate::evaluation::{
    kind::{BoundInput, InputRejection},
    typed::{BaseFacts, Component, OneInput},
};

use helios_core::interchange::perception::contact::{self, Touched};
use helios_core::kernel::si_unit::SiUnit;
use helios_core::prelude::{FrameId, MonotonicTime};

use std::collections::{BTreeMap, BTreeSet};

/// _Per-step._ The `contacts` kind, bound to one entry: reads what `body`
/// touched each tick, each value checked for it. A touch more than
/// `merge_gap` seconds after the last one starts a new episode, counted for
/// the body touching anything and for each side it touched.
pub struct Contacts {
    body: FrameId,
    merge_gap: f64,
}

impl Contacts {
    /// Whether a touch at `at` starts an episode: nothing was touched at the
    /// previous value, and the last touch, if any, is more than `merge_gap`
    /// back.
    fn starts_episode(
        &self,
        was_touched: bool,
        last_touched: Option<MonotonicTime>,
        at: MonotonicTime,
    ) -> bool {
        !was_touched && last_touched.is_none_or(|last| at.0 - last.0 > self.merge_gap)
    }
}

impl OneInput for Contacts {
    const NAME: &'static str = "contacts";
    /// The body's touches, its instance the body that touched.
    const BASE: &'static str = "contacts";
    type Params = ContactsConfig;
    type BasePayload = contact::Contacts;
    type BaseValue = Vec<Touched>;
    type Fit = ();
    type State = Episodes;
    type Output = Touching;

    fn keep_base(&self, contacts: &contact::Contacts) -> Result<Vec<Touched>, InputRejection> {
        if contacts.body == self.body {
            Ok(contacts
                .touching()
                .iter()
                .map(|contact| contact.other.clone())
                .collect())
        } else {
            Err(InputRejection("contacts of another body"))
        }
    }

    fn menu() -> Option<Vec<Component<Touching>>> {
        Some(components())
    }

    fn check(params: &ContactsConfig) -> Result<(), String> {
        params.check()
    }

    /// The input must select the body whose touches it reads.
    fn bind(params: ContactsConfig, contacts: &BoundInput) -> Result<Self, Vec<String>> {
        match contacts.frame(Self::BASE) {
            Ok(Some(body)) => Ok(Self {
                body,
                merge_gap: params.merge_gap,
            }),
            Ok(None) => Err(vec![format!(
                "the {} has no instance: {} reads one body's touches, so select the body as its \
                 instance",
                Self::BASE,
                Self::NAME
            )]),
            Err(problem) => Err(vec![problem]),
        }
    }

    fn components(&self) -> Vec<Component<Touching>> {
        components()
    }

    fn fit(&self, _run: &[&Vec<Touched>]) -> Result<(), InputRejection> {
        Ok(())
    }

    fn compute(
        &self,
        touching: &Vec<Touched>,
        facts: BaseFacts,
        episodes: &mut Episodes,
        _fit: &(),
    ) -> Result<Touching, InputRejection> {
        if !touching.is_empty() {
            let was_touching = !episodes.previous.is_empty();
            if self.starts_episode(was_touching, episodes.body_last_touched, facts.at) {
                episodes.body += 1;
            }
            episodes.body_last_touched = Some(facts.at);
        }
        for side in touching {
            let was_touched = episodes.previous.contains(side);
            let last_touched = episodes.last_touched.get(side).copied();
            if self.starts_episode(was_touched, last_touched, facts.at) {
                episodes.sides += 1;
            }
            episodes.last_touched.insert(side.clone(), facts.at);
        }
        episodes.previous = touching.iter().cloned().collect();
        Ok(Touching {
            contact_episodes: episodes.body,
            collisions: episodes.sides,
            in_contact: !touching.is_empty(),
        })
    }
}

/// _Per-step._ What [`Contacts`] carries from one value to the next: the
/// body's episodes so far and when it last touched anything, the episodes
/// of every side so far, the sides touched at the previous value, and when
/// each side was last touched.
#[derive(Debug, Default)]
pub struct Episodes {
    body: usize,
    body_last_touched: Option<MonotonicTime>,
    sides: usize,
    previous: BTreeSet<Touched>,
    last_touched: BTreeMap<Touched, MonotonicTime>,
}

/// _Per-step._ One value's output: the body's episodes and every side's so
/// far, and whether the body touched anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Touching {
    contact_episodes: usize,
    collisions: usize,
    in_contact: bool,
}

/// The running episode counts, and contact as 1 or 0, unbridged, so its time
/// above a half is the raw time touching. `contact_episodes` counts the body
/// touching anything, so a crash into a cone and a wall at once is one;
/// `collisions` counts each side apart, so it is two, and a scrape along a
/// wall built of three placements is three.
fn components() -> Vec<Component<Touching>> {
    vec![
        Component::each(
            "collisions",
            SiUnit::Dimensionless,
            |touching: &Touching| Ok(touching.collisions as f64),
        ),
        Component::each(
            "in_contact",
            SiUnit::Dimensionless,
            |touching: &Touching| Ok(f64::from(u8::from(touching.in_contact))),
        ),
        Component::each(
            "contact_episodes",
            SiUnit::Dimensionless,
            |touching: &Touching| Ok(touching.contact_episodes as f64),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::assertion::AssertionValue;
    use crate::evaluation::{kind::EvaluatorKind, typed::OneInputKind, Gap, GapWidth};

    use helios_core::interchange::perception::contact::Contact;
    use helios_core::interchange::perception::object_instance::ObjectInstanceId;
    use helios_core::interchange::perception::semantic_class::SemanticClass;
    use helios_core::prelude::AgentId;

    const COLLISIONS: usize = 0;
    const IN_CONTACT: usize = 1;
    const CONTACT_EPISODES: usize = 2;

    /// The tick the values arrive at, 400 Hz.
    const TICK: f64 = 0.0025;

    fn body() -> FrameId {
        FrameId::base_link(AgentId::new("car"))
    }

    fn wall() -> Touched {
        Touched::Object {
            instance: ObjectInstanceId::new("yard", "wall"),
            class: SemanticClass::UNLABELED,
        }
    }

    fn input(instance: Option<String>) -> BoundInput {
        BoundInput {
            agent: AgentId::new("car"),
            instance,
            expressed_in: None,
            convention: None,
            placement: None,
            schema: None,
            rate_hz: None,
            writer: None,
        }
    }

    fn params(merge_gap: f64) -> toml::Table {
        let mut params = toml::Table::new();
        params.insert("merge_gap".to_owned(), toml::Value::Float(merge_gap));
        params
    }

    /// The evaluation, at `merge_gap`, of one value a tick, each touching
    /// the sides `ticks` gives it, and its `metric` at every value.
    fn evaluated(merge_gap: f64, ticks: &[Vec<Touched>], metric: usize) -> Vec<f64> {
        let mut evaluation = OneInputKind::<Contacts>::new()
            .build(
                &params(merge_gap),
                &[input(Some(body().to_string()))],
                &[Some(Gap::Seconds(
                    GapWidth::try_from(1.0).expect("positive"),
                ))],
            )
            .unwrap_or_else(|problems| panic!("{problems:?}"));
        for (tick, touching) in ticks.iter().enumerate() {
            let at = MonotonicTime(tick as f64 * TICK);
            let contacts = contact::Contacts::new(
                body(),
                at,
                touching.iter().map(|other| Contact {
                    other: other.clone(),
                }),
            );
            evaluation.receive(0, at, &contacts);
        }
        let until = MonotonicTime(ticks.len() as f64 * TICK);
        evaluation.derive(until, &[metric]).metrics[metric]
            .as_ref()
            .expect("wanted")
            .samples()
            .iter()
            .map(|sample| match sample.value {
                AssertionValue::Float(value) => value,
                ref other => panic!("a float, not {other:?}"),
            })
            .collect()
    }

    /// The wall touched for two ticks, missed for one, touched for one,
    /// missed for nine, then touched again.
    fn chatter() -> Vec<Vec<Touched>> {
        let mut ticks = vec![vec![wall()], vec![wall()], vec![], vec![wall()]];
        ticks.extend(std::iter::repeat_n(vec![], 9));
        ticks.push(vec![wall()]);
        ticks
    }

    #[test]
    fn a_gap_of_zero_counts_every_re_touch() {
        let collisions = evaluated(0.0, &chatter(), COLLISIONS);

        assert_eq!(collisions.last(), Some(&3.0));
        assert_eq!(
            &collisions[..4],
            &[1.0, 1.0, 1.0, 2.0],
            "a held touch is one"
        );
    }

    #[test]
    fn a_re_touch_within_the_gap_continues_its_episode() {
        // The one-tick miss is 2 ticks between touches, the nine-tick miss 10.
        let short = evaluated(3.0 * TICK, &chatter(), COLLISIONS);
        let long = evaluated(11.0 * TICK, &chatter(), COLLISIONS);

        assert_eq!(short.last(), Some(&2.0));
        assert_eq!(long.last(), Some(&1.0));
    }

    #[test]
    fn each_side_has_its_own_episodes() {
        let truck = Touched::Agent(AgentId::new("truck"));
        let ticks = vec![
            vec![wall()],
            vec![wall(), truck.clone()],
            vec![truck],
            vec![wall()],
        ];

        let collisions = evaluated(0.0, &ticks, COLLISIONS);

        assert_eq!(collisions, vec![1.0, 2.0, 2.0, 3.0]);
    }

    #[test]
    fn the_body_has_one_episode_while_it_touches_anything() {
        let truck = Touched::Agent(AgentId::new("truck"));
        let ticks = vec![
            vec![wall()],
            vec![wall(), truck.clone()],
            vec![truck],
            vec![],
            vec![wall()],
        ];

        let at_zero = evaluated(0.0, &ticks, CONTACT_EPISODES);
        let bridged = evaluated(2.0 * TICK, &ticks, CONTACT_EPISODES);

        assert_eq!(at_zero, vec![1.0, 1.0, 1.0, 1.0, 2.0]);
        assert_eq!(bridged.last(), Some(&1.0));
    }

    #[test]
    fn in_contact_is_the_raw_touch_with_no_bridging() {
        let in_contact = evaluated(1.0, &chatter()[..4], IN_CONTACT);

        assert_eq!(in_contact, vec![1.0, 1.0, 0.0, 1.0]);
    }

    #[test]
    fn another_bodys_contacts_are_refused() {
        let contacts = Contacts {
            body: body(),
            merge_gap: 0.0,
        };
        let truck = contact::Contacts::new(
            FrameId::base_link(AgentId::new("truck")),
            MonotonicTime(0.0),
            [Contact { other: wall() }],
        );

        assert_eq!(
            contacts.keep_base(&truck),
            Err(InputRejection("contacts of another body"))
        );
    }

    #[test]
    fn no_instance_is_refused() {
        let config = ContactsConfig { merge_gap: 1.0 };

        let problems = Contacts::bind(config, &input(None)).err();

        assert!(problems.is_some_and(|problems| problems[0].contains("has no instance")));
    }

    #[test]
    fn a_missing_gap_is_refused_at_load_naming_the_kind() {
        let kind = OneInputKind::<Contacts>::new();

        let missing = kind.check_params(&toml::Table::new()).unwrap_err();
        let negative = kind.check_params(&params(-1.0)).unwrap_err();

        assert!(
            missing.starts_with("contacts: missing field `merge_gap`"),
            "{missing}"
        );
        assert!(
            negative.starts_with("contacts: merge_gap = -1"),
            "{negative}"
        );
    }
}
