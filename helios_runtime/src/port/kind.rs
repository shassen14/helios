//! Channel kinds and the table of which nodes may read and write each.
//!
//! A channel's [`ChannelKind`] says what its data is: a measurement, a value
//! one node hands another, reference truth, or a device's health. A node's
//! [`NodeClass`] says what licence it runs under: an algorithm runs the same
//! on hardware, a mock may stand in for one by reading truth. The kinds own
//! the rule, so the table lives here; the pipeline build checks every node's
//! declared inputs and outputs against it and refuses a violation naming the
//! node and the channel.
//!
//! | Kind | Written by | Algorithm reads | Algorithm writes | Mock reads | Mock writes |
//! |---|---|:-:|:-:|:-:|:-:|
//! | [`Measurement`](ChannelKind::Measurement) | body, or a node deriving one | ✅ | ✅ | ✅ | ❌ |
//! | [`Internal`](ChannelKind::Internal) | a node | ✅ | ✅ | ✅ | ✅ |
//! | [`Oracle`](ChannelKind::Oracle) | body, when truth is known | ❌ | ❌ | ✅ | ❌ |
//! | [`Health`](ChannelKind::Health) | body driver layer | ❌ | ❌ | ❌ | ❌ |

/// What a channel's data is. Carried in the channel's key so the build can
/// check who reads and writes it; the name alone still names one channel.
///
/// Ordered as declared, so keys that share a name sort the same in every
/// build.
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, Hash, Debug)]
pub enum ChannelKind {
    /// A measurement: what a sensor, drive or other device reports, or what
    /// a node derives from one without estimating (a range field flattened
    /// to a cloud, a de-skewed scan). The kind says what the data is, not
    /// who produced it.
    Measurement,
    /// A value one node writes for others: state, maps, paths, commands.
    Internal,
    /// Reference truth from a body that knows the answer: physics in sim,
    /// a survey-grade reference on hardware.
    Oracle,
    /// Driver and fault status a body publishes about its devices.
    Health,
}

impl ChannelKind {
    /// The kind's name in errors, logs and channel dumps.
    pub fn as_str(self) -> &'static str {
        match self {
            ChannelKind::Measurement => "measurement",
            ChannelKind::Internal => "internal",
            ChannelKind::Oracle => "oracle",
            ChannelKind::Health => "health",
        }
    }

    /// Whether a node of `class` may declare a channel of this kind as an
    /// input, required or optional.
    pub(crate) fn readable_by(self, class: NodeClass) -> bool {
        match (self, class) {
            (ChannelKind::Measurement | ChannelKind::Internal, _) => true,
            // Isolating an algorithm under perfect upstream swaps the node
            // upstream of it for a mock reading truth; the algorithm's code
            // and config stay identical, so its isolated and real runs
            // compare. A case no mock covers gets a new mock, never this row.
            (ChannelKind::Oracle, NodeClass::Algorithm) => false,
            (ChannelKind::Oracle, NodeClass::Mock) => true,
            // Which nodes read device health is decided with health checks.
            (ChannelKind::Health, _) => false,
        }
    }

    /// Whether a node of `class` may declare a channel of this kind as an
    /// output.
    pub(crate) fn writable_by(self, class: NodeClass) -> bool {
        match (self, class) {
            (ChannelKind::Internal, _) => true,
            (ChannelKind::Measurement, NodeClass::Algorithm) => true,
            // A measurement made from truth would reach an estimator as if
            // a device had reported it; a mock writes what it stands in for.
            (ChannelKind::Measurement, NodeClass::Mock) => false,
            // Only a body knows truth or its devices' health.
            (ChannelKind::Oracle | ChannelKind::Health, _) => false,
        }
    }
}

impl std::fmt::Display for ChannelKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The licence a node runs under, which decides the channel kinds it may
/// read and write.
#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug, Default)]
pub enum NodeClass {
    /// Runs the same in sim and on hardware, so never reads truth.
    #[default]
    Algorithm,
    /// Stands in for an algorithm, and may read truth to do it.
    Mock,
}

impl NodeClass {
    /// The class's name in errors.
    pub fn as_str(self) -> &'static str {
        match self {
            NodeClass::Algorithm => "algorithm",
            NodeClass::Mock => "mock",
        }
    }
}

impl std::fmt::Display for NodeClass {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KINDS: [ChannelKind; 4] = [
        ChannelKind::Measurement,
        ChannelKind::Internal,
        ChannelKind::Oracle,
        ChannelKind::Health,
    ];

    fn readable(class: NodeClass) -> Vec<ChannelKind> {
        KINDS
            .into_iter()
            .filter(|kind| kind.readable_by(class))
            .collect()
    }

    fn writable(class: NodeClass) -> Vec<ChannelKind> {
        KINDS
            .into_iter()
            .filter(|kind| kind.writable_by(class))
            .collect()
    }

    #[test]
    fn an_algorithm_reads_and_writes_measurements_and_internals_only() {
        let open = [ChannelKind::Measurement, ChannelKind::Internal];
        assert_eq!(readable(NodeClass::Algorithm), open);
        assert_eq!(writable(NodeClass::Algorithm), open);
    }

    #[test]
    fn a_mock_also_reads_truth_but_writes_internals_only() {
        assert_eq!(
            readable(NodeClass::Mock),
            [
                ChannelKind::Measurement,
                ChannelKind::Internal,
                ChannelKind::Oracle
            ]
        );
        assert_eq!(writable(NodeClass::Mock), [ChannelKind::Internal]);
    }
}
