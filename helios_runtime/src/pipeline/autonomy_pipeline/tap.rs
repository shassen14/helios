//! [`Tap`]: one channel instance the pipeline can record, whoever writes it.

use crate::{
    observe::key::{RecordedKey, Subject},
    port::{ChannelKey, Determinism, ErasedStamped, InstanceKey, Observable, PortBus, SlotVersion},
};

use helios_core::prelude::AgentId;

use std::sync::Arc;

/// What is written to one channel, or to one instance of a keyed one: the
/// value itself, shared with the bus, recorded while the tap is watched.
///
/// The tap's key names the channel, never the writer, so a value is recorded
/// under the same key whichever node, unit or outside sender writes it. When
/// it is recorded is the writer's side: a node's tap around the node's run,
/// a host-written channel's at the start of a tick.
#[derive(Debug, Clone, PartialEq)]
pub(in crate::pipeline) struct Tap {
    /// The key the tap records under: a [`Subject::Channel`] of the
    /// pipeline's agent.
    pub(in crate::pipeline) key: RecordedKey,
    pub(in crate::pipeline) channel: ChannelKey,
    /// The instance recorded, on a keyed channel.
    pub(in crate::pipeline) instance: Option<InstanceKey>,
    /// Whether what the writer puts here repeats across runs given the same
    /// seed, as the writer is labelled.
    pub(in crate::pipeline) determinism: Determinism,
    pub(in crate::pipeline) watched: bool,
}

impl Tap {
    /// An unwatched tap on `instance` of `channel`, a channel of `agent`'s
    /// pipeline.
    pub(in crate::pipeline) fn new(
        agent: &AgentId,
        channel: &ChannelKey,
        instance: Option<InstanceKey>,
        determinism: Determinism,
    ) -> Self {
        Self {
            key: RecordedKey {
                agent: agent.clone(),
                subject: Subject::channel(channel.clone(), instance.clone()),
            },
            channel: channel.clone(),
            instance,
            determinism,
            watched: false,
        }
    }

    /// The subject this tap records.
    pub(in crate::pipeline) fn subject(&self) -> &Subject {
        &self.key.subject
    }

    /// The catalog entry for this tap: a value of the channel's payload.
    pub(in crate::pipeline) fn observable(&self) -> Observable {
        Observable::typed(self.channel.name().clone(), self.determinism, &self.channel)
    }

    /// The tapped value and its version, `None` while unwritten.
    pub(in crate::pipeline) fn read(
        &self,
        bus: &PortBus,
    ) -> Option<(Arc<dyn ErasedStamped>, SlotVersion)> {
        match &self.instance {
            Some(instance) => bus.read_erased_instance_versioned(&self.channel, instance),
            None => bus.read_erased_versioned(&self.channel),
        }
    }
}

/// Each instance one writer writes on `channel`: `declared`, in declaration
/// order, on a keyed channel, and `None` alone on any other.
pub(in crate::pipeline) fn tapped_instances(
    channel: &ChannelKey,
    declared: &[InstanceKey],
) -> Vec<Option<InstanceKey>> {
    if channel.is_keyed() {
        declared.iter().cloned().map(Some).collect()
    } else {
        vec![None]
    }
}
