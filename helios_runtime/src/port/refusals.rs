//! The bus's record of the writes it refused: per channel, how many and why
//! the first was refused.
//!
//! Every write, a host's or a node's, passes through the [`PortBus`], so the
//! bus is the one place a dropped value cannot go unseen. A writer still warns
//! when its write is refused; the record is the fact a harness or a preflight
//! judges a run by, since a run whose wiring dropped values is unfit to judge
//! however its assertions came out.
//!
//! [`PortBus`]: crate::port::PortBus

use crate::port::{ChannelError, ChannelKey};

use std::{collections::HashMap, sync::Mutex};

/// The writes the bus refused on one channel: how many, and the error the
/// first was refused with.
#[derive(Debug, Clone)]
pub struct RefusedWrites {
    channel: ChannelKey,
    count: u64,
    first: ChannelError,
}

impl RefusedWrites {
    pub fn channel(&self) -> &ChannelKey {
        &self.channel
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    /// The error the first refused write was refused with.
    pub fn first(&self) -> &ChannelError {
        &self.first
    }
}

impl std::fmt::Display for RefusedWrites {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} refused {} write(s), the first because {}",
            self.channel, self.count, self.first
        )
    }
}

/// The refused writes of one bus, keyed by channel. Touched only when a write
/// is refused, so an accepted write never takes its lock.
#[derive(Debug, Default)]
pub(crate) struct RefusalLog {
    refused: Mutex<HashMap<ChannelKey, RefusedWrites>>,
}

impl RefusalLog {
    /// Every channel with a refused write, ordered by channel so a report
    /// reads the same on every run.
    pub(crate) fn refused_writes(&self) -> Vec<RefusedWrites> {
        let Ok(refused) = self.refused.lock() else {
            return Vec::new();
        };
        let mut writes: Vec<RefusedWrites> = refused.values().cloned().collect();
        writes.sort_by_cached_key(|refused| refused.channel.to_string());
        writes
    }

    /// Counts `result` against `channel` if the write was refused.
    pub(crate) fn note(&self, channel: &ChannelKey, result: &Result<(), ChannelError>) {
        let Err(error) = result else {
            return;
        };
        let Ok(mut refused) = self.refused.lock() else {
            return;
        };
        refused
            .entry(channel.clone())
            .and_modify(|refused| refused.count = refused.count.saturating_add(1))
            .or_insert_with(|| RefusedWrites {
                channel: channel.clone(),
                count: 1,
                first: error.clone(),
            });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_accepted_write_records_nothing() {
        let log = RefusalLog::default();
        log.note(&ChannelKey::internal::<u32>("a"), &Ok(()));

        assert!(log.refused_writes().is_empty());
    }

    #[test]
    fn refusals_are_counted_per_channel_keeping_the_first_error() {
        let log = RefusalLog::default();
        let a = ChannelKey::internal::<u32>("a");
        let b = ChannelKey::internal::<u32>("b");
        log.note(&b, &Err(ChannelError::UnknownChannel));
        log.note(&a, &Err(ChannelError::KeyedChannel));
        log.note(&a, &Err(ChannelError::NotKeyed));

        let refused = log.refused_writes();

        assert_eq!(refused.len(), 2);
        assert_eq!(refused[0].channel(), &a);
        assert_eq!(refused[0].count(), 2);
        assert!(matches!(refused[0].first(), ChannelError::KeyedChannel));
        assert_eq!(refused[1].channel(), &b);
        assert_eq!(refused[1].count(), 1);
    }
}
