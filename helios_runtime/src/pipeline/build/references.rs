//! References, the world frame a body channel's values are expressed in or
//! relative to: each must be one of the host's world frames, which give it
//! its meaning and its axes.

use crate::{
    port::{ChannelDescription, Writer},
    PipelineBuildError,
};

use helios_core::spatial::{transforms::Convention, FrameId};

use std::collections::HashMap;

/// Every reference a body channel states must be one of the host's world
/// frames, `host_frames`. Only the host knows where its world is, so a
/// reference it doesn't declare means nothing the build can check. One error
/// per channel.
pub(super) fn check_references(
    descriptions: &[ChannelDescription],
    host_frames: &HashMap<FrameId, Convention>,
    errors: &mut Vec<PipelineBuildError>,
) {
    for description in descriptions {
        let (Writer::Body { unit, .. }, Some(reference)) =
            (description.writer(), description.reference())
        else {
            continue;
        };
        if !host_frames.contains_key(reference) {
            errors.push(PipelineBuildError::ReferenceNotWorldFrame {
                unit: unit.clone(),
                channel: description.key().clone(),
                reference: reference.clone(),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::port::{ChannelKey, Provenance};
    use crate::BodyChannel;

    struct Reading;

    fn datum() -> FrameId {
        FrameId::global("datum")
    }

    fn gps(reference: FrameId) -> ChannelDescription {
        let channel = BodyChannel::new(
            ChannelKey::measurement::<Reading>("gps"),
            "gps",
            Provenance::Simulated,
        )
        .with_reference(reference);
        ChannelDescription::body(&channel)
    }

    fn reference_errors(descriptions: &[ChannelDescription]) -> Vec<PipelineBuildError> {
        let host_frames = HashMap::from([(datum(), Convention::Enu)]);
        let mut errors = vec![];
        check_references(descriptions, &host_frames, &mut errors);
        errors
    }

    #[test]
    fn a_reference_the_host_declares_passes() {
        assert!(reference_errors(&[gps(datum())]).is_empty());
    }

    #[test]
    fn a_channel_without_a_reference_passes() {
        let imu = BodyChannel::new(
            ChannelKey::measurement::<Reading>("imu"),
            "imu",
            Provenance::Simulated,
        );
        assert!(reference_errors(&[ChannelDescription::body(&imu)]).is_empty());
    }

    #[test]
    fn a_reference_the_host_does_not_declare_is_refused_naming_it() {
        let errors = reference_errors(&[gps(FrameId::global("site"))]);

        assert_eq!(errors.len(), 1);
        assert!(matches!(
            &errors[0],
            PipelineBuildError::ReferenceNotWorldFrame { unit, reference, .. }
                if unit == "gps" && *reference == FrameId::global("site")
        ));
        assert!(errors[0].to_string().ends_with(
            "is expressed relative to global.site, which is not one of the host's \
             world frames; the host declares each frame its values name"
        ));
    }
}
