//! Bus-output discovery shared by the plural-channel gizmos.
//!
//! `Path`, `MapData`, and future perception/tracking outputs are *plural*
//! channels whose names belong to the agent profile, so a gizmo must never
//! hardcode one. Each instead discovers every declared output of the type it
//! draws through the pipeline's topologically ordered [`channels()`] list,
//! filters by type, and reads each by its own key. That discover-and-read half
//! is identical across those gizmos; only the drawing differs. It lives here so
//! a gizmo system supplies solely the draw, and so a new visualized type is one
//! call plus its rendering.
//!
//! This is emphatically not a "read any instance of `T`" primitive: it walks
//! only *declared producers*, in deterministic topological order, off the same
//! `channels()` metadata the runtime exposes for exactly this purpose.
//!
//! [`channels()`]: helios_runtime::pipeline::AutonomyPipeline::channels

use std::any::{Any, TypeId};
use std::sync::Arc;

use helios_runtime::prelude::{AutonomyPipeline, Stamped};

/// The last-known-good value of every declared channel whose payload is `T`, in
/// the pipeline's topological order.
///
/// Empty until a producer of `T` has published, or when the stack declares no
/// such channel at all — the caller draws nothing in either case, no
/// special-casing needed. Collected eagerly: the matching-channel count is tiny
/// (usually one), and returning owned `Arc`s keeps the borrow of `pipeline`
/// from leaking into the caller's draw loop.
pub fn declared_outputs<T: Any + Send + Sync>(pipeline: &AutonomyPipeline) -> Vec<Arc<Stamped<T>>> {
    let want = TypeId::of::<T>();
    pipeline
        .channels()
        .filter_map(|(_node, key)| {
            (key.type_id() == want)
                .then(|| pipeline.bus().read::<T>(key.clone()))
                .flatten()
        })
        .collect()
}
