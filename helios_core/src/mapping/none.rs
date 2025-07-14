use crate::estimation::FilterContext;
use crate::mapping::{MapData, Mapper};
use crate::messages::ModuleInput;

/// A placeholder mapper that does nothing.
/// It is used when an agent is configured with an estimator but no mapping capability.
#[derive(Default, Debug, Clone)]
pub struct NoneMapper;

impl Mapper for NoneMapper {
    /// This implementation does nothing with the inputs.
    fn process(&mut self, _input: &ModuleInput, _context: &FilterContext) {
        // No-op
    }

    /// This implementation always returns an empty map.
    fn get_map(&self) -> &MapData {
        // We can't return a temporary, so we create a static one.
        static EMPTY_MAP: MapData = MapData::None;
        &EMPTY_MAP
    }
}
