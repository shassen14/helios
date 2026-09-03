use crate::state::{Component, Quantity};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct StateVariable {
    quantity: Quantity,
    component: Component,
}

impl StateVariable {
    pub fn new(quantity: Quantity, component: Component) -> Self {
        Self {
            quantity,
            component,
        }
    }

    pub fn quantity(&self) -> &Quantity {
        &self.quantity
    }

    pub fn component(&self) -> &Component {
        &self.component
    }
}
