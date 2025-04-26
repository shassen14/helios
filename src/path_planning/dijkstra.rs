// src/path_planning/dijkstra.rs
use super::grid_utils::GridCoord; // Use GridCoord as Node type
use num_traits::Zero;
use std::{
    cmp::{Eq, Ord, Ordering, PartialEq, PartialOrd},
    collections::HashMap,
    hash::Hash,
    option::Option,
};

// Type aliases for clarity
type Node = GridCoord;
type Cost = f64;

// DijkstraItem struct (unchanged logic, uses Node/Cost aliases)
#[derive(Debug, Copy, Clone)]
struct DijkstraItem {
    node: Node,
    parent: Option<Node>,
    cost: Cost,
}
// ... (impl Eq, PartialEq, Ord, PartialOrd for DijkstraItem - same logic) ...
impl Eq for DijkstraItem {}
impl PartialEq for DijkstraItem {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl Ord for DijkstraItem {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost
            .partial_cmp(&other.cost)
            .unwrap_or(Ordering::Equal)
    }
} // Use partial_cmp for f64
impl PartialOrd for DijkstraItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
} // Reversed for min-heap behavior

// Generic plan function (adapted slightly for Node/Cost types)
pub fn plan<FN, IT>(
    start: &Node,
    mut is_goal: impl FnMut(&Node) -> bool, // Simplified finish_fn to is_goal closure
    get_neighbors: &mut FN,                 // Renamed neighbor_fn for clarity
) -> Option<Vec<Node>>
where
    FN: FnMut(&Node) -> IT,
    IT: IntoIterator<Item = (Node, Cost)>, // Neighbor function returns Node and Cost
{
    let start_item = DijkstraItem {
        node: *start,
        parent: None,
        cost: Cost::zero(),
    };
    let mut open_set: HashMap<Node, DijkstraItem> = HashMap::new(); // Consider BinaryHeap for efficiency
    let mut closed_set: HashMap<Node, DijkstraItem> = HashMap::new();

    open_set.insert(*start, start_item);

    while !open_set.is_empty() {
        // Find node with minimum cost in open_set (inefficient with HashMap)
        let current_node = *open_set
            .iter()
            .min_by(|a, b| a.1.cost.partial_cmp(&b.1.cost).unwrap_or(Ordering::Equal))
            .map(|(node, _item)| node)?; // Get node or return None if open_set empty (shouldn't happen here)

        let current_item = open_set.remove(&current_node).unwrap(); // Should exist

        if is_goal(&current_node) {
            closed_set.insert(current_node, current_item); // Add goal to closed set
            return Some(calculate_final_path(&current_item, &closed_set)); // Path found
        }

        closed_set.insert(current_node, current_item);

        for (neighbor_node, move_cost) in get_neighbors(&current_item.node) {
            if closed_set.contains_key(&neighbor_node) {
                continue; // Skip already processed nodes
            }

            let new_cost = current_item.cost + move_cost;
            let neighbor_item = DijkstraItem {
                node: neighbor_node,
                parent: Some(current_node),
                cost: new_cost,
            };

            match open_set.get_mut(&neighbor_node) {
                Some(existing_item) => {
                    // If found a cheaper path to this neighbor, update it
                    if new_cost < existing_item.cost {
                        *existing_item = neighbor_item;
                    }
                }
                None => {
                    // If neighbor not in open set, add it
                    open_set.insert(neighbor_node, neighbor_item);
                }
            }
        }
    }

    None // No path found
}

// calculate_final_path function (unchanged logic, uses Node/Cost aliases)
fn calculate_final_path(
    goal_item: &DijkstraItem,
    closed_set: &HashMap<Node, DijkstraItem>,
) -> Vec<Node> {
    let mut path = Vec::new();
    let mut current_opt = Some(*goal_item);

    while let Some(current) = current_opt {
        path.push(current.node);
        if let Some(parent_node) = current.parent {
            current_opt = closed_set.get(&parent_node).copied(); // Get parent item from closed set
        } else {
            break; // Reached start node
        }
    }
    path.reverse(); // Reverse to get start -> goal order
    path
}
