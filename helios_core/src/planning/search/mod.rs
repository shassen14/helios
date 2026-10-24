//! The search family: planners that operate on a discrete graph/grid and find
//! an optimal path under a cost function (A*, Dijkstra, D*). They share the
//! [`SearchSpace`](search_space::SearchSpace) abstraction over the environment
//! they query.

pub mod astar;
pub mod search_space;
