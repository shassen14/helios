# In-File Item Order

**Cluster by type.** A type's definition is immediately followed by everything
that operates on it; never interleave one type's machinery with another's.
"Where is X?" has one answer: X's cluster. Top to bottom in a file:

1. `//!` module doc, then imports (the groups in the root `CLAUDE.md`).
2. Each type, primary-first (the type the module exists to provide), then its
   supporting types in rough order-of-use. For each: the `struct`/`enum`
   definition, then its inherent `impl`, then its trait impls.
3. Free functions that aren't attached to a type.
4. `#[cfg(test)] mod tests` — always last.

Rules that settle the common questions:
- **`From<B> for A` lives in A's cluster**, not B's — it defines A's construction.
- **Inside an inherent `impl`, order by lifecycle:** constructors (`new`,
  builders) → accessors (`&self` getters) → mutators (`&mut self`) → consumers
  (`self`, e.g. `into_*`) → private helpers.
- A free `fn` that serves exactly one type may sit right after that type's
  `impl` (proximity to its sole caller beats the free-function section).
- In a file built around adding methods to a type defined elsewhere (e.g.
  `runner/tick.rs`, `runner/setup.rs`), the `impl` block is the primary
  content; a small local return type (`TickAction`, `SetupError`) follows it.
- **A doc comment says what the signature and the types can't**, in as few
  sentences as that takes.
