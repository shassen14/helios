### **System Design: The TF (Transform) System**

**Purpose:**
To solve the "where is this sensor relative to the robot?" problem dynamically, allowing for articulation and complex sensor setups without hard-coded offsets.

**Architecture:**

1.  **The `TrackedFrame` Component:**
    - A marker component attached to any entity (robot base, sensor, wheel) whose transform needs to be queryable by the core algorithms.

2.  **The `TfTree` Resource:**
    - A read-only snapshot of the entire transform hierarchy for the _current frame_.
    - It is rebuilt once per frame in the `Precomputation` schedule.
    - **Why rebuild?** It's faster to rebuild a flat lookup table (HashMap) once than to have every sensor traverse the parent-child hierarchy recursively every time it needs a transform.

3.  **Lookup API:**
    - `lookup_transform(entity_id) -> Isometry3`: Returns global pose.
    - `lookup_relative(from, to) -> Isometry3`: Returns the transform from frame A to frame B.
