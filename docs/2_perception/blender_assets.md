# Helios — Blender Asset Standards

> Reference for artists and engineers creating assets for the Helios simulation
> platform. Follow this guide to ensure every asset loads correctly, collides
> accurately, and carries the semantic metadata needed by autonomy algorithms and
> dataset exporters.

---

## Table of Contents

1. [Coordinate System and Scale](#1-coordinate-system-and-scale)
2. [Asset Categories](#2-asset-categories)
3. [File Naming Convention](#3-file-naming-convention)
4. [Object Hierarchy in Blender](#4-object-hierarchy-in-blender)
5. [Semantic Metadata (Custom Properties)](#5-semantic-metadata-custom-properties)
6. [Collision Mesh Standards](#6-collision-mesh-standards)
7. [Material and Texture Standards](#7-material-and-texture-standards)
8. [Terrain Assets](#8-terrain-assets)
9. [World Object Assets](#9-world-object-assets)
10. [Export Settings](#10-export-settings)
11. [Catalog Registration](#11-catalog-registration)
12. [Scenario Placement](#12-scenario-placement)
13. [Combining Environments](#13-combining-environments)
14. [Class ID Registry](#14-class-id-registry)
15. [Checklist](#15-checklist)

---

## 1. Coordinate System and Scale

### Scale
- **1 Blender unit = 1 meter.** Apply all transforms before export (`Ctrl+A → All
  Transforms`).
- Do not rely on object-level scale; bake it into the mesh.

### Coordinate system
Blender uses **Z-up, Y-forward**. Bevy uses **Y-up, -Z-forward**. The GLB
exporter handles this automatically when **Forward = -Z, Up = Y** is selected
in the export dialog (this is the default).

The Helios coordinate standard in simulation code:

| Frame | +X | +Y | +Z |
|---|---|---|---|
| **World (ENU)** | East | North | Up |
| **Body (FLU)** | Forward | Left | Up |
| **Bevy World** | East | Up | -North |

Your Blender models live in Blender's Z-up space. The exporter converts them to
GLTF Y-up, and Helios converts Y-up Bevy space to ENU/FLU internally. **You do
not need to rotate anything manually.**

### Origin placement
- **Objects (signs, trees, buildings):** Place the origin at the geometric
  center of the base footprint. If the object sits on the ground, origin at
  ground level (Z = 0 in Blender).
- **Terrain:** Origin at the world origin (0, 0, 0). The scenario TOML provides
  the ENU offset.

---

## 2. Asset Categories

| Category | Description | Where used |
|---|---|---|
| **Terrain** | Large ground surfaces, water bodies, cave floors | `[[world.terrains]]` |
| **World Object** | Discrete, countable semantic entities | `[[world.objects]]` |
| **Vehicle** | Agent vehicles (Ackermann, quadrotor, etc.) | `plugins/vehicles/` |
| **Sensor** | Sensor geometry, optional (rarely needed) | `plugins/sensors/` |

This document covers **Terrain** and **World Objects**. Vehicle assets follow
the same rules (vehicle mesh standards are not yet separately documented; follow this guide).

---

## 3. File Naming Convention

```
<asset_name>.glb               ← visual mesh (all materials, full detail)
<asset_name>_col.glb           ← collision mesh (no materials, simplified)
```

Examples:

```
assets/
  terrain/
    valley_floor.glb
    valley_floor_col.glb
    lake_surface.glb              ← water terrain, no collision needed
  objects/
    stop_sign.glb
    stop_sign_col.glb
    building_office.glb
    building_office_col.glb
    traffic_cone.glb              ← simple shape → use [collider] in TOML, no _col.glb needed
    tree_oak.glb
```

**Rules:**
- All lowercase, words separated by underscores.
- No spaces, no special characters.
- The `_col` suffix is the Helios convention (like Unreal's `UCX_` prefix, but as
  a separate file).
- If a collision mesh file is absent, specify a primitive collider shape in the
  object's TOML prefab instead.

---

## 4. Object Hierarchy in Blender

### World Object (single type)

```
[Empty]  stop_sign              ← root empty; carries Custom Properties
   └── [Mesh]  SM_pole          ← visual submesh (SM_ = Static Mesh)
   └── [Mesh]  SM_sign_face
```

- The **root empty** is the object's "logical entity." Its Custom Properties
  become GLTF extras and are read automatically by Helios.
- Mesh children are named `SM_<description>`. The `SM_` prefix is optional but
  helps in large scenes.
- A root mesh (no empty parent) also works; add Custom Properties directly to it.

### Terrain tile

```
[Mesh]  valley_floor            ← single root mesh is fine for terrain
```

Terrain tiles rarely need custom properties — they are identified by their TOML
`medium` field, not by GLTF metadata.

### Collision mesh (in `_col.glb`)

```
[Mesh]  col_body               ← every mesh in this file becomes a trimesh collider
[Mesh]  col_base
```

- Keep collision meshes **convex or low-poly** (< 500 triangles per mesh).
- Split a building into `col_walls`, `col_roof`, etc. if needed for accuracy.
- No materials, no UVs required.

---

## 5. Semantic Metadata (Custom Properties)

Set these as **Custom Properties** on the root object in Blender
(`Object Properties → Custom Properties`). They are exported as GLTF `extras`
and read automatically by the `read_object_gltf_extras` system in Helios at
runtime.

| Property | Type | Required | Description |
|---|---|---|---|
| `label` | `string` | Yes | Semantic class name. Must match the catalog TOML. |
| `class_id` | `int` | Yes | Integer class ID for perception and labeling. |

### How to set in Blender

1. Select the root empty (or root mesh).
2. Open **Object Properties** (orange square icon).
3. Scroll to **Custom Properties** → click **+**.
4. Set **Name** = `label`, **Value** = `"stop_sign"`.
5. Repeat for `class_id` with an integer value.

### JSON format in GLTF extras

Blender exports custom properties as a JSON object in the GLTF `extras` field:
```json
{"label": "stop_sign", "class_id": 10}
```
Helios parses this and attaches a `SemanticLabel` component to the entity. If
the object's catalog TOML already provides a label (TOML takes precedence), the
GLTF extras serve as a fallback.

### Why bother if TOML already has the label?

- The GLTF is self-describing — useful for third-party tools (Godot, Unity,
  dataset viewers) that don't read our TOML.
- A future ML dataset exporter can extract annotations directly from the GLB
  without needing the catalog.
- Keeps artist intent (what this object *is*) inside the geometry file.

---

## 6. Collision Mesh Standards

### Option A: Primitive shape in TOML (preferred for simple objects)

For signs, cones, cylinders, spheres — no collision GLB needed.
Declare the shape in `configs/catalog/objects/<name>.toml`:

```toml
[collider]
shape       = "capsule"
radius      = 0.04
half_height = 1.0
```

Supported shapes: `"box"` (half_extents), `"sphere"` (radius),
`"capsule"` (radius + half_height), `"cylinder"` (radius + half_height).

### Option B: Trimesh from `_col.glb` (for complex shapes)

Specify `collider_mesh` in the object TOML prefab:

```toml
collider_mesh = "objects/building_office_col.glb"
```

Helios loads this GLB and creates a `Collider::trimesh` from every named mesh
inside it. This file is **never rendered** — it is physics-only.

**Collision mesh guidelines:**
- Target < 500 triangles per convex piece.
- A large building: separate meshes per wall and per floor. Each becomes an
  independent trimesh body.
- Terrain tiles: always use `_col.glb`. Large terrain trimeshes are fine
  (thousands of triangles) because they use `TrimeshFlags::FIX_INTERNAL_EDGES`.
- Water surfaces: **no collision mesh**. Declare `medium = "water"` in the
  terrain TOML and let the physics medium handle buoyancy.

### Priority rule

If both `collider_mesh` and `[collider]` are set, **`collider_mesh` wins**.

---

## 7. Material and Texture Standards

- Use the **Principled BSDF** shader for all materials.
- Textures: PNG or JPEG, power-of-two sizes (512, 1024, 2048, 4096).
- Bake all modifiers before export (`Ctrl+A → Make All Modifiers Real` if
  needed, then apply).
- **Collision meshes must have no materials** — Helios ignores them but Bevy
  will warn if they reference missing textures in a `_col.glb`.
- Keep texture atlases. One texture per object type avoids Bevy's material
  batching overhead.

---

## 8. Terrain Assets

Terrain tiles are large ground surfaces that define the physical substrate of a
scene. Multiple tiles can be combined to compose environments.

### When to split into multiple tiles

| Situation | Approach |
|---|---|
| Lake adjacent to ground | Separate `ground.glb` + `lake.glb` tiles |
| Mountain + valley | One seamless tile or two snapped tiles |
| Indoor + outdoor | Separate tiles; indoor tile has no sky lighting |
| Underwater section | Separate tile with `medium = "water"` |

### Tile seams

Tile edges should meet without visible gaps. In Blender:
- Snap tile edges to exact integer coordinates.
- Use the same vertex density along shared edges.
- Export both tiles from the same `.blend` file to ensure consistent scale.

### Heightmap vs mesh

| | Heightmap (future) | Trimesh (current) |
|---|---|---|
| File format | 16-bit PNG | GLB |
| Max poly count | — | ~500k tris for terrain |
| Overhangs | No | Yes |
| Dynamic deformation | Possible | No |

Helios currently uses trimesh terrain only. Heightmap support is planned.

### Terrain performance guidelines

- Visual mesh: up to ~500k triangles for a 200×200 m tile.
- Collision mesh: 50–100k triangles is sufficient for vehicle physics. Use
  Blender's **Decimate** modifier to reduce.
- Large flat areas: use a single quad subdivided minimally.

---

## 9. World Object Assets

### Object sizing

Always model at real-world scale (1 Blender unit = 1 meter):

| Object | Approx. size (W × H × D) |
|---|---|
| Stop sign | 0.6 × 2.6 × 0.1 m |
| Traffic cone | 0.45 × 0.75 × 0.45 m |
| Office building | 20 × 30 × 15 m |
| Oak tree | 6 × 12 × 6 m |
| Street lamp | 0.2 × 6 × 0.2 m |
| Car (parked NPC) | 2 × 1.5 × 4.5 m |

### LOD (future)

Planned naming convention for future LOD support:
```
stop_sign.glb          ← LOD0 (full detail, default)
stop_sign_lod1.glb     ← LOD1 (50% reduction)
stop_sign_lod2.glb     ← LOD2 (25% reduction)
```

### Instancing

Multiple instances of the same object type share **one loaded GLB** in memory
(Bevy handles this automatically via `Handle<Scene>`). You never need to
duplicate the GLB for multiple placements — just add more `[[world.objects]]`
entries in the scenario TOML.

---

## 10. Export Settings

In Blender: **File → Export → glTF 2.0 (.glb/.gltf)**

### Settings for visual mesh

| Setting | Value |
|---|---|
| Format | `glTF Binary (.glb)` |
| Include → Selected Objects | Off (export all) |
| Transform → +Y Up | **On** |
| Geometry → Apply Modifiers | **On** |
| Geometry → UVs | On |
| Geometry → Normals | On |
| Geometry → Tangents | On (if using normal maps) |
| Geometry → Vertex Colors | On (if used) |
| Material → Export | **On**, PBR Extensions |
| Animation | Off (static objects) |
| Extras → Export custom properties | **On** ← required for semantic metadata |

### Settings for collision mesh (`_col.glb`)

Same as above, except:
- Material → Export: **Off**
- Include only the collision mesh objects (use Collections or selection)

### Command-line export (scripting)

For CI/pipeline automation, use Blender's Python API:
```python
bpy.ops.export_scene.gltf(
    filepath="/path/to/stop_sign.glb",
    export_format="GLB",
    export_yup=True,
    export_apply=True,
    export_extras=True,          # Custom Properties → GLTF extras
    export_materials="EXPORT",
    export_animations=False,
)
```

---

## 11. Catalog Registration

Every object type needs a TOML prefab in `configs/catalog/objects/`.

```toml
# configs/catalog/objects/stop_sign.toml

label    = "stop_sign"
class_id = 10

# Path relative to the Bevy asset root (assets/ directory).
visual_mesh   = "objects/stop_sign.glb"
# collider_mesh = "objects/stop_sign_col.glb"  ← use this OR [collider], not both

# Axis-aligned bounding box: [width, height, depth] in meters.
# Used for debug visualization and sensor hit attribution.
bounding_box = [0.6, 2.6, 0.6]

# Primitive collider (alternative to collider_mesh for simple shapes).
[collider]
shape       = "capsule"
radius      = 0.04
half_height = 1.0
```

### Full field reference

| Field | Type | Required | Description |
|---|---|---|---|
| `label` | string | Yes | Semantic class name |
| `class_id` | u32 | Yes | Integer class ID (see registry below) |
| `visual_mesh` | path | No | GLB for visual rendering |
| `collider_mesh` | path | No | GLB for trimesh physics (takes priority over `[collider]`) |
| `bounding_box` | [f32;3] | No | [W, H, D] in meters |
| `[collider].shape` | string | No | `"box"`, `"sphere"`, `"capsule"`, `"cylinder"` |

---

## 12. Scenario Placement

Place object instances in a scenario TOML using `[[world.objects]]`.

```toml
# configs/scenarios/my_scene.toml

[[world.terrains]]
mesh     = "terrain/urban_flat.glb"
collider = "terrain/urban_flat_col.glb"
medium   = "air"

[world.atmosphere]
gravity       = [0.0, -9.81, 0.0]
sun_elevation = 45.0
sun_azimuth   = 180.0     # 0=North, 90=East, 180=South, 270=West
ambient_lux   = 8000.0

# --- Object instances ---

[[world.objects]]
prefab   = "objects.stop_sign"
position = [10.0, 5.0, 0.0]          # ENU: [east, north, up] meters
orientation_degrees = [0.0, 0.0, 0.0] # [roll, pitch, yaw]

[[world.objects]]
prefab   = "objects.building_office"
position = [50.0, 30.0, 0.0]
orientation_degrees = [0.0, 0.0, 45.0]   # rotate 45° about vertical axis
scale = [1.0, 1.5, 1.0]                  # stretch 1.5× vertically
```

### Coordinate reference

- `position` is in **ENU world frame**: `[east_m, north_m, up_m]`.
- `orientation_degrees` is **[roll, pitch, yaw]** in degrees, applied in ENU.
- `scale` is applied to the visual mesh only (not the collider).

---

## 13. Combining Environments

Helios supports **multiple terrain tiles per scenario**, enabling composite
environments such as a coastal road (ground + water) or a mountain pass (valley
floor + cliff face).

### Example: Ground + Lake

```toml
[[world.terrains]]
mesh     = "terrain/valley_ground.glb"
collider = "terrain/valley_ground_col.glb"
medium   = "air"
# No position offset — this tile is at the world origin.

[[world.terrains]]
mesh     = "terrain/lake_surface.glb"
medium   = "water"
position = [80.0, 40.0, -1.5]    # ENU: place 80m east, 40m north, 1.5m below ground
# No collider — agents don't stand on water; physics medium handles buoyancy.
```

### Medium types

| Value | Physics effect | Sensor effects |
|---|---|---|
| `"air"` | Standard gravity, no drag | All sensors work normally |
| `"water"` | Added drag, buoyancy (future) | GPS blocked, acoustic sensors enabled (future) |
| `"vacuum"` | No drag, full gravity | No acoustic sensors (future) |

The `medium` field on a terrain entity is queryable from ECS:
```rust
// In a sensor system: check if agent is over water terrain
fn check_terrain_medium(
    agent_query: Query<&GlobalTransform, With<GroundTruthState>>,
    terrain_query: Query<(&GlobalTransform, &TerrainMedium)>,
) {
    // ... range check agent position against terrain tiles
}
```

### Agent traversability (future config)

Planned field in `AgentBaseConfig`:
```toml
traversable_mediums = ["air"]       # ground vehicle
# traversable_mediums = ["water"]   # AUV
# traversable_mediums = ["air", "water"]  # amphibious
```

This will gate physics medium effects per-agent.

---

## 14. Class ID Registry

Reserve class ID ranges by category. IDs must be unique across the entire
catalog.

| Range | Category | Examples |
|---|---|---|
| 0 | Background / unlabeled | — |
| 1–9 | Infrastructure | road, sidewalk, crosswalk, lane marking |
| 10–19 | Traffic control | stop_sign, traffic_light, yield_sign, cone |
| 20–29 | Structures | building, wall, fence, bridge |
| 30–39 | Vegetation | tree, shrub, grass, hedge |
| 40–49 | Vehicles (static NPC) | parked_car, parked_truck, bus |
| 50–59 | Vehicles (dynamic NPC, future) | car, bicycle, motorcycle |
| 60–69 | Pedestrians (future) | person, cyclist |
| 70–79 | Terrain features | rock, hill, water_surface, sand_dune |
| 80–89 | Props | bench, mailbox, trash_can, fire_hydrant |
| 90–99 | Utility | power_pole, utility_box, manhole |
| 100+ | Custom / experimental | project-specific additions |

**Current assignments:**

| label | class_id |
|---|---|
| `stop_sign` | 10 |
| `traffic_cone` | 11 |
| `building` | 20 |
| `tree` | 30 |

---

## 15. Checklist

Use this checklist before committing a new asset.

### Blender preparation
- [ ] Scale applied (`Ctrl+A → All Transforms`)
- [ ] Origin placed at base center, Z = 0 at ground level
- [ ] No modifiers left un-applied
- [ ] Root empty (or root mesh) has `label` and `class_id` Custom Properties
- [ ] `label` value matches the catalog TOML exactly

### Visual mesh (`<name>.glb`)
- [ ] Export: `+Y Up` enabled
- [ ] Export: `Export custom properties` enabled
- [ ] All materials use Principled BSDF
- [ ] Textures are power-of-two resolution
- [ ] Poly count reasonable for intended use (see guidelines in §9)

### Collision mesh (`<name>_col.glb`)
- [ ] No materials assigned
- [ ] Simplified geometry (convex pieces preferred)
- [ ] All meshes have unique names (used as collider entity names in engine)
- [ ] OR: primitive collider declared in TOML instead (sufficient for simple shapes)

### Catalog entry (`configs/catalog/objects/<name>.toml`)
- [ ] `label` and `class_id` present
- [ ] `visual_mesh` path correct (relative to Bevy asset root `assets/`)
- [ ] `collider_mesh` OR `[collider]` section present (if physics needed)
- [ ] `bounding_box` specified (optional but recommended)
- [ ] Class ID unique and assigned from the correct range

### Terrain tile (`configs/catalog/objects/` or direct `[[world.terrains]]`)
- [ ] `medium` field set correctly (`"air"` / `"water"` / `"vacuum"`)
- [ ] `collider` GLB path specified (unless medium = "water")
- [ ] Tile edges align with adjacent tiles (no gaps)

### Scenario placement
- [ ] `position` is in ENU meters (not Bevy space)
- [ ] `orientation_degrees` is [roll, pitch, yaw] in degrees
- [ ] Object visible and properly grounded when running simulation
