# Blender Asset Guide

*Written 2026-09-28 for Blender 5.1. Assumes no Blender experience.*

This guide takes you from an empty Blender window to an asset Helios can
load. It builds a traffic cone first (to learn the pipeline, including a
collider part), then the pieces of a first world (a crate, a wall segment
and a ground slab), then a low-poly parked car.

**The whole contract is four rules:**

1. **One `.blend` file is one asset.** The file name is the asset name.
2. **Model it at the world origin, standing on the grid, facing +X.**
3. **Name collider parts `col_<anything>`.** Everything else is visual.
4. **Export with the script** (Part 5), never the export dialog.

Nothing else is required. Where each object's origin is, whether you
applied transforms, and whether parts are parented don't matter: Helios
reads the file's geometry relative to the world origin, exactly as you
see it in Blender. What can go wrong is checked at load (Part 8).

Blender holds geometry only. What the object *is* (class, mass, whether
it collides) lives in its prefab TOML, so changing those never needs a
re-export ([configuration.md](configuration.md) §4).

---

## Part 1. Blender basics you need

### 1.1 The window

When Blender opens, close the splash screen by clicking in the 3D view.

| Area | Where | What it is for |
| ---- | ----- | -------------- |
| 3D Viewport | Centre | Where you see and edit objects |
| Outliner | Top right | A list of every object; click to select, double-click to rename |
| Properties editor | Bottom right | Tabs down its left edge: Scene, Object, Modifiers, Material, … |
| Sidebar | Press `N` in the viewport | The **Item** tab shows an object's exact location, rotation, scale and dimensions |
| Redo panel | Bottom left, after an operation | Adjust the last operation (size, vertex count, …) after the fact. Also `F9` |

### 1.2 Moving around the viewport

| Action | Mouse / trackpad | Keyboard |
| ------ | ---------------- | -------- |
| Orbit | Middle-drag, or two-finger drag on a trackpad | Numpad `4 6 8 2` |
| Pan | `Shift` + middle-drag, or `Shift` + two-finger drag | `Ctrl` + numpad |
| Zoom | Scroll, or pinch | Numpad `+ −` |
| Frame the selection | | `Numpad .` (or View → Frame Selected) |
| Front / right / top view | | Numpad `1` / `3` / `7` |

No numpad on a MacBook: **Edit → Preferences → Input**, tick **Emulate
Numpad** (the number row acts as a numpad) and, on a trackpad, **Emulate
3 Button Mouse** (`Option` + drag orbits).

The coloured X/Y/Z gizmo at the top right of the viewport can be clicked
to snap to a view, and dragged to orbit.

### 1.3 Modes

`Tab` toggles:

- **Object Mode:** move whole objects.
- **Edit Mode:** move an object's vertices, edges and faces. `1` / `2` /
  `3` on the number row selects vertices / edges / faces.

If a key does nothing, check the mode (drop-down at the top left of the
viewport).

### 1.4 The core keys

The mouse pointer must be over the viewport.

| Key | Does |
| --- | ---- |
| `Shift` + `A` | Add an object |
| `G` | Grab (move). Then `X` / `Y` / `Z` to lock an axis, type a number, `Enter` |
| `R` | Rotate. `R` `X` `90` `Enter` rotates 90° about X |
| `S` | Scale. `S` `Z` `0.5` `Enter` halves the height |
| `Shift` + `D` | Duplicate (then right-click to leave the copy in place) |
| `X` or `Delete` | Delete the selection |
| `A` / `Option` + `A` | Select all / nothing |
| `Ctrl` + `J` | Join the selected objects into the active one |
| `Ctrl` + `Z` | Undo |
| `Ctrl` + `S` | Save |

Type exact numbers rather than dragging: `G Z 0.35 Enter`, not a drag
that lands near 0.35.

---

## Part 2. One-time setup

### 2.1 A start file with metric units

1. Delete the default cube, camera and light (`A`, then `X`).
2. **Properties → Scene tab (cone-and-sphere icon) → Units:** Unit
   System **Metric**, Unit Scale **1.0**, Length **Meters**, Mass
   **Kilograms**.
3. **File → Save As** `assets_src/start.blend` in the Helios repo.

Start every asset with **File → Save As** from this file.

Unit Scale must stay 1.0. The glTF exporter ignores it and writes raw
Blender units, so a scene at scale 0.01 that *shows* a 1 cm cube would
reach Helios as a 1 m cube. The export script refuses such a file.

### 2.2 Axes: Blender is already ENU

| Blender | Helios world (ENU) | Helios body (FLU) |
| ------- | ------------------ | ----------------- |
| +X (red) | East | Forward |
| +Y (green) | North | Left |
| +Z (blue) | Up | Up |

Model every object with its **front facing +X** and its **top facing
+Z**. In top view (numpad `7`), +X points right, so a car seen from above
points right. You never swap axes by hand: the exporter and Helios agree
on the conversion.

### 2.3 Where files go

| File | Location |
| ---- | -------- |
| `.blend` (what you edit) | `assets_src/objects/<asset>.blend` |
| `.glb` (what Helios loads; written by the script) | `helios_sim/assets/objects/<asset>.glb` |
| Prefab TOML (what the thing *is*) | `configs/entities/objects/<asset>.toml` |

Commit the `.blend` and the `.glb` together. The `.glb` is committed so
the sim runs without Blender installed. A pre-commit hook refuses a
`.blend` staged without its `.glb`; install it once per clone with
`git config core.hooksPath tools/git-hooks`.

---

## Part 3. Asset 1: a traffic cone

Target: a 0.70 m cone on a 0.38 m square base, standing on the origin.

### 3.1 The cone body

1. From `start.blend`, **File → Save As**
   `assets_src/objects/traffic_cone.blend`.
2. `Shift` + `C` puts the 3D cursor (where new objects appear) at the
   origin.
3. `Shift` + `A` → **Mesh → Cone**. In the redo panel:
   - Vertices **32**
   - Radius 1 **0.16** (bottom), Radius 2 **0.025** (top)
   - Depth **0.67**
   - Location Z **0.365** (the cone is centred on its location, so its
     bottom lands at 0.03 m, on top of the base, and its tip at 0.70 m)

### 3.2 The base

1. `Shift` + `A` → **Mesh → Cube**. Redo panel: Size **1**, Location Z
   **0.015**.
2. `S X 0.38 Enter`, `S Y 0.38 Enter`, `S Z 0.03 Enter`.

`N` → **Item** → Dimensions should read 0.38, 0.38, 0.03.

### 3.3 Colours

1. Select the cone. **Properties → Material tab (red sphere) → New.**
   Base Color orange.
2. Select the base, **New**, Base Color near black.
3. See the colours with **Material Preview** shading (third sphere icon
   at the top right of the viewport).

Use only Base Color, Metallic and Roughness. Procedural node setups
don't survive export.

### 3.4 Join the visual pieces

Click the base, `Shift`-click the cone, `Ctrl` + `J`: one object, both
colours. The object clicked last is the one the others join into, so
clicking the cone last keeps its clean scale of 1 (the base's is
0.38 × 0.38 × 0.03). Rename it `traffic_cone` (double-click in the Outliner). The
object name is for your own clarity; Helios names the asset after the
file.

Right-click it → **Shade Auto Smooth** so the sides render smooth.

### 3.5 Collider parts

With no collider parts, Helios gives an asset one box the size of its
bounding box. That box is far too wide at the cone's tip (Helios refuses
it: the cone fills a quarter of its box), so the cone gets collider
parts.

A **collider part** is any object whose name starts with `col_`. Helios
doesn't render it: it wraps it in its *convex hull* (the shape you'd get
by shrink-wrapping it) and uses that for collisions. Keep parts simple:
tens of vertices.

**Every part must be convex**, no dents. The hull fills a dent, so a
concave part collides where nothing is drawn, and Helios refuses it,
naming the part and the dent's depth. The cone shows the trap: one part
copied from the whole cone hulls into a square pyramid from the base's
corners to the tip, filling the air round the cone (a 4.6 cm dent). So it
gets two parts, base and cone, each convex:

1. Select `traffic_cone`, `Shift` + `D`, right-click (the copy stays in
   place). Rename the copy `col_base`.
2. `Tab` into Edit Mode, hover over the cone, `L` (selects the cone's
   vertices only), `X` → **Vertices**, `Tab`. Only the base is left.
3. Select `traffic_cone` again, `Shift` + `D`, right-click, rename
   `col_cone`; this time hover over the base, `L`, `X` → **Vertices**.
4. Optional, to tell them apart: **Properties → Object tab (orange
   square) → Viewport Display → Display As: Wire.**

If a rougher shape is good enough, say so in the geometry instead: in
Edit Mode, select all and **Mesh → Convex Hull**. The part becomes its
own hull, so it is convex by construction. How closely the collider
follows the mesh is your choice; the checks only catch a part that isn't
what you modelled. An L-shaped desk gets several boxes: `col_left`,
`col_right`, …

Parts must also stay within the visible mesh's box (to within 1 mm):
a part dragged or scaled away from the mesh is refused too.

Hiding an object (the eye icon, `H`) only changes your view: hidden
objects are still exported, so hidden parts still work. The flip side:
anything you hid to get it out of the way is exported too. Delete what
isn't part of the asset.

### 3.6 Save and export

`Ctrl` + `S`, then Part 5.

---

## Part 4. Good habits (optional)

None of these is required; Helios reads the geometry correctly either
way. They keep files easy to work with.

- **Face orientation.** Viewport **Overlays** drop-down → **Face
  Orientation**: outside faces should be blue. A red face points inward
  and renders wrongly: `Tab`, `A`, **Mesh → Normals → Recalculate
  Outside**, `Tab`.
- **Triangle count.** **Overlays → Statistics**. Props should be in the
  hundreds to low thousands; the cone is about 200.
- **Tidy transforms.** `Ctrl` + `A` → **All Transforms** makes Scale read
  1 and Rotation 0, so the Item panel shows the real size.
- **No mirroring by negative scale.** `S X -1` turns an object inside
  out, and Helios rejects it. Mirror in Edit Mode (**Mesh → Mirror**)
  instead.

---

## Part 5. Export

From the repo root:

```
/Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
    --python-exit-code 1 --python tools/blender/export_assets.py -- \
    assets_src/objects/traffic_cone.blend
```

With no file after `--`, it exports every `.blend` in
`assets_src/objects/`. It writes `helios_sim/assets/objects/<asset>.glb`,
refuses a file with the wrong units or no visual mesh, and never modifies
the `.blend`. A refused file is reported and the rest still export; the
command then ends with an error naming every refused asset, and exits
non-zero.

### 5.1 Check the first export

Open the result once to see what Helios gets: **File → New → General**,
delete everything, **File → Import → glTF 2.0**, pick the `.glb`. It
should stand upright on the origin at the right size, with `col_base`
and `col_cone` in the same place. Close without saving.

---

## Part 6. The first world's pieces: crate, wall, ground

Plain boxes, so no collider parts: the default box is exact. Their known
sizes also make them good test fixtures (a wall 10 m away should read
10 m on the lidar). For each: **File → Save As** from `start.blend`,
`Shift` + `C`, build, save, export.

### 6.1 Crate, 1 m (`crate_1m`)

1. `Shift` + `A` → Cube. Size **1**, Location Z **0.5**.
2. Optional rounded edges: **Properties → Modifiers tab (wrench) → Add
   Modifier → Generate → Bevel**, Amount **0.02**, Segments **2**. The
   script applies modifiers on export.
3. Material: wood brown.

**Acceptance check:** in Helios the crate's bounding box must be exactly
1 × 1 × 1 m. That confirms units survive the whole pipeline.

### 6.2 Wall segment, 4 m (`wall_4m`)

4 m long (X), 0.2 m thick (Y), 2 m tall (Z).

1. `Shift` + `A` → Cube. Size **1**, Location Z **1.0**.
2. `S X 4 Enter`, `S Y 0.2 Enter`, `S Z 2 Enter`.
3. Material: concrete grey.

Longer walls: several segments, or a placement scale along X. Stretching
is fine for a plain box; it would distort detail such as a door frame.

### 6.3 Ground slab, 100 m (`ground_100m`)

100 × 100 m, **1 m thick**, top face at z = 0.

A flat plane won't do: it has zero thickness, so it can't size a box
collider (Helios rejects it), and a paper-thin collider lets falling
objects pass through.

1. `Shift` + `A` → Cube. Size **1**, Location Z **−0.5** (top face at 0).
2. `S X 100 Enter`, `S Y 100 Enter`.
3. Material: mid grey.

The ground's *top* is at z = 0 rather than its bottom. Helios accepts
either: the rule is that the surface an object rests on, or that things
rest on, is at z = 0.

---

## Part 7. A low-poly parked car (`sedan`)

About 4.5 m long, 1.8 m wide, 1.5 m tall, nose at +X, wheels on the grid.

### 7.1 Body

`Shift` + `A` → Cube. Size **1**, Location Z **0.65**. Then `S X 4.5`,
`S Y 1.8`, `S Z 0.7`, each followed by `Enter`. The body spans 0.3–1.0 m
above the ground.

### 7.2 Cabin

`Shift` + `A` → Cube. Size **1**, Location X **−0.3**, Z **1.25**. Then
`S X 2.4`, `S Y 1.6`, `S Z 0.5`. Optional taper: `Tab`, `3` (face
select), click the top face, `S X 0.8 Enter`, `Tab`.

### 7.3 Wheels

1. `Shift` + `A` → **Mesh → Cylinder**. Vertices **24**, Radius
   **0.33**, Depth **0.22**, Rotation X **90°**, Location X **1.4**, Y
   **0.8**, Z **0.33**: the front-left wheel, touching the grid.
2. `Shift` + `D`, `Y −1.6 Enter` (front right). Select both front
   wheels, `Shift` + `D`, `X −2.8 Enter` (the rears).

A parked car's wheels are visual only.

### 7.4 Finish

1. Materials: paint for body and cabin (optionally a glass material on
   the window faces: Edit Mode, select the faces, **Assign** in the
   Material tab), dark grey for the wheels.
2. Select everything, `Ctrl` + `J`.
3. Top view (numpad `7`): the bonnet points right.
4. Save, export.

No collider part: the default box around the whole car suits a parked
car. A pickup with an open bed would get `col_` parts.

**Acceptance check:** in Helios the sedan's nose must point east. It is
the one asymmetric asset, so it is the one that can reveal an axis
mistake.

---

## Part 8. What Helios reads, and what it checks

| From the `.glb` | Used for |
| --------------- | -------- |
| All non-`col_` meshes and their materials | Rendering |
| Their tight bounding box, computed at load | Ground-truth box labels, and the default collider |
| `col_` meshes | The collider: one convex hull each. With none, one box the size of the bounding box |

| Refused at load | Why |
| --------------- | --- |
| No visual mesh: every mesh is a `col_` part | Nothing to see or label |
| A part named `Col_…` or `COL_…` | It would silently count as visual; the prefix must be lowercase `col_` |
| A negative scale anywhere | The mesh is turned inside out |
| A `col_` part that is flat (a plane, a line) | It has no volume |
| A concave `col_` part (a dent deeper than 1 mm) | Its hull fills the dent, so it collides where nothing is drawn |
| A `col_` part reaching more than 1 mm past the visible mesh's box | Moved or scaled away from its mesh |
| No `col_` parts and a flat box | The box can't be a collider |
| No `col_` parts and the mesh fills under 90% of its box | The box would be a poor collider (a cone, a cylinder); add `col_` parts |
| Geometry stored outside the `.glb`, or a malformed file | Re-export with the script |

Every refusal names the asset and, where it applies, the part. A scenario
with a refused asset does not start, and every refusal is listed at once.

Each asset that loads gets one line in the log, for example
`0.38 x 0.38 x 0.72 m, base at z 0 mm, 2 hulls`. A wrong size, a base
that isn't at 0 mm (the asset floats or sinks), or the wrong number of
hulls shows there before a run.

Not read from the file: class, mass, whether it collides. Those are in
`configs/entities/objects/<asset>.toml`, so changing them never needs a
re-export. Static or dynamic, and where the asset goes, is chosen by each
placement in a world file.

## Common problems

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| Script refuses: unit scale | Unit Scale isn't 1.0 | Part 2.1 |
| Log line shows the base away from 0 mm | Not standing on the grid | Move it so its bottom (the ground: its top) is at z = 0 |
| Log line shows the wrong size | A stray object in the file, or wrong units | Delete strays; Part 2.1 |
| Refused: fills too little of its box | A non-box shape with no collider parts | Add `col_` parts (Part 3.5) |
| Refused: part is concave | One part where several convex ones are needed | Split it, or **Mesh → Convex Hull** (Part 3.5) |
| Car faces the wrong way | Modelled facing +Y or −X | `R Z` to face +X |
| Patches invisible or black | Flipped normals | Part 4, face orientation |
| Colours missing | Procedural material nodes | Base Color only |
| Lidar sees through an object | Missing faces (open mesh) | Close the mesh in Edit Mode |
