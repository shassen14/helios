"""Export each asset .blend in assets_src/objects/ to a .glb in
helios_sim/assets/objects/, with every export setting fixed here.

Runs inside Blender, not with a system Python:

    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \\
        --python-exit-code 1 --python tools/blender/export_assets.py -- \\
        [files...]

With no files after "--", exports every .blend in assets_src/objects/.
"""

import math
import sys
from pathlib import Path

import bpy

SOURCE_DIR = "assets_src/objects"
OUTPUT_DIR = "helios_sim/assets/objects"
RESERVED_PREFIXES = ("col_", "sensor_")
# Every export option Helios depends on, set explicitly so a Blender
# upgrade that changes a default can't change the output. Blender rejects
# a misspelt key. Differ from the 5.1 defaults: export_apply,
# use_active_scene, export_animations.
EXPORT_SETTINGS = {
    "export_format": "GLB",
    "export_yup": True,
    "export_apply": True,
    "use_selection": False,
    "use_active_scene": True,
    "export_cameras": False,
    "export_lights": False,
    "export_animations": False,
    "export_extras": False,
    "export_materials": "EXPORT",
    "export_texcoords": True,
    "export_normals": True,
    "export_draco_mesh_compression_enable": False,
}


def repo_root():
    p = Path(__file__).resolve()
    return p.parents[2]


def source_dir():
    return repo_root() / SOURCE_DIR


def output_dir():
    return repo_root() / OUTPUT_DIR


def get_args(args):
    """Return the arguments after "--" in `args`, or an empty list when
    there is no "--". Blender keeps its own flags in sys.argv; everything
    after "--" belongs to this script."""
    after_separator = False

    arg_list = []

    for arg in args:
        if after_separator:
            arg_list.append(arg)

        if arg == "--":
            after_separator = True

    return arg_list


def collect_blend_files(arg_list) -> list[Path]:
    """Return the absolute paths of the .blend files to export.

    With no arguments, every .blend directly inside the source folder,
    sorted. Otherwise each argument, which must be an existing .blend
    directly inside the source folder: the asset name comes from the file
    name, so a subfolder could hold a second file writing the same .glb.
    Raises if a file is refused or none are found."""
    paths = []

    if len(arg_list) == 0:
        all_blend = source_dir().glob("*.blend")
        paths = sorted(all_blend)
    else:
        for arg in arg_list:
            path = Path(arg).resolve()

            if not path.is_file():
                raise ValueError(f"{path} does not exist")

            if path.suffix != ".blend":
                raise ValueError(f"{path} is not a .blend file")

            if path.parent != source_dir():
                raise ValueError(f"{path} must be directly inside {source_dir()}")

            paths.append(path)

    if len(paths) == 0:
        raise ValueError(f"no .blend files found in {source_dir()}")

    return paths


def check_scene() -> list[str]:
    """Return the problems with the open file that its .glb can't show,
    or an empty list when it can be exported.

    The exporter ignores the unit scale, so a wrong scale would export at
    the wrong size unnoticed. Every other rule is checked when the .glb is
    loaded."""
    units = bpy.context.scene.unit_settings
    problems = []

    if units.system != "METRIC":
        problems.append(
            f"unit system is {units.system}, must be METRIC "
            "(Properties → Scene → Units → Unit System)"
        )

    if not math.isclose(units.scale_length, 1.0):
        problems.append(
            f"unit scale is {units.scale_length}, must be 1.0 "
            "(Properties → Scene → Units → Unit Scale)"
        )

    mesh_names = []
    has_visual = False

    for obj in bpy.context.scene.objects:
        if obj.type == "MESH":
            mesh_names.append(obj.name)

            if not obj.name.startswith(RESERVED_PREFIXES):
                has_visual = True

    if not mesh_names:
        problems.append("no meshes")
    elif not has_visual:
        problems.append(
            "no visual mesh: every mesh is a collider or sensor part "
            f"({', '.join(mesh_names)})"
        )

    return problems


def export_scene(output_path):
    """Write the open file's active scene to `output_path` as a .glb.
    Creates missing folders; raises RuntimeError if the export fails."""
    bpy.ops.export_scene.gltf(filepath=str(output_path), **EXPORT_SETTINGS)


def export_file(path) -> list[str]:
    """Open `path`, check it, and export it if the checks pass. Return its
    problems, or an empty list when it was exported."""
    try:
        bpy.ops.wm.open_mainfile(filepath=str(path))
    except RuntimeError as err:
        return [f"could not open: {err}"]

    problems = check_scene()
    if len(problems) > 0:
        return problems

    output_path = output_dir() / f"{path.stem}.glb"
    try:
        export_scene(output_path)
    except RuntimeError as err:
        return [f"export failed: {err}"]

    print(f"{path.stem}: ok -> {output_path.relative_to(repo_root())}")
    return []


def main():
    """Export every requested file, reporting each one, and raise at the
    end if any failed so the command exits non-zero. One bad file never
    stops the others."""
    arg_list = get_args(sys.argv)
    print(bpy.app.version_string)

    paths = collect_blend_files(arg_list)
    failed = []

    for path in paths:
        problems = export_file(path)

        if len(problems) > 0:
            failed.append(path.stem)
            print(f"{path.stem}: FAIL")
            for problem in problems:
                print(f"  {problem}")

    if len(failed) > 0:
        raise RuntimeError(f"{len(failed)} asset(s) failed: {', '.join(failed)}")


if __name__ == "__main__":
    main()
