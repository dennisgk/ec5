"""Run with Blender --background BrachiFlex_CAN_Aligned.blend --python this_file.

Only the working copy is modified. The application ships the generated GLB.
"""
from pathlib import Path
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parent
bpy.context.preferences.filepaths.save_version = 0
OUTPUT = ROOT.parent / "exoskeleton_controller_mobile" / "assets" / "brachiflex.glb"
OUTPUT.parent.mkdir(parents=True, exist_ok=True)

for obj in list(bpy.data.objects):
    if "strap" in obj.name.lower():
        bpy.data.objects.remove(obj, do_unlink=True)

fixed = []
moving = []
for obj in list(bpy.context.scene.objects):
    if obj.type != "MESH":
        continue
    if any(collection.name.startswith("09 ") for collection in obj.users_collection):
        continue
    # Retain assembly geometry; omit sub-3 mm electronic and screw detail.
    if max(obj.dimensions) < 3:
        continue
    if obj.hide_render:
        continue
    target = moving if obj.name.startswith(("Lower brace", "Elbow sprocket")) else fixed
    target.append(obj)

def join(objects, name):
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.hide_set(False)
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.convert(target="MESH")
    bpy.ops.object.join()
    obj = bpy.context.object
    obj.name = name
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if len(obj.data.polygons) > 50000:
        modifier = obj.modifiers.new("Mobile mesh reduction", "DECIMATE")
        modifier.ratio = 50000 / len(obj.data.polygons)
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    return obj

fixed_mesh = join(fixed, "FixedAssembly")
moving_mesh = join(moving, "MovingForearm")
pivot = bpy.data.objects.new("ElbowPivot", None)
bpy.context.collection.objects.link(pivot)
pivot.location = Vector((5.517, 7.977, 30.2))

bpy.ops.object.select_all(action="DESELECT")
for obj in (fixed_mesh, moving_mesh, pivot):
    obj.select_set(True)

bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "BrachiFlex_Mobile.blend"))
bpy.ops.export_scene.gltf(
    filepath=str(OUTPUT), export_format="GLB", use_selection=True,
    export_yup=True, export_materials="EXPORT", export_cameras=False,
    export_lights=False, export_animations=False,
)
print("EC5_EXPORT", OUTPUT, OUTPUT.stat().st_size)
