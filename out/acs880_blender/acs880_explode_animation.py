"""ACS880-01 disassemble / reassemble animation for Blender.

Works from the command line, from the Scripting tab, or pasted into Blender MCP's
`execute_blender_code`. Edit the CONFIG block, then run.

The ABB STEP files are one "SHRINKWRAP" product with anonymous shells, so there are no real part
names. The script groups shells by size and proximity, which gives believable plates and covers
that stay together, but the groups are NOT ABB's real component names.

    blender --background --python acs880_explode_animation.py -- --obj ACS880-01_R4_P940.obj --out /tmp/acs880
"""
import math
import os
import sys

import bpy
from mathutils import Vector

# ----------------------------------------------------------------------------- CONFIG
CONFIG = dict(
    obj_path="",            # OBJ exported from the STEP file (metres). Empty = use meshes already in the scene.
    out_dir="",             # if set: save the .blend here and render the PNG frames listed in STILLS
    fps=24,
    hold_start=24,          # frames fully assembled at the start
    disassemble=96,         # frames to take it apart
    hold_open=36,           # frames held exploded
    assemble=96,            # frames to put it back together
    hold_end=24,            # frames fully assembled at the end
    explode_scale=0.55,     # explode distance relative to model size
    stagger=0.55,           # 0 = everything moves together, 1 = strictly one part after another
    orbit_degrees=140,      # camera orbit over the whole animation (0 = static camera)
    anchor_diag=0.055,      # shells with a bbox diagonal above this fraction of the model size become own parts
    anchor_tris=1500,       # ...or with at least this many triangles
    resolution=(1920, 1080),
    samples=48,
    engine="AUTO",          # AUTO = EEVEE if available, else Cycles. Or "CYCLES" / "BLENDER_EEVEE".
    video="",               # if set: render the whole animation to this .mp4 (needs out_dir)
    stills=(),              # frames to render as PNG when out_dir is set, e.g. (1, 90, 150, 210)
)
# -----------------------------------------------------------------------------


def parse_argv():
    if "--" not in sys.argv:
        return
    a = sys.argv[sys.argv.index("--") + 1:]
    it = iter(a)
    for k in it:
        v = next(it, None)
        if k == "--obj":
            CONFIG["obj_path"] = v
        elif k == "--out":
            CONFIG["out_dir"] = v
        elif k == "--stills":
            CONFIG["stills"] = tuple(int(x) for x in v.split(","))
        elif k == "--video":
            CONFIG["video"] = v
        elif k == "--engine":
            CONFIG["engine"] = v
        elif k == "--samples":
            CONFIG["samples"] = int(v)
        elif k == "--res":
            w, h = v.split("x")
            CONFIG["resolution"] = (int(w), int(h))


def ease(t):
    return t * t * (3 - 2 * t)


def import_obj(path):
    before = set(bpy.data.objects)
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
    else:
        bpy.ops.import_scene.obj(filepath=path, axis_forward="Y", axis_up="Z")
    return [o for o in bpy.data.objects if o not in before and o.type == "MESH"]


def world_bbox(o):
    pts = [o.matrix_world @ Vector(c) for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return lo, hi


def point_box_dist(p, lo, hi):
    d = Vector((max(lo.x - p.x, 0, p.x - hi.x), max(lo.y - p.y, 0, p.y - hi.y), max(lo.z - p.z, 0, p.z - hi.z)))
    return d.length


def group_shells(meshes, model_diag):
    info = {}
    for o in meshes:
        lo, hi = world_bbox(o)
        info[o] = (lo, hi, (lo + hi) / 2, (hi - lo).length, len(o.data.polygons))
    anchors = [o for o in meshes
               if info[o][3] > CONFIG["anchor_diag"] * model_diag or info[o][4] >= CONFIG["anchor_tris"]]
    if not anchors:
        anchors = [max(meshes, key=lambda o: info[o][4])]
    groups = {a: [a] for a in anchors}
    for o in meshes:
        if o in groups:
            continue
        c = info[o][2]
        best = min(anchors, key=lambda a: point_box_dist(c, info[a][0], info[a][1]) + 1e-4 * (info[a][1] - info[a][0]).length)
        groups[best].append(o)
    return groups, info


def build_parts(groups, info):
    parts = []
    for i, (anchor, members) in enumerate(sorted(groups.items(), key=lambda kv: kv[0].name)):
        lo = Vector((min(info[m][0].x for m in members), min(info[m][0].y for m in members), min(info[m][0].z for m in members)))
        hi = Vector((max(info[m][1].x for m in members), max(info[m][1].y for m in members), max(info[m][1].z for m in members)))
        centre = (lo + hi) / 2
        empty = bpy.data.objects.new("Part_%02d" % i, None)
        empty.empty_display_type = "PLAIN_AXES"
        empty.empty_display_size = 0.01
        empty.location = centre
        bpy.context.scene.collection.objects.link(empty)
        bpy.context.view_layer.update()
        for m in members:
            m.parent = empty
            m.matrix_parent_inverse = empty.matrix_world.inverted()
        parts.append(dict(empty=empty, centre=centre, members=members, size=(hi - lo)))
    return parts


def plan_explosion(parts, model_centre, model_diag):
    """Assign each part an offset and a start/end slot. Outermost parts leave first and return last."""
    for p in parts:
        d = p["centre"] - model_centre
        radial = Vector((d.x, d.y, 0.0))
        # Stack is mostly along Z (base -> cover); push along Z by layer, plus a radial fan-out.
        z_push = (d.z / max(model_diag, 1e-6)) * 2.2
        offset = Vector((radial.x * 1.1, radial.y * 0.75, 0.0)) + Vector((0, 0, z_push * model_diag * 0.55))
        if offset.length < 0.05 * model_diag:
            offset += Vector((0, 0, 0.18 * model_diag))
        p["offset"] = offset * CONFIG["explode_scale"] / max(offset.length / (0.45 * model_diag), 0.6)
        p["rank_key"] = p["centre"].z + 0.35 * radial.length
    order = sorted(parts, key=lambda p: -p["rank_key"])
    n = max(len(order) - 1, 1)
    for i, p in enumerate(order):
        p["slot"] = i / n


def keyframe(parts):
    sc = bpy.context.scene
    c = CONFIG
    f0 = 1
    f1 = f0 + c["hold_start"]
    f2 = f1 + c["disassemble"]
    f3 = f2 + c["hold_open"]
    f4 = f3 + c["assemble"]
    f5 = f4 + c["hold_end"]
    sc.frame_start, sc.frame_end = f0, f5
    for p in parts:
        e = p["empty"]
        home = e.location.copy()
        away = home + p["offset"]
        s = c["stagger"]
        span = 1.0 - s
        # outermost (slot 0) starts first on the way out, and starts last on the way back in
        out_start = f1 + p["slot"] * s * c["disassemble"]
        out_end = out_start + span * c["disassemble"]
        in_start = f3 + (1 - p["slot"]) * s * c["assemble"]
        in_end = in_start + span * c["assemble"]
        for fr, loc in ((f0, home), (out_start, home), (out_end, away), (in_start, away), (in_end, home), (f5, home)):
            e.location = loc
            e.keyframe_insert("location", frame=int(round(fr)))
        if e.animation_data and e.animation_data.action:
            for fc in _fcurves(e.animation_data.action):
                for kp in fc.keyframe_points:
                    kp.interpolation = "BEZIER"
                    kp.handle_left_type = kp.handle_right_type = "AUTO_CLAMPED"
    return f0, f5


def _fcurves(action):
    # Blender 5 moved fcurves into layered actions; older versions expose action.fcurves.
    if hasattr(action, "fcurves"):
        return list(action.fcurves)
    out = []
    for layer in getattr(action, "layers", []):
        for strip in layer.strips:
            for bag in getattr(strip, "channelbags", []):
                out.extend(bag.fcurves)
    return out


def materials(parts, model_centre, model_diag):
    palette = [(0.62, 0.64, 0.68), (0.82, 0.83, 0.86), (0.30, 0.32, 0.36), (0.86, 0.12, 0.10), (0.18, 0.30, 0.55), (0.75, 0.55, 0.20)]
    mats = []
    for i, col in enumerate(palette):
        m = bpy.data.materials.new("ACS_mat_%d" % i)
        m.use_nodes = True
        b = m.node_tree.nodes.get("Principled BSDF")
        b.inputs["Base Color"].default_value = (*col, 1)
        b.inputs["Metallic"].default_value = 0.65 if i != 3 else 0.2
        b.inputs["Roughness"].default_value = 0.38
        mats.append(m)
    for i, p in enumerate(parts):
        big = p["size"].length > 0.35 * model_diag
        m = mats[0 if big else (i % (len(palette) - 1)) + 1]
        for o in p["members"]:
            o.data.materials.clear()
            o.data.materials.append(m)
            for poly in o.data.polygons:
                poly.use_smooth = True


def stage(model_centre, model_diag, f0, f5):
    sc = bpy.context.scene
    sc.render.fps = CONFIG["fps"]
    sc.render.resolution_x, sc.render.resolution_y = CONFIG["resolution"]
    engines = ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE", "CYCLES") if CONFIG["engine"] == "AUTO" else (CONFIG["engine"],)
    for eng in engines:
        try:
            sc.render.engine = eng
            break
        except TypeError:
            continue
    if sc.render.engine == "CYCLES":
        sc.cycles.samples = CONFIG["samples"]
        sc.cycles.use_denoising = True
    try:
        sc.view_settings.view_transform = "Standard"
    except TypeError:
        pass
    world = sc.world or bpy.data.worlds.new("World")
    sc.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.045, 0.05, 0.06, 1)
    bg.inputs["Strength"].default_value = 0.5

    target = bpy.data.objects.new("CamTarget", None)
    target.location = model_centre + Vector((0, 0, 0.04 * model_diag))
    sc.collection.objects.link(target)
    pivot = bpy.data.objects.new("CamPivot", None)
    pivot.location = target.location
    sc.collection.objects.link(pivot)
    cam_data = bpy.data.cameras.new("Cam")
    cam_data.lens = 42
    cam = bpy.data.objects.new("Camera", cam_data)
    cam.parent = pivot
    cam.location = Vector((0.55, -1.35, 0.78)).normalized() * model_diag * 2.9
    sc.collection.objects.link(cam)
    tc = cam.constraints.new("TRACK_TO")
    tc.target, tc.track_axis, tc.up_axis = target, "TRACK_NEGATIVE_Z", "UP_Y"
    sc.camera = cam
    pivot.rotation_euler = (0, 0, math.radians(-CONFIG["orbit_degrees"] / 2))
    pivot.keyframe_insert("rotation_euler", frame=f0)
    pivot.rotation_euler = (0, 0, math.radians(CONFIG["orbit_degrees"] / 2))
    pivot.keyframe_insert("rotation_euler", frame=f5)

    def light(name, kind, loc, energy):
        d = bpy.data.lights.new(name, kind)
        d.energy = energy
        o = bpy.data.objects.new(name, d)
        o.location = loc
        sc.collection.objects.link(o)
        t = o.constraints.new("TRACK_TO")
        t.target, t.track_axis, t.up_axis = target, "TRACK_NEGATIVE_Z", "UP_Y"
        return d

    k = light("Key", "AREA", Vector((1.2, -1.4, 1.6)) * model_diag, 60 * (model_diag / 0.7) ** 2)
    k.size = model_diag * 1.2
    r = light("Rim", "AREA", Vector((-1.5, 1.2, 1.0)) * model_diag, 30 * (model_diag / 0.7) ** 2)
    r.size = model_diag
    f = light("Fill", "AREA", Vector((-1.2, -1.0, 0.5)) * model_diag, 14 * (model_diag / 0.7) ** 2)
    f.size = model_diag * 1.5


def main():
    parse_argv()
    c = CONFIG
    if c["obj_path"]:
        for o in list(bpy.data.objects):
            bpy.data.objects.remove(o, do_unlink=True)
        meshes = import_obj(c["obj_path"])
    else:
        meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not meshes:
        raise SystemExit("No mesh objects found: set CONFIG['obj_path'] or import the model first.")
    bpy.context.view_layer.update()

    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for o in meshes:
        l, h = world_bbox(o)
        lo = Vector((min(lo.x, l.x), min(lo.y, l.y), min(lo.z, l.z)))
        hi = Vector((max(hi.x, h.x), max(hi.y, h.y), max(hi.z, h.z)))
    model_centre, model_diag = (lo + hi) / 2, (hi - lo).length

    groups, info = group_shells(meshes, model_diag)
    parts = build_parts(groups, info)
    print("shells: %d -> parts: %d" % (len(meshes), len(parts)))
    plan_explosion(parts, model_centre, model_diag)
    materials(parts, model_centre, model_diag)
    f0, f5 = keyframe(parts)
    stage(model_centre, model_diag, f0, f5)
    bpy.context.scene.frame_set(f0)

    if c["out_dir"]:
        os.makedirs(c["out_dir"], exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=os.path.join(c["out_dir"], "acs880_explode.blend"))
        if c["video"]:
            r = bpy.context.scene.render
            try:
                r.image_settings.file_format = "FFMPEG"
                r.ffmpeg.format, r.ffmpeg.codec = "MPEG4", "H264"
                r.ffmpeg.constant_rate_factor = "HIGH"
                r.filepath = os.path.join(c["out_dir"], c["video"])
            except TypeError:  # headless pip `bpy` builds ship without FFMPEG: write a PNG sequence instead
                r.image_settings.file_format = "PNG"
                os.makedirs(os.path.join(c["out_dir"], "frames"), exist_ok=True)
                r.filepath = os.path.join(c["out_dir"], "frames", "f_")
            bpy.ops.render.render(animation=True)
        for fr in c["stills"]:
            bpy.context.scene.frame_set(fr)
            bpy.context.scene.render.filepath = os.path.join(c["out_dir"], "frame_%04d.png" % fr)
            bpy.ops.render.render(write_still=True)


main()
