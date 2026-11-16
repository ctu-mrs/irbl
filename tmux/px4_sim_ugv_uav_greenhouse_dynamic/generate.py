#!/usr/bin/env python3
"""Generate worlds/greenhouse_dynamic.sdf: the greenhouse of ../px4_sim_ugv_uav_greenhouse with moving plants
and people walking around.

  ./generate.py                 # -> worlds/greenhouse_dynamic.sdf + worlds/models/greenhouse_crops_dynamic/
  ./generate.py --seed 3        # other plants / crates
  ./generate.py --no-operators  # wind only

Same layout as worlds/worldgen/worlds/greenhouse.py of ../px4_sim_1_uav_3dlidar (rows, aisles, gutters,
posts, crates, collision boxes, plant models: imported from it), plus:

  - Wind-blown foliage. The plants of each row segment are cut into CHUNK-long groups, each a few visuals
    of the (static) row model:
        sway_<id>          stems + fruit, origin at the stem base on the gutter (the pivot of the sway)
        flutter_<id>_<k>   leaves, one group per height band and aisle side, origin at its centroid
    The gz system plugin gz_greenhouse_dynamics::PlantSway (ros_packages/gz_greenhouse_dynamics, loaded by
    config/gz_server.config, wind settings there) moves these visuals: gusts sweep across the greenhouse,
    each plant group sways at its own frequency, the leaves flutter. Visual only: cameras and gpu_lidars
    see it, the collision boxes (physics, the planners' prior map) are the static ones of greenhouse.py.
    Strawberries (low, on raised gutters) only flutter.
  - Operators: gz <actor>s (walking / standing person, worlds/models/greenhouse_operator) on looping
    routes along the aisle centrelines (OPERATORS below), stopping now and then in front of a crop row.
    Not in the prior map (the mission parses <model>s only) and without collision: like the crates, only
    the onboard sensors see them -- rbl_controller has to avoid them from its point cloud.
"""
import argparse
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "px4_sim_1_uav_3dlidar", "worlds"))
from worldgen.sdf import FOOTER, box, header, model  # noqa: E402
from worldgen.worlds import greenhouse as gh  # noqa: E402

WORLD_NAME = "greenhouse_dynamic"
MESH_MODEL = "greenhouse_crops_dynamic"
OPERATOR_MODEL = "greenhouse_operator"

CHUNK = 2.0                 # [m] plant group length along the row (one sway pivot)
LEAF_BANDS = {"tomato": 2, "cucumber": 2, "pepper": 2, "strawberry": 1}  # leaf groups over the height
LEAF_MATS = {"leaf", "leaf2", "leaf_dark"}

# ---- fruit ground truth + colour segmentation (crop_scanner.py) -------------------------------------------------
# In greenhouse.py the green fruits have the leaves' / stems' colour (cucumber hue 116 deg, green pepper 107 deg,
# leaves 94..113 deg): invisible to colour segmentation. Here they get their own (still realistic) greens; in the
# render, shading scales brightness only, so hue / saturation identify a material.
MATERIAL_OVERRIDES = {
    "cucumber": (0.08, 0.30, 0.22),       # bluish dark green          (hue ~156 deg)
    "pepper_green": (0.05, 0.42, 0.22),   # deep glossy green          (hue ~148 deg)
    "tomato_green": (0.58, 0.76, 0.18),   # yellowish green, unripe    (hue ~ 79 deg)
}
# fruit material -> (crop, ripeness class)
FRUIT_MATS = {
    "tomato_red": ("tomato", "ripe"), "tomato_orange": ("tomato", "turning"), "tomato_green": ("tomato", "unripe"),
    "pepper_red": ("pepper", "ripe"), "pepper_yellow": ("pepper", "ripe"), "pepper_orange": ("pepper", "ripe"),
    "pepper_green": ("pepper", "unripe"),
    "cucumber": ("cucumber", "ripe"),
    "strawberry": ("strawberry", "ripe"), "strawberry_unripe": ("strawberry", "unripe"),
}
# coverage of this session (config/greenhouse_map.yaml): lane distance from the row face, pass heights
STANDOFF = 1.2
# Two passes: this UAV's rbl keeps it between ~1.1 and ~2.0 m here (0.9 / 2.2 m goals were never reached), and the
# portrait camera sees +-1.3 m of the plants at 1.6 m, so 1.2 / 1.9 m cover the gutter to the plant tops
COVER_LEVELS = [1.2, 1.9]
# lateral colour + depth cameras of the UAV (crop_scanner.py): portrait (rolled 90 deg about the view axis), so the 80 deg
# FOV spans the plant height: +-1.3 m at 1.6 m; a 6 cm tomato is ~35 px there
LATCAM = {"width": 1280, "height": 960, "hfov": 1.40, "rate": 5, "y": 0.10, "z": 0.03, "roll": math.pi / 2}
FRUITS = []      # every fruit of the world: dict(id, crop, ripeness, color, x, y, z, r, row, segment, ...)
SEG_TRIS = {}    # row segment index -> (triangles of its plants as (N, 3, 3) array, fruit ids in it)


class FruitMesh(gh.Mesh):
    """gh.Mesh that also records every fruit drawn into it (world coordinates, rest pose)."""

    def __init__(self):
        super().__init__()
        self.fruits = []

    def ellipsoid(self, mat, c, rx, ry, rz, nu=6, nv=4):
        if mat in FRUIT_MATS:
            self.fruits.append((mat, tuple(c), (rx, ry, rz)))
        super().ellipsoid(mat, c, rx, ry, rz, nu, nv)

WALK_SPEED = 0.9            # [m/s]
TURN_TIME = 0.8             # [s] turning on the spot between two legs
ACTOR_Z = 1.0               # hip height of the actor mesh origin
# Looping routes on the aisle centrelines (aisles y = -11.4 .. 11.4, cross-aisles x = -14, 0, 14, strips
# along the end walls x = +-29). Each stop: (x, y, dwell [s], facing yaw or None). Crates stand 0.55..0.95 m
# off the centreline, a person on the centreline passes them.
OPERATORS = {
    # round the block between the main aisle and aisle 5, segment 3; crosses 0.8 m in front of the UGV's
    # spawn point (0, 0) and meets the vehicles going to rows 4/5 column 3
    "operator_1": [(0.8, 0.0, 0, None), (7.0, 0.0, 5, math.pi / 2), (14.0, 0.0, 0, None),
                   (14.0, 3.8, 0, None), (6.0, 3.8, 6, -math.pi / 2), (0.8, 3.8, 0, None)],
    # harvesting along aisle 2 (y = -7.6), back and forth over segments 1-2, through the x = -14 cross-aisle
    "operator_2": [(-27.0, -7.6, 4, math.pi / 2), (-20.0, -7.6, 8, -math.pi / 2), (-9.0, -7.6, 8, math.pi / 2),
                   (-3.0, -7.6, 3, None)],
    # aisle 6 (y = 7.6) far end, then down the x = -14 cross-aisle to the main aisle and back
    "operator_3": [(-26.0, 7.6, 6, -math.pi / 2), (-14.0, 7.6, 0, None), (-14.0, 0.0, 4, math.pi),
                   (-14.0, 3.8, 0, None), (-22.0, 3.8, 7, math.pi / 2), (-14.0, 3.8, 0, None),
                   (-14.0, 7.6, 0, None)],
}


# ------------------------------------------------------------------------------------------------
# sub-meshes (a subset of a gh.Mesh's faces, re-indexed, translated to a local origin)
# ------------------------------------------------------------------------------------------------
def faces_of(mesh, keep):
    """[(mat, (ia, ib, ic, n))] of the faces whose material passes keep(mat)."""
    return [(mat, f) for mat, fs in mesh.f.items() if keep(mat) for f in fs]


def centroid(mesh, faces):
    idx = {i for _, f in faces for i in f[:3]}
    n = max(1, len(idx))
    return tuple(sum(mesh.v[i - 1][k] for i in idx) / n for k in range(3))


def write_sub(mesh, faces, origin, path, name):
    vmap, nmap, out_v, out_n, by_mat = {}, {}, [], [], {}
    for mat, (a, b, c, n) in faces:
        tri = []
        for i in (a, b, c):
            if i not in vmap:
                p = mesh.v[i - 1]
                out_v.append((p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]))
                vmap[i] = len(out_v)
            tri.append(vmap[i])
        if n not in nmap:
            out_n.append(mesh.n[n - 1])
            nmap[n] = len(out_n)
        by_mat.setdefault(mat, []).append((*tri, nmap[n]))
    with open(path, "w") as f:
        f.write(f"# generated by px4_sim_ugv_uav_greenhouse_dynamic/generate.py\nmtllib crops.mtl\no {name}\n")
        f.writelines(f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n" for p in out_v)
        f.writelines(f"vn {q[0]:.3f} {q[1]:.3f} {q[2]:.3f}\n" for q in out_n)
        for mat, fs in by_mat.items():
            f.write(f"usemtl {mat}\n")
            f.writelines(f"f {a}//{n} {b}//{n} {c}//{n}\n" for a, b, c, n in fs)


def visual(name, mesh_name, origin):
    return f"""        <visual name="{name}">
          <pose>{origin[0]:.4f} {origin[1]:.4f} {origin[2]:.4f} 0 0 0</pose>
          <geometry><mesh><uri>model://{MESH_MODEL}/meshes/{mesh_name}.obj</uri></mesh></geometry>
        </visual>
"""


# ------------------------------------------------------------------------------------------------
# crop row segment = greenhouse.crop_row with the plants cut into swaying / fluttering groups
# ------------------------------------------------------------------------------------------------
def crop_row(rng, idx, x0, x1, y, crop, mesh_dir):
    length = x1 - x0
    cx = (x0 + x1) / 2
    plant, step = gh.CROPS[crop]
    gz = gh.SB_GUTTER_Z if crop == "strawberry" else gh.GUTTER_Z
    body = box("soil", cx, y, 0.002, length, gh.ROW_DEPTH + 0.2, 0.004, gh.SOIL, collide=False)
    body += box("gutter", cx, y, gz, length, gh.GUTTER_W, gh.GUTTER_H, gh.GUTTER)
    nlegs = max(2, round(length / 2.5) + 1)
    for i in range(nlegs):
        lx = x0 + 0.1 + i * (length - 0.2) / (nlegs - 1)
        body += box(f"leg{i}", lx, y, (gz - gh.GUTTER_H / 2) / 2, 0.05, 0.25, gz - gh.GUTTER_H / 2, gh.STEEL)
    body += box("wire", cx, y, gh.WIRE_Z, length, 0.02, 0.02, gh.STEEL)
    for k, px in enumerate((x0 + 0.05, cx, x1 - 0.05)):
        body += box(f"post{k}", px, y, gh.POST_H / 2, 0.08, 0.08, gh.POST_H, gh.STEEL)

    top = gz + gh.GUTTER_H / 2
    # rockwool slabs: static
    slabs = gh.Mesh()
    nslabs = int(length / 1.0)
    for i in range(nslabs):
        slabs.box("slab", (x0 + (i + 0.5) * length / nslabs, y, top + 0.04), length / nslabs - 0.05, 0.2, 0.08)
    write_sub(slabs, faces_of(slabs, lambda m: True), (0, 0, 0), os.path.join(mesh_dir, f"row{idx}_slabs.obj"),
              f"row{idx}_slabs")
    body += visual("slabs_v", f"row{idx}_slabs", (0, 0, 0))

    # plants, chunk by chunk (same plant positions / random sequence as greenhouse.py)
    nchunks = max(1, round(length / CHUNK))
    chunks = [FruitMesh() for _ in range(nchunks)]
    hmax = 0.0
    nplants = int(length / step)
    for i in range(nplants):
        px = x0 + (i + 0.5) * length / nplants
        if crop != "strawberry" and (abs(px - cx) < 0.12 or px - x0 < 0.12 or x1 - px < 0.12):
            continue  # posts
        c = min(nchunks - 1, int((px - x0) / length * nchunks))
        hmax = max(hmax, plant(chunks[c], rng, px, y))
    tris = 0
    bands = LEAF_BANDS[crop]
    zlo, zhi = (top, top + 0.25) if crop == "strawberry" else (0.6, max(hmax, 1.0))
    for c, m in enumerate(chunks):
        if not m.v:
            continue
        tris += m.triangles()
        cid = f"r{idx}c{c}"
        pivot = (x0 + (c + 0.5) * length / nchunks, y, top)
        sway = crop != "strawberry"
        if sway:
            stems = faces_of(m, lambda mat: mat not in LEAF_MATS)
            write_sub(m, stems, pivot, os.path.join(mesh_dir, f"{cid}_stems.obj"), f"{cid}_stems")
            body += visual(f"sway_{cid}", f"{cid}_stems", pivot)
        else:  # strawberries: stems / berries static, leaves flutter
            rest = faces_of(m, lambda mat: mat not in LEAF_MATS)
            write_sub(m, rest, (0, 0, 0), os.path.join(mesh_dir, f"{cid}_rest.obj"), f"{cid}_rest")
            body += visual(f"static_{cid}", f"{cid}_rest", (0, 0, 0))
        # leaf groups: height band x aisle side
        groups = {}
        for mat, f in faces_of(m, lambda mat: mat in LEAF_MATS):
            pa, pb, pc = (m.v[i - 1] for i in f[:3])
            zc = (pa[2] + pb[2] + pc[2]) / 3
            yc = (pa[1] + pb[1] + pc[1]) / 3
            band = min(bands - 1, max(0, int((zc - zlo) / max(1e-6, zhi - zlo) * bands)))
            side = 0 if (yc >= y or crop == "strawberry") else 1
            groups.setdefault((band, side), []).append((mat, f))
        for (band, side), faces in sorted(groups.items()):
            o = centroid(m, faces)
            name = f"{cid}_l{band}{side}"
            write_sub(m, faces, o, os.path.join(mesh_dir, f"{name}.obj"), name)
            # strawberries: id without a matching sway_ visual -> flutter only
            vis = f"flutter_{cid}_{band}{side}" if sway else f"flutter_s{idx}c{c}{band}{side}"
            body += visual(vis, name, o)

    # fruit ground truth + this segment's triangles (for the visibility check)
    seg = gh.segments().index((x0, x1)) + 1
    ids, tri_list = [], []
    # occluders besides the plants: the gutter, its legs and the rockwool slabs (strawberries hang over both
    # gutter edges: the far side's berries are hidden by it)
    np_ = __import__("numpy")
    occ = gh.Mesh()
    npc = max(1, int(math.ceil(length / 0.5)))   # in 0.5 m pieces: the ray-cast picks occluders by their centre's x
    for k in range(npc):
        occ.box("x", (x0 + (k + 0.5) * length / npc, y, gz), length / npc, gh.GUTTER_W, gh.GUTTER_H)
    for i in range(nlegs):
        lx = x0 + 0.1 + i * (length - 0.2) / (nlegs - 1)
        occ.box("x", (lx, y, (gz - gh.GUTTER_H / 2) / 2), 0.05, 0.25, gz - gh.GUTTER_H / 2)
    for m in (occ, slabs):
        V = np_.array(m.v)
        for faces in m.f.values():
            tri_list.append(V[np_.array([f[:3] for f in faces]) - 1])
    for m in chunks:
        for mat, c, rr in m.fruits:
            crop_, ripe = FRUIT_MATS[mat]
            fid = f"{crop_[:3].upper()}-{len(FRUITS) + 1:05d}"
            FRUITS.append(dict(id=fid, crop=crop_, ripeness=ripe, color=mat, x=round(c[0], 3), y=round(c[1], 3),
                               z=round(c[2], 3), r=[round(v, 3) for v in rr], row=idx // 4, row_y=y, segment=seg))
            ids.append(fid)
        if m.v:
            V = __import__("numpy").array(m.v)
            for faces in m.f.values():
                F = __import__("numpy").array([f[:3] for f in faces]) - 1
                tri_list.append(V[F])
    if tri_list:
        SEG_TRIS[idx] = (__import__("numpy").concatenate(tri_list), ids)
    if crop == "strawberry":
        body += gh.collision_box("canopy", cx, y, top + 0.12, length, 0.5, 0.24)
    else:
        z0 = 0.75
        body += gh.collision_box("canopy", cx, y, (z0 + hmax) / 2, length, 0.6, hmax - z0)
    return model(f"croprow{idx}", body), tris


# ------------------------------------------------------------------------------------------------
# operators (gz actors)
# ------------------------------------------------------------------------------------------------
def actor(name, stops):
    uri = f"model://{OPERATOR_MODEL}/meshes"
    trajs, tid = [], 0

    def traj(kind, poses):  # poses: [(t, x, y, yaw)]
        nonlocal tid
        wps = "".join(f"<waypoint><time>{t:.2f}</time><pose>{x:.3f} {y:.3f} {ACTOR_Z} 0 0 {yaw:.4f}</pose></waypoint>"
                      for t, x, y, yaw in poses)
        trajs.append(f'        <trajectory id="{tid}" type="{kind}">{wps}</trajectory>\n')
        tid += 1

    n = len(stops)
    heading = math.atan2(stops[1][1] - stops[0][1], stops[1][0] - stops[0][0])
    for i in range(n):
        x, y, dwell, face = stops[i]
        nx, ny = stops[(i + 1) % n][:2]
        nxt = math.atan2(ny - y, nx - x)
        if dwell > 0:
            face = heading if face is None else face
            traj("stand", [(0, x, y, heading), (TURN_TIME, x, y, _unwrap(heading, face)),
                           (TURN_TIME + dwell, x, y, _unwrap(heading, face))])
            heading = face
        traj("stand", [(0, x, y, heading), (TURN_TIME, x, y, _unwrap(heading, nxt))])
        d = math.hypot(nx - x, ny - y)
        traj("walk", [(0, x, y, nxt), (d / WALK_SPEED, nx, ny, nxt)])
        heading = nxt
    x0, y0 = stops[0][:2]
    return f"""    <actor name="{name}">
      <pose>{x0:.3f} {y0:.3f} {ACTOR_Z} 0 0 0</pose>
      <skin><filename>{uri}/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>{uri}/walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <animation name="stand"><filename>{uri}/stand.dae</filename><scale>1.0</scale></animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
{''.join(trajs)}      </script>
    </actor>
"""


def _unwrap(a, b):
    """b shifted by 2*pi*k to be within pi of a (shortest turn)."""
    return a + math.atan2(math.sin(b - a), math.cos(b - a))


def write_model_files(model_dir, name, description):
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
""")
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f'<?xml version="1.0"?>\n<sdf version="1.9"><model name="{name}"><static>true</static>'
                f'<link name="link"/></model></sdf>\n')


def patch_session_files():
    """Coverage levels and tolerances of config/greenhouse_map.yaml. Idempotent."""
    p = os.path.join(HERE, "config", "greenhouse_map.yaml")
    t = open(p).read()
    for a, b in (("      tolerance: 0.6 ", "      tolerance: 0.5 "), ("      tolerance_pass: 1.0", "      tolerance_pass: 0.4")):
        t = t.replace(a, b, 1)
    import re as _re
    t = _re.sub(r"      levels: \[[^]]*\]", f"      levels: {COVER_LEVELS}", t, count=1)
    if "tolerance_z:" not in t:
        # a coverage goal counts only within 0.15 m of its pass height (see the warehouse_dynamic session's README)
        t = t.replace("      tolerance_pass: 0.4", "      tolerance_z: 0.15        # [m] coverage goals: also within this "
                      "of their height\n      tolerance_pass: 0.4", 1)
    open(p, "w").write(t)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-operators", action="store_true")
    args = ap.parse_args()
    rng = random.Random(args.seed)
    patch_session_files()

    worlds_dir = os.path.join(HERE, "worlds")
    model_dir = os.path.join(worlds_dir, "models", MESH_MODEL)
    mesh_dir = os.path.join(model_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    for f in os.listdir(mesh_dir):
        if f.endswith(".obj"):
            os.remove(os.path.join(mesh_dir, f))
    gh.MATERIALS.update(MATERIAL_OVERRIDES)
    gh.write_mtl(os.path.join(mesh_dir, "crops.mtl"))
    write_model_files(model_dir, MESH_MODEL, "Crop meshes of worlds/greenhouse_dynamic.sdf, cut into swaying / "
                      "fluttering groups by generate.py -- not spawned on its own.")
    op_dir = os.path.join(worlds_dir, "models", OPERATOR_MODEL)
    write_model_files(op_dir, OPERATOR_MODEL, "Walking / standing person (skin + animations of the 'actor' model "
                      "by Mingfei on fuel.gazebosim.org), used by the actors of worlds/greenhouse_dynamic.sdf.")

    out = header(WORLD_NAME, ground=(0.62, 0.62, 0.60), ground_size=(100, 60), background=(0.75, 0.85, 0.95),
                 ambient=0.6)
    out += gh.walls() + gh.roof_gutters()
    i, tris = 0, 0
    nsway = nflut = 0
    for y, crop in zip(gh.ROW_YS, gh.ROW_CROPS):
        for x0, x1 in gh.segments():
            xml, t = crop_row(rng, i, x0, x1, y, crop, mesh_dir)
            nsway += xml.count('name="sway_')
            nflut += xml.count('name="flutter_')
            out += xml
            tris += t
            i += 1
    out += gh.temp_obstacles(rng.randint(0, 10**6))
    if not args.no_operators:
        for name, stops in OPERATORS.items():
            out += actor(name, stops)
    out += FOOTER
    path = os.path.join(worlds_dir, WORLD_NAME + ".sdf")
    with open(path, "w") as f:
        f.write(out)
    print(f"{path}: {i} row segments, {tris} plant triangles, {nsway} swaying plant groups, {nflut} leaf groups, "
          f"{0 if args.no_operators else len(OPERATORS)} operators")
    write_crop_truth(os.path.join(worlds_dir, "crop_truth.json"))
    import session_gen
    session_gen.make_session_files(LATCAM)


def _rays_blocked(o, d, tris, eps=1e-6):
    """Moller-Trumbore, rays o + t d (t in (0, 1)) x triangles: (R,) bool, any triangle hit."""
    import numpy as np
    blocked = np.zeros(len(o), bool)
    if len(tris) == 0:
        return blocked
    v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    for k in range(len(o)):
        p = np.cross(d[k], e2)
        det = np.einsum("ij,ij->i", e1, p)
        ok = np.abs(det) > eps
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        t_ = o[k] - v0
        u = np.einsum("ij,ij->i", t_, p) * inv
        q = np.cross(t_, e1)
        v = np.einsum("j,ij->i", d[k], q) * inv
        t = np.einsum("ij,ij->i", e2, q) * inv
        blocked[k] = np.any(ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-4) & (t < 0.999))
    return blocked


def write_crop_truth(path):
    """worlds/crop_truth.json: every fruit, and from which aisle side it is visible: a fruit counts as visible
    from a face if at least 3 of 5 points of its front (centre + 4 around it) are unoccluded by leaves / stems /
    other fruits from some lane point (fruit x and +-0.6 m along the row, every pass height, STANDOFF from the
    face). The plants sway in the wind, so this is the rest pose; the reports score against it."""
    import numpy as np
    by_id = {f["id"]: f for f in FRUITS}
    for idx, (tris, ids) in SEG_TRIS.items():
        cen = tris.mean(axis=1)
        for fid in ids:
            f = by_id[fid]
            fc, rmax = np.array([f["x"], f["y"], f["z"]]), max(f["r"])
            f["visible_from"] = []
            for sgn in (-1, 1):   # face looking into the aisle at -y / +y of the row
                aisle_y = f["row_y"] + sgn * 1.9
                if not any(abs(aisle_y - a) < 1e-6 for a in gh.AISLE_YS):
                    continue
                lane_y = f["row_y"] + sgn * (gh.ROW_DEPTH / 2 + STANDOFF)
                # candidate occluders: between the fruit and the lane, near its x, not the fruit itself
                near = (np.abs(cen[:, 0] - fc[0]) < 0.9) & ((cen[:, 1] - fc[1]) * sgn > -rmax) & \
                       (np.linalg.norm(cen - fc, axis=1) > rmax * 1.15)
                T = tris[near]
                front = fc + np.array([0, sgn * rmax * 0.8, 0])
                pts = [front] + [front + np.array([dx, 0, dz]) for dx, dz in
                                 ((0.6 * f["r"][0], 0), (-0.6 * f["r"][0], 0), (0, 0.6 * f["r"][2]), (0, -0.6 * f["r"][2]))]
                seen = False
                for vx in (0.0, -0.6, 0.6):
                    for h in COVER_LEVELS:
                        cam = np.array([fc[0] + vx, lane_y, h])
                        o = np.repeat(cam[None], len(pts), 0)
                        d = np.array(pts) - cam
                        if (~_rays_blocked(o, d, T)).sum() >= 3:
                            seen = True
                            break
                    if seen:
                        break
                if seen:
                    aisle = gh.AISLE_YS.index(min(gh.AISLE_YS, key=lambda a: abs(a - aisle_y))) + 1
                    f["visible_from"].append(dict(aisle=aisle, side="left" if sgn < 0 else "right"))
    with open(path, "w") as fh:
        json.dump(dict(description="Fruits of worlds/greenhouse_dynamic.sdf (generate.py): rest position [m, world "
                                   "frame], radii, crop, ripeness, render colour, row segment and the aisle sides "
                                   "(deploy.sh row = aisle, column = segment) from which each is visible.",
                       materials={k: list(v) for k, v in gh.MATERIALS.items()},
                       fruits=FRUITS), fh, indent=0)
    import collections
    cnt = collections.Counter((f["crop"], f["ripeness"]) for f in FRUITS)
    vis = sum(1 for f in FRUITS if f["visible_from"])
    print(f"{path}: {len(FRUITS)} fruits {dict(cnt)}, {vis} visible from at least one aisle")


if __name__ == "__main__":
    main()
