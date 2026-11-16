#!/usr/bin/env python3
"""Generate px4_sim_ugv_uav_warehouse_dynamic: ONE UGV + UAV pair (--pairs: up to 5) of the warehouse session of
../px4_sim_5ugv_5uav_warehouse_light (same vehicles, ArUco precision landing, light settings: its generate.py
is imported and re-pointed here) in a warehouse that moves and has codes on the shelves.

  ./generate.py              1 pair (PAIRS)
  ./generate.py --pairs 3    more (up to 5)

On top of the light session:
  - shelf codes: a QR LOCATION label on the front edge of every bay x level of every rack face that
    faces an aisle (LOC:A<aisle>-S<segment>-<L|R>-B<bay>-L<level>), and a QR CARTON label on both faces
    of every carton (CTN-<id>/SKU-<sku>). Packed into one texture atlas + one mesh per rack row
    (models/shelf_labels), visual only. Ground truth for the reports: worlds/shelf_codes.json.
  - UAV models uav<k>_x500_scan: + left / right lateral cameras (LATCAM below), rendered only while
    that UAV's shelf_scanner subscribes (coverage of a shelf, only the camera facing it). The scanner reads
    them straight from gz-transport: a ros_gz_bridge, even lazy, keeps them subscribed (= rendered).
  - dynamic scene: OPERATORS (gz actors walking the aisles, stopping at the racks) and FORKLIFTS
    (static models driven along the cross-aisles by gz_greenhouse_dynamics::PathFollower). Visual only,
    no collision, not in the planners' prior map: the depth cameras / rbl have to deal with them.
  - worlds/warehouse.sdf: the shared warehouse with its intersecting cartons packed side by side (fix_cartons:
    a label inside another carton can't be read), and every label checked visible from its aisle.
  - config/warehouse_map.yaml: coverage pass heights 0.8 / 2.0 / 3.2 m so the lateral cameras see
    every label level (0.26 / 1.46 / 2.66 / 3.86 m) inside their field of view.
  - session.yml: + `scan` window (shelf_scanner.py per UAV: decodes the
    codes, logs them, writes a report when the shelf is done, see README.md).

Edit this file (or the light session's generate.py for what it shares), not the outputs.
"""
import argparse
import importlib.util
import json
import math
import os
import random
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LIGHT_DIR = os.path.join(HERE, '..', 'px4_sim_5ugv_5uav_warehouse_light')

spec = importlib.util.spec_from_file_location('light_gen', os.path.join(LIGHT_DIR, 'generate.py'))
PAIRS = 1
L = importlib.util.module_from_spec(spec)
spec.loader.exec_module(L)
# re-point the light generator at this session (its functions read these module globals at call time)
L.HERE = HERE
L.SESSION = 'ugv_uav_warehouse_dynamic'
L.SOCKET = 'whdyn5p'
L.DOMAIN = 46
L.WORLD = 'warehouse_dynamic'
L.m_ugv = lambda k: f'ugv{k}_carrier_dyn'   # own model names: the light session's PX4 links stay intact
L.m_uav = lambda k: f'uav{k}_x500_scan'

# --------------------------------------------------------------------------------- warehouse layout
# (mirrors ../px4_sim_1_uav_3dlidar/worlds/worldgen/worlds/warehouse.py, which made worlds/warehouse.sdf)
ROW_YS = [-14.7, -10.5, -6.3, -2.1, 2.1, 6.3, 10.5, 14.7]
AISLE_YS = [-12.6, -8.4, -4.2, 0.0, 4.2, 8.4, 12.6]
SEGMENTS = [(-33.0, -18.0), (-15.0, -1.5), (1.5, 15.0), (18.0, 33.0)]
SHELF_Z = [0.15, 1.35, 2.55, 3.75]
RACK_DEPTH = 1.2
BAY = 2.7

# --------------------------------------------------------------------------------- labels
TILE_W, TILE_H = 160, 192   # atlas tile [px]: QR (25 modules incl. quiet zone x 6 px = 150 px) + text
MODULE_PX = 6
PLATE_W = 0.16              # [m] label width -> 1 mm per texel, 6 mm QR modules (21 x 6 = 126 mm code)
PLATE_H = PLATE_W * TILE_H / TILE_W
QR_SIZE = 21 * MODULE_PX * PLATE_W / TILE_W   # [m] side of the code itself (finder pattern corners)
LABEL_OUT = 0.006           # [m] in front of the rack face / carton face
SKU_SEED = 7

# --------------------------------------------------------------------------------- lateral cameras
# portrait (roll 90 deg about the view axis): the 1.40 rad FOV spans the HEIGHT of the rack -> +-1.0 m of shelf at
# the 1.2 m standoff (+-0.8 m along it). Landscape (+-0.8 m vertically) missed the lowest / highest label level:
# rbl does not reach the pass heights exactly (arrival tolerances).
LATCAM = {'width': 1600, 'height': 1200, 'hfov': 1.40, 'rate': 6, 'y': 0.10, 'z': 0.0, 'roll': math.pi / 2}

# --------------------------------------------------------------------------------- dynamics
WALK_SPEED, TURN_TIME, ACTOR_Z = 0.9, 0.8, 1.0
# routes on aisle centrelines (red crates stand 0.53..0.98 m off them), away from the x = 0 cross-aisle
# where the pairs park: (x, y, dwell [s], facing yaw or None)
OPERATORS = {
    'operator_1': [(-31.0, -8.4, 5, math.pi / 2), (-20.0, -8.4, 0, None), (-9.0, -8.4, 6, -math.pi / 2),
                   (-3.0, -8.4, 2, None)],
    'operator_2': [(3.0, 4.2, 4, -math.pi / 2), (12.0, 4.2, 7, math.pi / 2), (30.0, 4.2, 4, None)],
    'operator_3': [(30.0, -4.2, 5, math.pi / 2), (8.0, -4.2, 6, -math.pi / 2), (3.0, -4.2, 2, None)],
    'operator_4': [(-30.0, 0.0, 3, None), (-12.0, 0.0, 5, math.pi / 2), (-3.0, 0.0, 2, None)],
}
# forklifts shuttle up and down the x = +-16.5 cross-aisles (columns at x = +-15.7 / 17.3), between the
# strips along the side walls, waiting at each end
FORKLIFTS = {
    'forklift_1': {'path': [(-17.0, -16.6), (-17.0, 16.6)], 'speed': 1.2, 'dwell': 6},
    'forklift_2': {'path': [(16.2, 16.6), (16.2, -16.6)], 'speed': 1.0, 'dwell': 8},
}


def write(path, text):
    L.write(path, text)


# ================================================================================= vehicle models
def add_lateral_cameras(n):
    """Left (+y) / right (-y) cameras on every uav<k>_x500_scan, looking sideways at the shelf."""
    c = LATCAM
    for k in range(1, n + 1):
        path = os.path.join(HERE, f'models/{L.m_uav(k)}/model.sdf')
        s = open(path).read()
        links = ''
        for side, sgn in (('left', 1), ('right', -1)):
            links += f'''
    <link name="{side}_cam_link">
      <pose relative_to="base_link">0 {sgn * c['y']} {c['z']} {c['roll']:.4f} 0 {sgn * math.pi / 2:.4f}</pose>
      <inertial>
        <mass>0.02</mass>
        <inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia>
      </inertial>
      <visual name="{side}_cam_visual">
        <geometry><box><size>0.02 0.04 0.03</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="{side}_cam" type="camera">
        <gz_frame_id>uav{k}/{side}_cam_link</gz_frame_id>
        <camera>
          <horizontal_fov>{c['hfov']}</horizontal_fov>
          <image><width>{c['width']}</width><height>{c['height']}</height><format>L8</format></image>
          <clip><near>0.1</near><far>15.0</far></clip>
          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.004</stddev></noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>{c['rate']}</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="{side}_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>{side}_cam_link</child>
    </joint>
'''
        assert s.rstrip().endswith('</sdf>') and '</model>' in s
        s = s.replace('  </model>', links + '  </model>', 1)
        s = s.replace('depth camera 96x72. -->', f"depth camera 96x72, + left / right {c['width']}x{c['height']} "
                      'lateral cameras (px4_sim_ugv_uav_warehouse_dynamic/generate.py). -->')
        open(path, 'w').write(s)


# ================================================================================= shelf labels
def faces():
    """Rack faces seen from an aisle: (row index, row y, face y, normal sign, aisle id, side)."""
    out = []
    for r, ry in enumerate(ROW_YS):
        for sgn in (-1, 1):
            ay = ry + sgn * 2.1   # the aisle this face looks into
            if not any(abs(ay - a) < 1e-6 for a in AISLE_YS):
                continue          # outer faces: along the side walls, no aisle
            aisle = AISLE_YS.index(min(AISLE_YS, key=lambda a: abs(a - ay))) + 1
            side = 'left' if sgn < 0 else 'right'   # rack at +y of the aisle = its left shelf
            out.append((r, ry, ry + sgn * RACK_DEPTH / 2, sgn, aisle, side))
    return out


def cartons_by_rack():
    """Cartons of worlds/warehouse.sdf (visuals c<n> of models rack<i>): rack index -> [(x, y, z, sx, sy, sz)]."""
    w = ET.parse(os.path.join(HERE, 'worlds/warehouse.sdf')).getroot().find('world')
    out = {}
    for m in w.findall('model'):
        name = m.get('name')
        if not name.startswith('rack'):
            continue
        idx = int(name[4:])
        Tm = L._T(L._pose(m))
        for l in m.findall('link'):
            Tl = Tm @ L._T(L._pose(l))
            for v in l.findall('visual'):
                if not re.fullmatch(r'c\d+_v|c\d+', v.get('name', '')):
                    continue
                size = [float(x) for x in v.find('geometry/box/size').text.split()]
                p = (Tl @ L._T(L._pose(v)))[:3, 3]
                out.setdefault(idx, []).append((p[0], p[1], p[2], *size))
    return out


CARTON_GAP = 0.08   # [m] between two cartons packed side by side in one bay
# coverage of this session (config/warehouse_map.yaml): pass heights, lane distance from the face, height tolerance.
# The visibility check flies the lateral camera through them.
COVER_LEVELS = [0.8, 2.0, 3.4]
STANDOFF = 1.2
TOL_Z = 0.15


def fix_cartons():
    """worlds/warehouse.sdf = the shared world (../px4_sim_1_uav_3dlidar/worlds/warehouse.sdf) with the cartons
    of each bay x level that INTERSECT (its generator draws 1-2 per bay at random x, overlap allowed) packed side
    by side instead: an intersecting carton has its label inside the other box, unreadable by any camera (22 of
    the 23 such cartons were missed). Visual and collision boxes move together, so the planners' map (collisions)
    matches what the cameras see. Everything else of the world is unchanged."""
    src = os.path.join(HERE, '..', 'px4_sim_1_uav_3dlidar', 'worlds', 'warehouse.sdf')
    tree = ET.parse(src)
    w = tree.getroot().find('world')
    moved = 0
    for m in w.findall('model'):
        if not m.get('name', '').startswith('rack'):
            continue
        idx = int(m.get('name')[4:])
        x0, x1 = SEGMENTS[idx % 4]
        nb = max(1, round((x1 - x0) / BAY))
        bay = (x1 - x0) / nb
        for l in m.findall('link'):
            boxes = {}  # carton name -> [visual, collision]
            for e in list(l.findall('visual')) + list(l.findall('collision')):
                mm = re.fullmatch(r'(c\d+)_[vc]', e.get('name', ''))
                if mm:
                    boxes.setdefault(mm.group(1), []).append(e)
            groups = {}
            for name, els in boxes.items():
                p = [float(v) for v in els[0].find('pose').text.split()]
                sx, sy, sz = [float(v) for v in els[0].find('geometry/box/size').text.split()]
                b = min(nb - 1, max(0, int((p[0] - x0) / bay)))
                lv = min(range(len(SHELF_Z)), key=lambda i: abs(p[2] - sz / 2 - 0.025 - SHELF_Z[i]))
                groups.setdefault((b, lv), []).append([p, sx, els])
            for (b, lv), cs in groups.items():
                cs.sort(key=lambda c: c[0][0])
                overlap = any(cs[i][0][0] + cs[i][1] / 2 > cs[i + 1][0][0] - cs[i + 1][1] / 2
                              for i in range(len(cs) - 1))
                if not overlap:
                    continue
                total = sum(c[1] for c in cs) + CARTON_GAP * (len(cs) - 1)
                free = bay - 0.08 - 0.1   # uprights + margins
                assert total <= free, (idx, b, lv, total)
                x = x0 + (b + 0.5) * bay - total / 2
                for p, sx, els in cs:
                    p[0] = x + sx / 2
                    x += sx + CARTON_GAP
                    for e in els:
                        q = e.find('pose').text.split()
                        q[0] = f'{p[0]:.3f}'
                        e.find('pose').text = ' '.join(q)
                    moved += 1
    body = ET.tostring(tree.getroot(), encoding='unicode')
    dst = os.path.join(HERE, 'worlds', 'warehouse.sdf')
    if os.path.islink(dst):
        os.remove(dst)
    write('worlds/warehouse.sdf', '<?xml version="1.0" encoding="UTF-8"?>\n<!-- GENERATED by generate.py (fix_cartons): '
          '../px4_sim_1_uav_3dlidar/worlds/warehouse.sdf with intersecting cartons packed side by side -->\n' + body)
    print(f'cartons: {moved} intersecting cartons repacked side by side')


def check_label_visibility(manifest, cartons):
    """Every label must be visible straight out from its face: not inside a carton, not behind one."""
    bad = []
    allc = [c for v in cartons.values() for c in v]
    for e in manifest:
        sgn = -1 if e['side'] == 'left' else 1    # outward normal of the face (towards its aisle)
        lx0, lx1, lz0, lz1 = e['x'] - PLATE_W / 2, e['x'] + PLATE_W / 2, e['z'] - PLATE_H / 2, e['z'] + PLATE_H / 2
        for (cx, cy, cz, sx, sy, sz) in allc:
            if min(lx1, cx + sx / 2) - max(lx0, cx - sx / 2) <= 0.005 or min(lz1, cz + sz / 2) - max(lz0, cz - sz / 2) <= 0.005:
                continue
            # any part of the box on the aisle side of the label (or around it) hides it
            if (cy + sy / 2 > e['y'] + 1e-3) if sgn > 0 else (cy - sy / 2 < e['y'] - 1e-3):
                if abs(cy - e['y']) < 1.0:   # same rack row (not the rack across the aisle)
                    bad.append(e['code'])
                    break
    assert not bad, f'{len(bad)} labels hidden by cartons, e.g. {bad[:5]}'
    print(f'labels: all {len(manifest)} visible from their aisle (none inside / behind a carton)')
    check_label_sightlines(manifest, allc)


def _ray_hits_box(p, q, lo, hi):
    """2D segment p->q (y, z) intersects the open box lo..hi?"""
    t0, t1 = 0.0, 1.0
    for k in range(2):
        d = q[k] - p[k]
        if abs(d) < 1e-12:
            if not lo[k] < p[k] < hi[k]:
                return False
            continue
        a, b = (lo[k] - p[k]) / d, (hi[k] - p[k]) / d
        a, b = min(a, b), max(a, b)
        t0, t1 = max(t0, a), min(t1, b)
        if t0 >= t1:
            return False
    return True


def check_label_sightlines(manifest, allc):
    """Each label's whole QR code (incl. quiet zone) must be seen by the lateral camera, at the label's x, from
    at least one coverage pass -- at its height and TOL_Z above / below it, STANDOFF from the face: inside 95 % of
    the vertical half-FOV and not blocked by a shelf plank's front edge or another carton (y-z plane)."""
    half_v = LATCAM["hfov"] / 2 * 0.95   # portrait: the horizontal FOV spans the height (codes at ~35 deg decode)
    qr_top = PLATE_H / 2 - 2 / TILE_H * PLATE_H
    qr_bot = PLATE_H / 2 - (2 + 25 * MODULE_PX) / TILE_H * PLATE_H
    bad = []
    for e in manifest:
        sgn = -1 if e['side'] == 'left' else 1
        ry = e['y'] - sgn * (RACK_DEPTH / 2 + LABEL_OUT) if e['type'] == 'location' else \
            min(ROW_YS, key=lambda r: abs(r - e['y']))
        obst = [((ry - RACK_DEPTH / 2, z - 0.025), (ry + RACK_DEPTH / 2, z + 0.025)) for z in SHELF_Z]
        for (cx, cy, cz, sx, sy, sz) in allc:
            if abs(cy - ry) < 1.0 and abs(cx - e['x']) < sx / 2 + PLATE_W / 2 and \
                    not (abs(cx - e['x']) < 1e-3 and abs(cz - e['z']) < sz / 2):   # not its own carton
                obst.append(((cy - sy / 2, cz - sz / 2), (cy + sy / 2, cz + sz / 2)))
        yc = e['y'] + sgn * STANDOFF
        ok = False
        for h in COVER_LEVELS:
            seen = True
            for zc in (h - TOL_Z, h, h + TOL_Z):
                for zq in (e['z'] + qr_top, e['z'] + qr_bot):
                    if abs(math.atan2(zq - zc, STANDOFF)) > half_v or \
                            any(_ray_hits_box((yc, zc), (e['y'], zq), lo, hi) for lo, hi in obst):
                        seen = False
            if seen:
                ok = True
                break
        if not ok:
            bad.append(e['code'])
    assert not bad, f'{len(bad)} labels not visible from any coverage pass, e.g. {bad[:6]}'
    print(f'labels: all {len(manifest)} fully visible from a coverage pass (levels {COVER_LEVELS} +-{TOL_Z} m, '
          f'{STANDOFF} m from the face)')


def qr_tile(text, caption):
    p = cv2.QRCodeEncoder_Params()
    p.version = 1
    p.correction_level = cv2.QRCodeEncoder_CORRECT_LEVEL_L
    q = cv2.QRCodeEncoder.create(p).encode(text)  # 25 x 25 incl. 2-module quiet zone
    assert q.shape == (25, 25), (text, q.shape)
    q = cv2.resize(q, None, fx=MODULE_PX, fy=MODULE_PX, interpolation=cv2.INTER_NEAREST)
    tile = np.full((TILE_H, TILE_W), 255, np.uint8)
    x0 = (TILE_W - q.shape[1]) // 2
    tile[2:2 + q.shape[0], x0:x0 + q.shape[1]] = q
    cv2.rectangle(tile, (0, 0), (TILE_W - 1, TILE_H - 1), 0, 1)
    scale = 0.36
    (tw, th), _ = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.putText(tile, caption, ((TILE_W - tw) // 2, TILE_H - 14), cv2.FONT_HERSHEY_SIMPLEX, scale, 0, 1, cv2.LINE_AA)
    return tile


def make_labels():
    """Location + carton labels: models/shelf_labels/meshes/row<r>.{obj,mtl,png}, worlds/shelf_codes.json."""
    rng = random.Random(SKU_SEED)
    cartons = cartons_by_rack()
    labels = {}   # row -> [(code, caption, x, z, face_y, sgn, record)]
    manifest = []
    n_ctn = 0
    carton_ids = {}
    for r, ry, fy, sgn, aisle, side in faces():
        for s, (x0, x1) in enumerate(SEGMENTS, 1):
            nb = max(1, round((x1 - x0) / BAY))
            bay = (x1 - x0) / nb
            face = f'A{aisle}-S{s}-{side[0].upper()}'
            # bays numbered along +x, levels from the floor
            for b in range(1, nb + 1):
                for lv, sz in enumerate(SHELF_Z, 1):
                    code = f'LOC:{face}-B{b:02d}-L{lv}'
                    x, z = x0 + (b - 0.5) * bay, sz + 0.025 + PLATE_H / 2 + 0.003
                    rec = dict(code=code, type='location', aisle=aisle, segment=s, side=side, bay=b, level=lv,
                               x=round(x, 3), y=round(fy + sgn * LABEL_OUT, 3), z=round(z, 3))
                    labels.setdefault(r, []).append((code, f'{face} B{b:02d} L{lv}', x, z, fy + sgn * LABEL_OUT, sgn))
                    manifest.append(rec)
            # cartons of this rack segment (rack index = row * 4 + segment - 1)
            for (cx, cy, cz, sx, sy, szz) in cartons.get(r * 4 + s - 1, []):
                key = (round(cx, 3), round(cy, 3), round(cz, 3))
                if key not in carton_ids:
                    n_ctn += 1
                    carton_ids[key] = (f'CTN-{n_ctn:04d}', f'SKU-{rng.randint(10000, 99999)}')
                cid, sku = carton_ids[key]
                b = min(nb, max(1, int((cx - x0) / bay) + 1))
                lv = min(range(len(SHELF_Z)), key=lambda i: abs(cz - szz / 2 - 0.025 - SHELF_Z[i])) + 1
                y_face = cy + sgn * sy / 2 + sgn * LABEL_OUT
                # above the location label plate of the bay (which sits at the front edge of the shelf), on every
                # level: a label low on a top-level carton set back in the rack is hidden by the shelf's front edge
                # when seen from the top pass, below it
                lx = cx
                z = cz - szz / 2 + min(0.32, szz - PLATE_H / 2 - 0.02)
                code = f'{cid}/{sku}'
                labels.setdefault(r, []).append((code, f'{cid} {sku}', lx, z, y_face, sgn))
                manifest.append(dict(code=code, type='carton', carton=cid, sku=sku, aisle=aisle, segment=s, side=side,
                                     bay=b, level=lv, x=round(lx, 3), y=round(y_face, 3), z=round(z, 3)))
    check_label_visibility(manifest, cartons)

    mdir = 'models/shelf_labels'
    write(f'{mdir}/model.config', L.model_config('shelf_labels', 'QR location / carton labels of the racks, '
                                                 'one atlas + mesh per rack row'))
    write(f'{mdir}/model.sdf', '<?xml version="1.0"?>\n<sdf version="1.9"><model name="shelf_labels"><static>true'
          '</static><link name="link"/></model></sdf>\n')
    os.makedirs(os.path.join(HERE, mdir, 'meshes'), exist_ok=True)
    visuals = ''
    for r, items in sorted(labels.items()):
        cols = math.ceil(math.sqrt(len(items)))
        rows = math.ceil(len(items) / cols)
        atlas = np.full((rows * TILE_H, cols * TILE_W), 255, np.uint8)
        W, H = atlas.shape[1], atlas.shape[0]
        obj = [f'# GENERATED by generate.py: {len(items)} labels of rack row {r}', f'mtllib row{r}.mtl', f'o row{r}']
        vts, fs = [], []
        for i, (code, cap, x, z, fy, sgn) in enumerate(items):
            tr, tc = divmod(i, cols)
            atlas[tr * TILE_H:(tr + 1) * TILE_H, tc * TILE_W:(tc + 1) * TILE_W] = qr_tile(code, cap)
            u0, u1 = (tc * TILE_W + 0.5) / W, ((tc + 1) * TILE_W - 0.5) / W
            v1, v0 = 1 - (tr * TILE_H + 0.5) / H, 1 - ((tr + 1) * TILE_H - 0.5) / H   # OBJ v: up
            hw, hh = PLATE_W / 2, PLATE_H / 2
            # viewer on the normal side: image right = +x for a -y normal, -x for a +y normal
            xl, xr = (x - hw, x + hw) if sgn < 0 else (x + hw, x - hw)
            base = len(vts)
            for (px, pz, u, v) in ((xl, z - hh, u0, v0), (xr, z - hh, u1, v0), (xr, z + hh, u1, v1), (xl, z + hh, u0, v1)):
                obj.append(f'v {px:.4f} {fy:.4f} {pz:.4f}')
                vts.append(f'vt {u:.6f} {v:.6f}')
            n = len(fs) // 1 + 1
            fs.append((base, sgn))
        obj += vts
        obj += ['vn 0 -1 0', 'vn 0 1 0', 'usemtl label']
        for base, sgn in fs:
            nn = 1 if sgn < 0 else 2
            obj.append('f ' + ' '.join(f'{base + j + 1}/{base + j + 1}/{nn}' for j in range(4)))
        write(f'{mdir}/meshes/row{r}.obj', '\n'.join(obj) + '\n')
        write(f'{mdir}/meshes/row{r}.mtl', f'newmtl label\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nd 1.0\nillum 1\n'
                                           f'map_Kd row{r}.png\n')
        cv2.imwrite(os.path.join(HERE, mdir, 'meshes', f'row{r}.png'), atlas)
        visuals += (f'        <visual name="labels_row{r}"><geometry><mesh><uri>model://shelf_labels/meshes/row{r}.obj'
                    f'</uri></mesh></geometry></visual>\n')
    write('worlds/shelf_codes.json', json.dumps({
        'description': 'Ground truth of the QR labels of worlds/warehouse_dynamic.sdf (generate.py). Positions in '
                       'the gz world frame = control frame "world" [m]: centre of the label plate.',
        'qr_size': round(QR_SIZE, 4), 'plate': [round(PLATE_W, 4), round(PLATE_H, 4)],
        'codes': manifest}, indent=1))
    n_loc = sum(1 for m in manifest if m['type'] == 'location')
    print(f'labels: {n_loc} location + {len(manifest) - n_loc} carton labels ({n_ctn} cartons) in {len(labels)} atlases')
    return ('    <model name="shelf_labels">\n      <static>true</static>\n      <link name="link">\n'
            + visuals + '      </link>\n    </model>\n')


# ================================================================================= dynamic objects
def _unwrap(a, b):
    return a + math.atan2(math.sin(b - a), math.cos(b - a))


def actor(name, stops):
    uri = 'model://greenhouse_operator/meshes'
    trajs, tid = [], 0

    def traj(kind, poses):
        nonlocal tid
        wps = ''.join(f'<waypoint><time>{t:.2f}</time><pose>{x:.3f} {y:.3f} {ACTOR_Z} 0 0 {yaw:.4f}</pose></waypoint>'
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
            f = _unwrap(heading, heading if face is None else face)
            traj('stand', [(0, x, y, heading), (TURN_TIME, x, y, f), (TURN_TIME + dwell, x, y, f)])
            heading = f
        traj('stand', [(0, x, y, heading), (TURN_TIME, x, y, _unwrap(heading, nxt))])
        traj('walk', [(0, x, y, nxt), (math.hypot(nx - x, ny - y) / WALK_SPEED, nx, ny, nxt)])
        heading = nxt
    return f'''    <actor name="{name}">
      <pose>{stops[0][0]:.3f} {stops[0][1]:.3f} {ACTOR_Z} 0 0 0</pose>
      <skin><filename>{uri}/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>{uri}/walk.dae</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <animation name="stand"><filename>{uri}/stand.dae</filename><scale>1.0</scale></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
{''.join(trajs)}      </script>
    </actor>
'''


def forklift(name, cfg):
    """Counterbalance forklift from boxes (+x = forks), driven by PathFollower (forks first going out, back
    on the way back: reverse_every_other)."""
    yellow, dark, grey = '0.95 0.70 0.05 1', '0.12 0.12 0.12 1', '0.55 0.55 0.58 1'
    parts = [  # name, x y z, sx sy sz, colour
        ('body', (0.0, 0, 0.55), (1.6, 1.05, 0.8), yellow),
        ('counterweight', (-0.75, 0, 0.6), (0.3, 1.05, 0.7), dark),
        ('roof', (-0.1, 0, 2.1), (1.1, 1.0, 0.06), dark),
        ('post_fl', (0.4, 0.45, 1.5), (0.06, 0.06, 1.2), dark), ('post_fr', (0.4, -0.45, 1.5), (0.06, 0.06, 1.2), dark),
        ('post_rl', (-0.55, 0.45, 1.5), (0.06, 0.06, 1.2), dark), ('post_rr', (-0.55, -0.45, 1.5), (0.06, 0.06, 1.2), dark),
        ('seat', (-0.25, 0, 1.05), (0.45, 0.5, 0.2), dark),
        ('mast_l', (0.9, 0.3, 1.25), (0.08, 0.1, 2.3), grey), ('mast_r', (0.9, -0.3, 1.25), (0.08, 0.1, 2.3), grey),
        ('fork_l', (1.45, 0.25, 0.12), (1.1, 0.12, 0.05), grey), ('fork_r', (1.45, -0.25, 0.12), (1.1, 0.12, 0.05), grey),
        ('pallet', (1.45, 0, 0.21), (1.2, 0.8, 0.13), '0.55 0.40 0.22 1'),
        ('load', (1.45, 0, 0.62), (1.0, 0.75, 0.7), '0.65 0.48 0.28 1'),
    ] + [(f'wheel{i}', (wx, wy, 0.25), (0.5, 0.2, 0.5), dark)
         for i, (wx, wy) in enumerate(((0.55, 0.5), (0.55, -0.5), (-0.5, 0.5), (-0.5, -0.5)))]
    vis = ''.join(f'''        <visual name="{n}"><pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose>
          <geometry><box><size>{s[0]} {s[1]} {s[2]}</size></box></geometry>
          <material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>
''' for n, p, s, c in parts)
    wps = '  '.join(f'{x} {y}' for x, y in cfg['path'])
    x0, y0 = cfg['path'][0]
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x0} {y0} 0 0 0 0</pose>
      <plugin filename="gz_greenhouse_dynamics" name="gz_greenhouse_dynamics::PathFollower">
        <waypoints>{wps}</waypoints><speed>{cfg['speed']}</speed><dwell>{cfg['dwell']}</dwell>
        <turn_rate>0.6</turn_rate><reverse_every_other>true</reverse_every_other>
      </plugin>
      <link name="link">
{vis}      </link>
    </model>
'''


def make_dynamic_world(labels_xml):
    path = os.path.join(HERE, f'worlds/{L.WORLD}.sdf')
    s = open(path).read()
    extra = labels_xml + ''.join(forklift(n, c) for n, c in FORKLIFTS.items()) + \
        ''.join(actor(n, st) for n, st in OPERATORS.items())
    assert s.count('</world>') == 1
    s = s.replace('</world>', extra + '  </world>')
    s = s.replace('<!-- GENERATED by generate.py:', '<!-- GENERATED by px4_sim_ugv_uav_warehouse_dynamic/generate.py '
                  '(+ shelf labels, forklifts, operators) from the light generator:', 1)
    open(path, 'w').write(s)
    print(f'world: + {len(FORKLIFTS)} forklifts, {len(OPERATORS)} operators')


# ================================================================================= session
def patch_env():
    p = os.path.join(HERE, 'env.sh')
    s = open(p).read().replace('shared environment of px4_sim_5ugv_5uav_warehouse_light',
                               'shared environment of px4_sim_ugv_uav_warehouse_dynamic')
    s = s.replace('export ROS_DOMAIN_ID=${UGV_UAV_5PAIRS_DOMAIN_ID:-46}', 'export ROS_DOMAIN_ID=${UGV_UAV_DYN_DOMAIN_ID:-46}')
    s += '''
# PathFollower (forklifts) from ros_packages/gz_greenhouse_dynamics (colcon build --packages-select gz_greenhouse_dynamics)
export IRBL_INSTALL=${IRBL_INSTALL:-$(readlink -f "$(dirname "$(readlink -f ./env.sh)")/../../../../install")}
case ":$GZ_SIM_SYSTEM_PLUGIN_PATH:" in *":$IRBL_INSTALL/gz_greenhouse_dynamics/lib:"*) ;;
  *) export GZ_SIM_SYSTEM_PLUGIN_PATH=$IRBL_INSTALL/gz_greenhouse_dynamics/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH} ;;
esac
# shelf_scanner.py output: $SCAN_DATA_DIR/$SCAN_RUN_ID/ = scans/<start time>/ in this session folder (start.sh sets
# SCAN_RUN_ID once per run)
export SCAN_DATA_DIR=${SCAN_DATA_DIR:-$(dirname "$(readlink -f ./env.sh)")/scans}
export SCAN_RUN_ID=${SCAN_RUN_ID:-$(date +%Y%m%d_%H%M%S)}
'''
    open(p, 'w').write(s)


def patch_session(n):
    p = os.path.join(HERE, 'session.yml')
    s = open(p).read()

    def rep(a, b):
        nonlocal s
        assert a in s, a[:70]
        s = s.replace(a, b, 1)

    rep('# GENERATED by generate.py', '# GENERATED by generate.py (px4_sim_ugv_uav_warehouse_dynamic: the light session + shelf codes,\n'
        '# lateral cameras + shelf_scanner reports, forklifts and walking operators; README.md)')
    # no ros_gz_bridge for the lateral cameras: even with lazy:=true it keeps them subscribed in gz, so gz
    # renders them all the time (10 x 1600x1200 took the 5-pair RTF from ~0.15 to 0.04). shelf_scanner reads
    # them straight from gz-transport and subscribes only while it scans.
    panes = []
    for k in range(1, n + 1):
        cam = lambda c: f'/world/$PX4_GZ_WORLD/model/{L.gz_uav(k)}/link/{c}_cam_link/sensor/{c}_cam'
        panes.append(L.sh_list([
            f'ros2 run rbl_mission shelf_scanner.py --ros-args -r __ns:=/{L.uav(k)} -r __node:=shelf_scanner',
            f'-p use_sim_time:=true -p uav:={L.uav(k)} -p manifest:=$(readlink -f ./worlds/shelf_codes.json)',
            '-p data_dir:=$SCAN_DATA_DIR/$SCAN_RUN_ID',
            f'-p left_gz_topic:={cam("left")} -p right_gz_topic:={cam("right")}',
            f'-p state_topic:=/{L.uav(k)}/warehouse_inspection/state']))
    rep('  - viz:\n', '  - scan:\n      layout: tiled\n      panes:\n'
        '        # per UAV: QR codes of the shelf it covers (lateral cameras) -> $SCAN_DATA_DIR/$SCAN_RUN_ID/ + report\n'
        + '\n'.join(panes) + '\n  - viz:\n')
    import yaml
    yaml.safe_load(s)
    open(p, 'w').write(s)


def patch_rviz(n):
    """+ per UAV: the scanner's annotated lateral camera image and the codes it read (3D markers)."""
    import yaml
    p = os.path.join(HERE, 'ugv_uav_pairs.rviz')
    r = yaml.safe_load(open(p))
    disp = r['Visualization Manager']['Displays']
    for k in range(1, n + 1):
        disp.append({'Class': 'rviz_default_plugins/Image', 'Enabled': True, 'Max Value': 1, 'Median window': 5,
                     'Min Value': 0, 'Name': f'[{k}] QR scan (lateral camera)', 'Normalize Range': True,
                     'Topic': {'Depth': 2, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                               'Reliability Policy': 'Best Effort', 'Value': f'/{L.uav(k)}/shelf_scanner/debug_image'},
                     'Value': True})
        disp.append({'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True, 'Name': f'[{k}] QR codes read',
                     'Namespaces': {}, 'Topic': {'Depth': 5, 'Durability Policy': 'Transient Local',
                                                 'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable',
                                                 'Value': f'/{L.uav(k)}/shelf_scanner/codes'}, 'Value': True})
    open(p, 'w').write(yaml.safe_dump(r, sort_keys=False, width=200))


def make_configs():
    src = open(os.path.join(LIGHT_DIR, 'config/warehouse_map.yaml')).read()
    a = 'levels: [1.0, 2.2, 3.4]'
    assert a in src
    s = src.replace(a, f'levels: {COVER_LEVELS}', 1)
    s = s.replace('world_sdf: worlds/warehouse.sdf', 'world_sdf: worlds/warehouse_dynamic.sdf', 1)
    # the passes must really reach their heights (the lowest / highest label level is at the edge of the view):
    # pass ends counted as reached within 0.5 m, pass-through goals within 0.4 m (was 0.6 / 1.0)
    for a, b in (('tolerance: 0.6 ', 'tolerance: 0.5 '), ('tolerance_pass: 1.0', 'tolerance_pass: 0.4')):
        assert a in s, a
        s = s.replace(a, b, 1)
    # + height: a coverage goal counts only within 0.15 m of its level (else the top pass stayed ~0.4 m low and
    # the labels of the top cartons, at ~3.9 m, were at the steep top edge of the view: missed)
    a = '      tolerance_pass: 0.4'
    assert a in s
    s = s.replace(a, f'      tolerance_z: {TOL_Z}        # [m] coverage goals: also within this of their height\n' + a, 1)
    s = ('# GENERATED by generate.py: ../px4_sim_ugv_uav_warehouse_stereo/config/warehouse_map.yaml with the coverage\n'
         '# pass heights at 0.8 / 2.0 / 3.4 m (the portrait lateral cameras, +-1.0 m vertical view at 1.2 m standoff, see\n'
         '# all 4 label levels at 0.26 / 1.46 / 2.66 / 3.86 m and the labels of the top cartons at ~4.1 m),\n'
         '# mission.tolerance 0.5 / tolerance_pass 0.4 (so the\n'
         '# passes reach those heights) and world_sdf -> worlds/warehouse_dynamic.sdf.\n' + s)
    write('config/warehouse_map.yaml', s)
    us = open(os.path.join(LIGHT_DIR, 'config/uav_sensors.yaml')).read().rstrip() + f'''
  # lateral cameras (shelf codes): camera frame x along the view, left = +y of the body, right = -y, rolled 90 deg
  # about the view axis (portrait: the long image side is vertical)
  - frame_id: left_cam_link
    translation: [0.0, {LATCAM['y']}, {LATCAM['z']}]
    rotation: [{LATCAM['roll']:.4f}, 0.0, 1.5708]
  - frame_id: right_cam_link
    translation: [0.0, {-LATCAM['y']}, {LATCAM['z']}]
    rotation: [{LATCAM['roll']:.4f}, 0.0, -1.5708]
'''
    write('config/uav_sensors.yaml', '# GENERATED by generate.py (../px4_sim_5ugv_5uav_warehouse_light/config + lateral cameras)\n' + us)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--pairs', type=int, default=PAIRS)
    n = ap.parse_args().pairs
    assert 1 <= n <= len(L.SPAWNS)
    L.make_models(n)
    add_lateral_cameras(n)
    fix_cartons()
    L.make_world()
    make_dynamic_world(make_labels())
    L.make_env(n)
    patch_env()
    L.make_session(n)
    patch_session(n)
    L.make_rviz(n)
    patch_rviz(n)
    make_configs()
    print(f'{n} pairs generated')
