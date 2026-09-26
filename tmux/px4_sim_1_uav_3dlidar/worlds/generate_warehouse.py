#!/usr/bin/env python3
"""Generates warehouse.sdf: a 70 x 36 m warehouse (walls, no roof) full of pallet racks, for
testing rbl_controller with the x500_lidar_3d airframe's 3D lidar in a structured, indoor-like
environment (as opposed to cylinder_forest's scattered trunks).

Layout (x forward, y left, origin = UAV spawn point, in the middle of the y=0 main aisle):
  - 8 rows of racks parallel to x at y = +-2.1, +-6.3, +-10.5, +-14.7 (1.2 m deep, 4.5 m tall),
    leaving 3 m wide aisles between rows (main aisle at y in [-1.5, 1.5]).
  - each row is cut by 3 m wide cross-aisles at x = -16.5, 0, 16.5, giving 4 rack segments per row.
  - racks are open shelving (uprights + 4 shelf planks + random cartons on the planks), so the
    lidar sees through/around them like real racking rather than off a solid wall.
  - loose clutter (pallets with box stacks) and a few columns are dropped into the aisles, keeping
    >= 1.6 m of free width in every aisle and the spawn area clear.

Usage: ./generate_warehouse.py [--seed N] > warehouse.sdf   (the committed warehouse.sdf uses seed 1)
Install: session.yml symlinks worlds/*.sdf into $PX4_DIR/Tools/simulation/gz/worlds automatically.
"""
import argparse
import random

X_MIN, X_MAX = -35.0, 35.0
Y_MIN, Y_MAX = -18.0, 18.0
WALL_H, WALL_T = 6.0, 0.3

RACK_DEPTH, RACK_H = 1.2, 4.5
ROW_YS = [-14.7, -10.5, -6.3, -2.1, 2.1, 6.3, 10.5, 14.7]
CROSS_AISLES = [-16.5, 0.0, 16.5]  # x centers
CROSS_W = 3.0
SHELF_Z = [0.15, 1.35, 2.55, 3.75]
BAY = 2.7  # upright spacing

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="warehouse">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 100</size></plane></geometry>
          <material>
            <ambient>0.55 0.55 0.57 1</ambient>
            <diffuse>0.55 0.55 0.57 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""

FOOTER = "  </world>\n</sdf>\n"

BLUE = (0.10, 0.25, 0.65)
ORANGE = (0.90, 0.45, 0.10)
CARTON = [(0.65, 0.48, 0.28), (0.72, 0.55, 0.33), (0.58, 0.42, 0.24)]
WOOD = (0.55, 0.40, 0.22)
CONCRETE = (0.62, 0.62, 0.64)


def box(name, cx, cy, cz, sx, sy, sz, rgb):
    r, g, b = rgb
    return f"""        <collision name="{name}_c">
          <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="{name}_v">
          <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
"""


def model(name, body):
    return f"""    <model name="{name}">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
"""


def segments():
    """Rack segments along x, split by the cross-aisles: list of (x_start, x_end)."""
    edges = [X_MIN + 2.0] + [v for c in CROSS_AISLES for v in (c - CROSS_W / 2, c + CROSS_W / 2)] + [X_MAX - 2.0]
    return [(edges[i], edges[i + 1]) for i in range(0, len(edges), 2)]


def rack(rng, idx, x0, x1, y):
    length = x1 - x0
    cx = (x0 + x1) / 2
    nbays = max(1, round(length / BAY))
    bay = length / nbays
    body = ""
    # uprights: a thin frame plane across the rack depth at every bay boundary
    for i in range(nbays + 1):
        body += box(f"up{i}", x0 + i * bay, y, RACK_H / 2, 0.08, RACK_DEPTH, RACK_H, BLUE)
    # shelf planks + horizontal beam at the top
    for k, z in enumerate(SHELF_Z):
        body += box(f"sh{k}", cx, y, z, length, RACK_DEPTH, 0.05, ORANGE)
    # cartons on the shelves
    n = 0
    for i in range(nbays):
        for k, z in enumerate(SHELF_Z):
            if rng.random() < 0.35:
                continue
            for _ in range(rng.choice([1, 2])):
                sx = rng.uniform(0.5, 1.1)
                sy = rng.uniform(0.6, RACK_DEPTH - 0.1)
                sz = rng.uniform(0.4, 0.9)
                px = x0 + (i + 0.5) * bay + rng.uniform(-(bay - sx) / 2 + 0.1, (bay - sx) / 2 - 0.1)
                pz = z + 0.025 + sz / 2
                body += box(f"c{n}", px, y + rng.uniform(-0.05, 0.05), pz, sx, sy, sz, rng.choice(CARTON))
                n += 1
    return model(f"rack{idx}", body)


def clutter(rng):
    """Pallets with box stacks and columns in the aisles. Aisle centerlines are y = 0, +-4.2, +-8.4,
    +-12.6, +-16.65 (outer strip); items stay narrow enough to keep >= 1.6 m of free width."""
    out = ""
    aisle_ys = [0.0, 4.2, -4.2, 8.4, -8.4, 12.6, -12.6]
    placed = []
    n = 0
    for ay in aisle_ys:
        for _ in range(rng.randint(2, 3)):
            for _try in range(50):
                x = rng.uniform(X_MIN + 4, X_MAX - 4)
                y = ay + rng.uniform(-0.5, 0.5)
                # keep spawn area clear, and don't sit in a cross-aisle centre (that's where the
                # columns go, and it would block the turn)
                if abs(x) < 3.0 and abs(y) < 3.0:
                    continue
                if any(abs(x - c) < CROSS_W / 2 + 0.6 for c in CROSS_AISLES):
                    continue
                if any((x - px) ** 2 + (y - py) ** 2 < 4.0 for px, py in placed):
                    continue
                placed.append((x, y))
                break
            else:
                continue
            body = box("pallet", x, y, 0.075, 1.2, 0.8, 0.15, WOOD)
            z = 0.15
            for j in range(rng.randint(1, 3)):
                h = rng.uniform(0.35, 0.6)
                body += box(f"b{j}", x, y, z + h / 2, rng.uniform(0.9, 1.1), rng.uniform(0.6, 0.75), h,
                            rng.choice(CARTON))
                z += h
            out += model(f"pallet{n}", body)
            n += 1
    # columns in the cross-aisles, off-centre so a 3 m cross-aisle keeps >= 2 m of free width
    for i, cxa in enumerate(CROSS_AISLES):
        for j, cy in enumerate([-8.4, 4.2, 12.6]):
            if abs(cxa) < 1e-6 and abs(cy) < 4.3 and cy != 4.2:
                continue
            out += model(f"column{i}_{j}", box("col", cxa + 0.8, cy, WALL_H / 2, 0.5, 0.5, WALL_H, CONCRETE))
    return out


def walls():
    xm, ym = (X_MIN + X_MAX) / 2, (Y_MIN + Y_MAX) / 2
    lx, ly = X_MAX - X_MIN + WALL_T, Y_MAX - Y_MIN + WALL_T
    body = box("n", xm, Y_MAX + WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, CONCRETE)
    body += box("s", xm, Y_MIN - WALL_T / 2, WALL_H / 2, lx, WALL_T, WALL_H, CONCRETE)
    body += box("e", X_MAX + WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, CONCRETE)
    body += box("w", X_MIN - WALL_T / 2, ym, WALL_H / 2, WALL_T, ly, WALL_H, CONCRETE)
    return model("walls", body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    rng = random.Random(ap.parse_args().seed)
    out = HEADER + walls()
    i = 0
    for y in ROW_YS:
        for x0, x1 in segments():
            out += rack(rng, i, x0, x1, y)
            i += 1
    out += clutter(rng)
    print(out + FOOTER, end="")


if __name__ == "__main__":
    main()
