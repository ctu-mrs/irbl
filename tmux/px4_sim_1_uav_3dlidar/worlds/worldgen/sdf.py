"""SDF building blocks shared by every world generator.

Everything is built from static primitives (box / cylinder / sphere): cheap for the physics engine,
and exactly what the gpu_lidar renders, so what the lidar sees is what the UAV can collide with.
Purely cosmetic details (lane markings, window bands, ...) use visual-only shapes (collide=False).
"""
import math

HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{name}">
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
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>{bg[0]} {bg[1]} {bg[2]} 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size[0]} {ground_size[1]}</size></plane></geometry>
          <material>
            <ambient>{ground[0]} {ground[1]} {ground[2]} 1</ambient>
            <diffuse>{ground[0]} {ground[1]} {ground[2]} 1</diffuse>
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


def header(name, ground=(0.55, 0.55, 0.57), ground_size=(200, 100), background=(0.7, 0.7, 0.7), ambient=0.5):
    return HEADER.format(name=name, ground=ground, ground_size=ground_size, bg=background, ambient=ambient)


def _rot(rpy):
    if not any(rpy):
        return "0 0 0"
    return f"{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}"


def shape(name, geom, cx, cy, cz, rgb, rpy=(0, 0, 0), collide=True, emissive=None):
    """One collision+visual pair (or visual only) inside a link. `geom` is the inner <geometry> xml."""
    r, g, b = rgb
    pose = f"<pose>{cx:.3f} {cy:.3f} {cz:.3f} {_rot(rpy)}</pose>"
    out = ""
    if collide:
        out += f"""        <collision name="{name}_c">
          {pose}
          <geometry>{geom}</geometry>
        </collision>
"""
    emit = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    out += f"""        <visual name="{name}_v">
          {pose}
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>{emit}
          </material>
        </visual>
"""
    return out


def box(name, cx, cy, cz, sx, sy, sz, rgb, rpy=(0, 0, 0), **kw):
    return shape(name, f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>", cx, cy, cz, rgb, rpy, **kw)


def cylinder(name, cx, cy, cz, radius, length, rgb, rpy=(0, 0, 0), **kw):
    return shape(name, f"<cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder>",
                 cx, cy, cz, rgb, rpy, **kw)


def sphere(name, cx, cy, cz, radius, rgb, **kw):
    return shape(name, f"<sphere><radius>{radius:.3f}</radius></sphere>", cx, cy, cz, rgb, **kw)


def leaning_cylinder(name, x, y, z0, radius, length, tilt, heading, rgb, **kw):
    """Cylinder whose BASE is at (x, y, z0), leaning `tilt` rad away from vertical towards `heading`.
    Returns (xml, top_xyz) so callers can stack things on its tip."""
    ax = (math.sin(tilt) * math.cos(heading), math.sin(tilt) * math.sin(heading), math.cos(tilt))
    c = [p + a * length / 2 for p, a in zip((x, y, z0), ax)]
    top = tuple(p + a * length for p, a in zip((x, y, z0), ax))
    return cylinder(name, *c, radius, length, rgb, rpy=(0, tilt, heading), **kw), top


def lying_cylinder(name, cx, cy, radius, length, heading, rgb, **kw):
    """Horizontal cylinder (log, pipe) resting on the ground, axis along `heading`."""
    return cylinder(name, cx, cy, radius, radius, length, rgb, rpy=(0, math.pi / 2, heading), **kw)


def model(name, body):
    return f"""    <model name="{name}">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
"""


def jitter(rgb, rng, amount=0.05):
    """Slightly vary a colour so repeated objects don't look copy-pasted."""
    return tuple(round(min(1.0, max(0.0, c + rng.uniform(-amount, amount))), 3) for c in rgb)
