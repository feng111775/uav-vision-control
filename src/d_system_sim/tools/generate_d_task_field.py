#!/usr/bin/env python3
"""Generate the deterministic, fully local Gazebo Harmonic D-task field."""

import math
from pathlib import Path
import sys


def load_flat_yaml(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = float(value.strip())
    return values


def visual_box(name, x, y, z, sx, sy, sz, yaw, color):
    return f"""      <visual name="{name}">
        <pose>{x:.9f} {y:.9f} {z:.6f} 0 0 {yaw:.12f}</pose>
        <geometry><box><size>{sx:.9f} {sy:.9f} {sz:.6f}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        <cast_shadows>false</cast_shadows>
      </visual>"""


def visual_cylinder(name, x, y, z, radius, length, color):
    return f"""      <visual name="{name}">
        <pose>{x:.9f} {y:.9f} {z:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius:.9f}</radius><length>{length:.6f}</length></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        <cast_shadows>false</cast_shadows>
      </visual>"""


def segment(name, ax, ay, bx, by, width):
    dx, dy = bx - ax, by - ay
    return visual_box(
        name, (ax + bx) / 2, (ay + by) / 2, 0.001,
        math.hypot(dx, dy) + 0.0002, width, 0.001,
        math.atan2(dy, dx), "0.01 0.01 0.01 1")


def generate(config_path, output_path):
    p = load_flat_yaml(config_path)
    width, height = p["field_width_m"], p["field_height_m"]
    line_width, border = p["line_width_m"], p["border_width_m"]
    lx, rx = p["left_x_m"], p["right_x_m"]
    low, high = p["lower_y_m"], p["upper_y_m"]
    cx, radius = p["arc_center_x_m"], p["arc_radius_m"]
    count = int(p["arc_segments_per_half"])
    visuals = [
        visual_box("field_white", width / 2, height / 2, -0.005,
                   width, height, 0.01, 0, "0.95 0.95 0.95 1"),
        segment("border_bottom", 0, 0, width, 0, border),
        segment("border_top", 0, height, width, height, border),
        segment("border_left", 0, 0, 0, height, border),
        segment("border_right", width, 0, width, height, border),
        segment("track_left", lx, low, lx, high, line_width),
        segment("track_right", rx, low, rx, high, line_width),
        segment("start_line_a", lx - p["start_line_length_m"] / 2, low,
                lx + p["start_line_length_m"] / 2, low, line_width),
    ]
    for half, center_y, start_angle in (
        ("upper", high, math.pi),
        ("lower", low, 0.0),
    ):
        for index in range(count):
            a0 = start_angle - math.pi * index / count
            a1 = start_angle - math.pi * (index + 1) / count
            visuals.append(segment(
                f"track_{half}_{index:03d}",
                cx + radius * math.cos(a0), center_y + radius * math.sin(a0),
                cx + radius * math.cos(a1), center_y + radius * math.sin(a1),
                line_width))
    # Fixed UAV takeoff / landing circle from the official field dimensions.
    visuals.append(visual_cylinder(
        "h_takeoff_area", p["h_center_x_m"], p["h_center_y_m"], 0.001,
        p["h_diameter_m"] / 2.0, 0.001, "0.55 0.55 0.55 1"))
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="d_task_field">
    <gravity>0 0 -9.8</gravity>
    <physics name="step" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene><ambient>0.8 0.8 0.8 1</ambient><background>0.7 0.8 0.9 1</background></scene>
    <light name="sun" type="directional">
      <pose>2 2 10 0 0 0</pose><direction>-0.2 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
    </light>
    <model name="d_task_field_surface">
      <static>true</static><link name="field_link">
        <collision name="field_collision">
          <pose>{width / 2:.3f} {height / 2:.3f} -0.010 0 0 0</pose>
          <geometry><box><size>{width:.3f} {height:.3f} 0.020</size></box></geometry>
        </collision>
{chr(10).join(visuals)}
      </link>
    </model>
  </world>
</sdf>
"""
    output_path.write_text(sdf, encoding="utf-8")


if __name__ == "__main__":
    base = Path(__file__).resolve().parents[1]
    config = Path(sys.argv[1]) if len(sys.argv) > 1 else base / "config/d_task_field.yaml"
    output = Path(sys.argv[2]) if len(sys.argv) > 2 else base / "worlds/d_task_field.sdf"
    generate(config, output)
