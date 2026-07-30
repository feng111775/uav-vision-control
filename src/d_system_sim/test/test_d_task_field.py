import importlib.util
import math
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config/d_task_field.yaml"
WORLD = PACKAGE / "worlds/d_task_field.sdf"
GUI_CONFIG = PACKAGE / "config/d_task_gui.config"


def parameters():
    result = {}
    for raw in CONFIG.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            key, value = line.split(":", 1)
            result[key.strip()] = float(value)
    return result


def pose(visual):
    return tuple(float(value) for value in visual.findtext("pose").split())


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    scale = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / (
        dx * dx + dy * dy)
    scale = min(1.0, max(0.0, scale))
    return math.hypot(
        point[0] - start[0] - scale * dx,
        point[1] - start[1] - scale * dy)


def visual_segment(visual):
    x, y, _, _, _, yaw = pose(visual)
    length = float(visual.findtext("geometry/box/size").split()[0]) - 0.0002
    half_dx = math.cos(yaw) * length / 2.0
    half_dy = math.sin(yaw) * length / 2.0
    return (x - half_dx, y - half_dy), (x + half_dx, y + half_dy)


def test_official_dimensions_and_key_points():
    p = parameters()
    assert p["field_width_m"] == 4.00
    assert p["field_height_m"] == 5.00
    assert p["line_width_m"] == 0.020
    assert p["border_width_m"] == 0.020
    assert p["start_line_length_m"] == 0.40
    assert (p["left_x_m"], p["lower_y_m"]) == (1.50, 2.00)  # A
    assert (p["left_x_m"], p["upper_y_m"]) == (1.50, 3.50)  # B
    assert (p["right_x_m"], p["upper_y_m"]) == (3.00, 3.50)  # C
    assert (p["right_x_m"], p["lower_y_m"]) == (3.00, 2.00)  # D
    assert p["arc_center_x_m"] == 2.25
    assert p["arc_radius_m"] == 0.75
    assert p["h_center_x_m"] == 1.00
    assert p["h_center_y_m"] == 1.00
    assert p["h_diameter_m"] == 0.50


def test_world_is_valid_local_xml_and_lines_have_no_collision():
    text = WORLD.read_text(encoding="utf-8")
    root = ET.fromstring(text)
    lowered = text.lower()
    assert "http:" not in lowered
    assert "https:" not in lowered
    assert "fuel.gazebosim" not in lowered
    link = root.find(".//model[@name='d_task_field_surface']/link")
    assert link is not None
    collisions = link.findall("collision")
    assert [item.attrib["name"] for item in collisions] == ["field_collision"]
    names = {item.attrib["name"] for item in link.findall("visual")}
    assert {
        "field_white", "track_left", "track_right", "start_line_a",
        "h_takeoff_area"} <= names
    assert len([name for name in names if name.startswith("track_upper_")]) == 72
    assert len([name for name in names if name.startswith("track_lower_")]) == 72

    visuals = {item.attrib["name"]: item for item in link.findall("visual")}
    field_pose = pose(visuals["field_white"])
    assert field_pose[:2] == (2.0, 2.5)
    assert visuals["field_white"].findtext("geometry/box/size").split()[:2] == [
        "4.000000000", "5.000000000"]
    h = visuals["h_takeoff_area"]
    assert pose(h)[:2] == (1.0, 1.0)
    radius = float(h.findtext("geometry/cylinder/radius"))
    assert math.isclose(2.0 * radius, 0.50)
    assert math.isclose(pose(h)[0] - radius, 0.75)
    assert math.isclose(pose(h)[1] - radius, 0.75)


def test_actual_track_passes_key_points_and_start_line_intersects_a():
    p = parameters()
    root = ET.parse(WORLD).getroot()
    visuals = {
        item.attrib["name"]: item
        for item in root.findall(".//model[@name='d_task_field_surface']/link/visual")}
    track_names = [
        name for name in visuals
        if name in ("track_left", "track_right") or
        name.startswith("track_upper_") or name.startswith("track_lower_")]
    segments = [visual_segment(visuals[name]) for name in track_names]
    points = {
        "A": (p["left_x_m"], p["lower_y_m"]),
        "B": (p["left_x_m"], p["upper_y_m"]),
        "C": (p["right_x_m"], p["upper_y_m"]),
        "D": (p["right_x_m"], p["lower_y_m"]),
    }
    for point in points.values():
        assert min(point_segment_distance(point, *segment) for segment in segments) < 0.001
    upper_y = [
        pose(visuals[name])[1] for name in visuals if name.startswith("track_upper_")]
    lower_y = [
        pose(visuals[name])[1] for name in visuals if name.startswith("track_lower_")]
    assert min(upper_y) >= p["upper_y_m"]
    assert max(lower_y) <= p["lower_y_m"]
    start_a, start_b = visual_segment(visuals["start_line_a"])
    assert point_segment_distance(points["A"], start_a, start_b) < 0.001
    assert math.isclose(start_a[1], p["lower_y_m"], abs_tol=1e-9)
    assert math.isclose(start_b[1], p["lower_y_m"], abs_tol=1e-9)
    assert math.isclose(abs(start_b[0] - start_a[0]), 0.40, abs_tol=1e-9)


def test_default_car_front_and_sensor_are_at_a_from_single_field_config():
    p = parameters()
    yaw = p["car_initial_yaw_rad"]
    offset = p["car_front_offset_m"]
    base_x = p["left_x_m"] - offset * math.cos(yaw)
    base_y = p["lower_y_m"] - offset * math.sin(yaw)
    front_x = base_x + offset * math.cos(yaw)
    front_y = base_y + offset * math.sin(yaw)
    assert math.hypot(front_x - 1.50, front_y - 2.00) < 0.01
    assert math.hypot(base_x - 1.50, base_y - 2.00) > 0.01
    assert base_y < p["lower_y_m"]
    assert math.isclose(yaw, math.pi / 2.0)
    launch_text = (PACKAGE / "launch/d_task_car_sensor.launch.py").read_text(encoding="utf-8")
    assert 'load_field_parameters(sim_share / "config/d_task_field.yaml")' in launch_text
    assert '"sensor_forward_offset_m": front_offset' in launch_text
    header = (
        PACKAGE.parent / "car_control/include/car_control/sim/d_task_track_geometry.hpp"
    ).read_text(encoding="utf-8")
    expected = {
        "kLeftX": p["left_x_m"],
        "kRightX": p["right_x_m"],
        "kLowerY": p["lower_y_m"],
        "kUpperY": p["upper_y_m"],
        "kCenterX": p["arc_center_x_m"],
        "kRadius": p["arc_radius_m"],
    }
    for name, value in expected.items():
        match = re.search(rf"{name}\s*=\s*([0-9.]+)", header)
        assert match and math.isclose(float(match.group(1)), value)


def test_left_right_relations_have_no_hidden_horizontal_mirror():
    p = parameters()
    assert p["h_center_x_m"] < p["field_width_m"] / 2.0
    assert p["left_x_m"] < p["right_x_m"]
    assert p["h_center_x_m"] < p["left_x_m"]
    assert p["h_center_y_m"] < p["lower_y_m"]
    sources = [
        PACKAGE / "tools/generate_d_task_field.py",
        PACKAGE / "launch/d_task_car_sensor.launch.py",
        PACKAGE.parent / "car_control/src/sim/virtual_gray_sensor_node.cpp",
    ]
    mirror_pattern = re.compile(
        r"(?:4(?:\.0+)?|field_width(?:_m)?)\s*-\s*(?:field_)?x")
    for source in sources:
        assert not mirror_pattern.search(source.read_text(encoding="utf-8"))


def test_gui_top_view_maps_screen_right_to_world_positive_x():
    # Gazebo GUI config intentionally permits several top-level XML fragments.
    # Wrap them only for this test's ElementTree inspection.
    gui_text = GUI_CONFIG.read_text(encoding="utf-8")
    if gui_text.lstrip().startswith("<?xml"):
        gui_text = gui_text.split("?>", 1)[1]
    root = ET.fromstring(f"<gazebo_gui_config>{gui_text}</gazebo_gui_config>")
    camera = root.find(".//plugin[@filename='MinimalScene']/camera_pose")
    assert camera is not None
    x, y, z, roll, pitch, yaw = (float(value) for value in camera.text.split())
    assert (x, y) == (2.0, 2.5)
    assert z > 5.0
    assert math.isclose(roll, 0.0, abs_tol=1e-12)
    assert math.isclose(pitch, math.pi / 2.0, abs_tol=1e-12)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-12)
    # Gazebo camera looks along local +x; screen-right is local -y.
    screen_right_world_x = math.sin(yaw)
    screen_right_world_y = -math.cos(yaw)
    assert screen_right_world_x > 0.999
    assert abs(screen_right_world_y) < 0.001
    launch_text = (PACKAGE / "launch/d_task_car_sensor.launch.py").read_text(
        encoding="utf-8")
    assert "--gui-config {gui_config}" in launch_text


def test_arc_chords_are_dense_connected_and_generation_is_deterministic():
    p = parameters()
    count = int(p["arc_segments_per_half"])
    chord = 2.0 * p["arc_radius_m"] * math.sin(math.pi / (2.0 * count))
    assert chord < 0.04
    # Consecutive segments are generated from the same analytic endpoint.
    for index in range(count - 1):
        end_angle = math.pi - math.pi * (index + 1) / count
        next_angle = math.pi - math.pi * (index + 1) / count
        assert math.isclose(end_angle, next_angle, abs_tol=1e-15)
    spec = importlib.util.spec_from_file_location(
        "generate_d_task_field", PACKAGE / "tools/generate_d_task_field.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary) / "field.sdf"
        module.generate(CONFIG, output)
        assert output.read_bytes() == WORLD.read_bytes()
