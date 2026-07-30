import importlib.util
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config/d_task_field.yaml"
WORLD = PACKAGE / "worlds/d_task_field.sdf"


def parameters():
    result = {}
    for raw in CONFIG.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            key, value = line.split(":", 1)
            result[key.strip()] = float(value)
    return result


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
    assert {"field_white", "track_left", "track_right", "start_line_a"} <= names
    assert len([name for name in names if name.startswith("track_upper_")]) == 72
    assert len([name for name in names if name.startswith("track_lower_")]) == 72


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
