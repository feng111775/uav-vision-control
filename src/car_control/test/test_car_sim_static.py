#!/usr/bin/env python3
import math
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(sys.argv[1]).resolve()
XACRO = PACKAGE / "urdf" / "car_sim.urdf.xacro"
CONFIG = PACKAGE / "config" / "car_controllers.yaml"
SENSOR_CONFIG = PACKAGE / "config" / "virtual_gray_sensor.yaml"
FIELD_CONFIG = PACKAGE.parent / "d_system_sim" / "config" / "d_task_field.yaml"
WORLD = PACKAGE / "worlds" / "car_empty_test.sdf"
DESCRIPTION_HELPER = PACKAGE / "python" / "car_control_launch" / "robot_description.py"
BASIC_LAUNCH = PACKAGE / "launch" / "car_basic_sim.launch.py"
D_TASK_LAUNCH = PACKAGE.parent / "d_system_sim" / "launch" / "d_task_car_sensor.launch.py"


class CarSimulationStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(
            ["xacro", str(XACRO), "controllers_file:=car_controllers.yaml"],
            check=True, capture_output=True, text=True)
        cls.urdf_text = result.stdout
        cls.root = ET.fromstring(result.stdout)
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.sensor_config = yaml.safe_load(
            SENSOR_CONFIG.read_text(encoding="utf-8"))["virtual_gray_sensor_node"]["ros__parameters"]
        cls.field_config = yaml.safe_load(FIELD_CONFIG.read_text(encoding="utf-8"))

    def test_required_joints_and_control_plugin(self):
        joints = {node.attrib["name"]: node for node in self.root.findall("joint")}
        self.assertIn("left_wheel_joint", joints)
        self.assertIn("right_wheel_joint", joints)
        control = self.root.find("ros2_control")
        self.assertIsNotNone(control)
        self.assertEqual(
            control.findtext("hardware/plugin"), "gz_ros2_control/GazeboSimSystem")
        self.assertNotIn("gazebo_" + "ros2_control", self.urdf_text)
        self.assertIn("libgz_ros2_control-system.so", self.urdf_text)

    def test_wheel_geometry_and_axes(self):
        joints = {node.attrib["name"]: node for node in self.root.findall("joint")}
        left_y = float(joints["left_wheel_joint"].find("origin").attrib["xyz"].split()[1])
        right_y = float(joints["right_wheel_joint"].find("origin").attrib["xyz"].split()[1])
        self.assertGreater(left_y, 0.0)
        self.assertLess(right_y, 0.0)
        self.assertEqual(
            joints["left_wheel_joint"].find("axis").attrib["xyz"], "0 1 0")
        self.assertEqual(
            joints["right_wheel_joint"].find("axis").attrib["xyz"], "0 1 0")
        params = self.config["diff_drive_controller"]["ros__parameters"]
        self.assertGreater(float(params["wheel_radius"]), 0.0)
        self.assertGreater(float(params["wheel_separation"]), 0.0)
        self.assertAlmostEqual(left_y - right_y, float(params["wheel_separation"]))

    def test_inertias_are_finite_and_positive(self):
        inertials = self.root.findall(".//inertial")
        self.assertGreaterEqual(len(inertials), 5)
        for inertial in inertials:
            mass = float(inertial.find("mass").attrib["value"])
            self.assertTrue(math.isfinite(mass) and mass > 0.0)
            inertia = inertial.find("inertia")
            for key in ("ixx", "iyy", "izz"):
                value = float(inertia.attrib[key])
                self.assertTrue(math.isfinite(value) and value > 0.0)

    def test_controller_joint_names_match(self):
        control_joints = {
            node.attrib["name"] for node in self.root.findall("ros2_control/joint")}
        params = self.config["diff_drive_controller"]["ros__parameters"]
        configured = set(params["left_wheel_names"] + params["right_wheel_names"])
        self.assertEqual(control_joints, configured)
        self.assertFalse(params["open_loop"])
        self.assertTrue(params["position_feedback"])

    def test_front_reference_matches_physical_front_and_sensor(self):
        chassis = self.root.find("link[@name='chassis_link']/collision/geometry/box")
        chassis_length = float(chassis.attrib["size"].split()[0])
        joint = self.root.find("joint[@name='car_front_reference_joint']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.find("parent").attrib["link"], "base_link")
        front_offset = float(joint.find("origin").attrib["xyz"].split()[0])
        self.assertAlmostEqual(front_offset, chassis_length / 2.0)
        self.assertAlmostEqual(
            front_offset, float(self.sensor_config["sensor_forward_offset_m"]))
        self.assertAlmostEqual(
            front_offset, float(self.field_config["car_front_offset_m"]))
        yaw = float(self.field_config["car_initial_yaw_rad"])
        base_x = float(self.sensor_config["initial_world_x"])
        base_y = float(self.sensor_config["initial_world_y"])
        self.assertAlmostEqual(
            base_x + front_offset * math.cos(yaw),
            float(self.field_config["left_x_m"]))
        self.assertAlmostEqual(
            base_y + front_offset * math.sin(yaw),
            float(self.field_config["lower_y_m"]))

    def test_world_is_local_and_valid_xml(self):
        world_text = WORLD.read_text(encoding="utf-8")
        ET.fromstring(world_text)
        lowered = world_text.lower()
        for forbidden in ("ht" + "tp:", "ht" + "tps:", "fuel." + "gazebosim"):
            self.assertNotIn(forbidden, lowered)

    def test_launches_share_explicit_string_robot_description(self):
        helper = DESCRIPTION_HELPER.read_text(encoding="utf-8")
        self.assertIn("ParameterValue(content, value_type=str)", helper)
        self.assertIn("content = Command(", helper)
        for launch in (BASIC_LAUNCH, D_TASK_LAUNCH):
            text = launch.read_text(encoding="utf-8")
            self.assertIn(
                "from car_control_launch.robot_description import "
                "make_robot_description", text)
            self.assertIn("description = make_robot_description(", text)
            self.assertNotIn('"robot_description": Command(', text)
        self.assertIn("<robot", self.urdf_text)
        ET.fromstring(self.urdf_text)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
