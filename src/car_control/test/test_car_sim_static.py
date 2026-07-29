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
WORLD = PACKAGE / "worlds" / "car_empty_test.sdf"


class CarSimulationStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(
            ["xacro", str(XACRO), "controllers_file:=car_controllers.yaml"],
            check=True, capture_output=True, text=True)
        cls.urdf_text = result.stdout
        cls.root = ET.fromstring(result.stdout)
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

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

    def test_world_is_local_and_valid_xml(self):
        world_text = WORLD.read_text(encoding="utf-8")
        ET.fromstring(world_text)
        lowered = world_text.lower()
        for forbidden in ("ht" + "tp:", "ht" + "tps:", "fuel." + "gazebosim"):
            self.assertNotIn(forbidden, lowered)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
