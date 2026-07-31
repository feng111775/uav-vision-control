from pathlib import Path
import unittest


class FirstFlightLaunchTests(unittest.TestCase):
    def test_real_launch_keeps_first_flight_chain_minimal(self):
        text = Path(__file__).parents[1].joinpath(
            'launch', 'first_flight_real.launch.py').read_text()
        self.assertIn("mission_controller_node", text)
        self.assertIn("first_flight_supervisor_node", text)
        self.assertNotIn("visual_servo_node", text)
        self.assertNotIn("vision_interface_node", text)
        self.assertNotIn("MicroXRCEAgent", text)
        self.assertNotIn("uxrce_dds_client start", text)

    def test_real_launch_disables_all_non_hover_features(self):
        text = Path(__file__).parents[1].joinpath(
            'launch', 'first_flight_real.launch.py').read_text()
        for expected in (
            "'enable_auto_arm': False",
            "'enable_visual_follow': False",
            "'enable_payload_release': False",
            "'enable_dynamic_landing': False",
            "'enable_second_takeoff': False",
            "'dry_run_px4_commands': False",
        ):
            self.assertIn(expected, text)

    def test_bench_launch_uses_dry_run_vehicle_commands(self):
        text = Path(__file__).parents[1].joinpath(
            'launch', 'first_flight_bench.launch.py').read_text()
        self.assertIn("'dry_run_px4_commands': True", text)
        self.assertNotIn("vision_interface_node", text)
