from pathlib import Path
import unittest

class FirstFlightTests(unittest.TestCase):
    def test_source_has_no_px4_input_publishers(self):
        source = Path(__file__).parents[1].joinpath('real_practise', 'first_flight_supervisor_node.py').read_text()
        self.assertNotIn('/fmu/in/', source)
        self.assertIn("'/uav/mission/state'", source)
        self.assertIn('WAIT_START', source)
        self.assertIn('/real_practise/start', source)

    def test_real_config_safety_defaults(self):
        text = Path(__file__).parents[1].joinpath('config', 'first_flight_real.yaml').read_text()
        self.assertIn('target_altitude: 0.50', text)
        self.assertIn('hover_test_seconds: 3.0', text)
        self.assertIn('dry_run_px4_commands: false', text)
        self.assertIn('enable_auto_arm: false', text)
        self.assertIn('enable_visual_follow: false', text)

    def test_supervisor_accepts_both_px4_local_position_topic_names(self):
        source = Path(__file__).parents[1].joinpath('real_practise', 'first_flight_supervisor_node.py').read_text()
        self.assertIn('/fmu/out/vehicle_local_position', source)
        self.assertIn('/fmu/out/vehicle_local_position_v1', source)

    def test_supervisor_uses_px4_v1_16_disarmed_constant(self):
        source = Path(__file__).parents[1].joinpath('real_practise', 'first_flight_supervisor_node.py').read_text()
        self.assertIn('ARMING_STATE_DISARMED', source)
        self.assertNotIn('ARMING_STATE_STANDBY', source)
