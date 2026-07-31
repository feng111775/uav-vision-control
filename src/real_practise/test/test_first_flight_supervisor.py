from pathlib import Path
import unittest

class FirstFlightTests(unittest.TestCase):
    def test_source_has_no_px4_input_publishers(self):
        source = Path(__file__).parents[1].joinpath('real_practise', 'first_flight_supervisor_node.py').read_text()
        self.assertNotIn('/fmu/in/', source)

    def test_real_config_safety_defaults(self):
        text = Path(__file__).parents[1].joinpath('config', 'first_flight_real.yaml').read_text()
        self.assertIn('target_altitude: 0.50', text)
        self.assertIn('hover_test_seconds: 3.0', text)
        self.assertIn('enable_auto_arm: false', text)
        self.assertIn('enable_visual_follow: false', text)
