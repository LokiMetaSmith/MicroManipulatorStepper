import unittest
from open_micro_manipulator.interface import OpenMicroStageInterface
from open_micro_manipulator.serial_interface import SerialInterface
from open_micro_manipulator.types import DeviceState

class TestInterface(unittest.TestCase):
    def setUp(self):
        self.interface = OpenMicroStageInterface(mock=True, show_communication=False, show_log_messages=False)
        self.interface.connect("mock")

    def tearDown(self):
        self.interface.disconnect()

    def test_read_encoder_angles(self):
        # M51 is already mocked in MockSerialInterface
        angles = self.interface.read_encoder_angles()
        self.assertEqual(len(angles), 3)
        # Mock returns: angle = self.position[i] * 10.0 + (i * 15.0)
        # Position is [0,0,0] initially
        self.assertAlmostEqual(angles[0], 0.0)
        self.assertAlmostEqual(angles[1], 15.0)
        self.assertAlmostEqual(angles[2], 30.0)

    def test_read_device_state_info(self):
        # M57 is now mocked in MockSerialInterface
        state = self.interface.read_device_state_info()

        # Verify it returns a DeviceState object
        self.assertIsInstance(state, DeviceState)

        # Verify content
        self.assertEqual(len(state.joints), 3)

        for i in range(3):
            self.assertTrue(state.joints[i].is_homed)
            self.assertTrue(state.joints[i].is_calibrated)
            self.assertAlmostEqual(state.joints[i].encoder_angle, 0.0) # position is 0 -> angle is 0 in M57 mock

        self.assertEqual(state.servo_loop_rate_khz, 10.0)
        self.assertEqual(state.motion_controller_rate_hz, 1000.0)
        self.assertEqual(state.files_on_flash, ["test_file.dat"])

if __name__ == '__main__':
    unittest.main()
