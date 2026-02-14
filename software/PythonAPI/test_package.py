import sys
import os
import logging
from open_micro_manipulator import OpenMicroStageInterface
from open_micro_manipulator.exceptions import DeviceNotFoundError
from open_micro_manipulator.serial_interface import SerialInterface

# Configure logging to see output
logging.basicConfig(level=logging.DEBUG)

def test_mock_interface():
    print("\n--- Testing Mock Interface ---")
    oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True, mock=True)

    # Test Connect
    oms.connect('mock_port')
    print("Connected to mock port.")

    # Test Home
    res = oms.home()
    print(f"Home result: {res}")
    assert res == SerialInterface.ReplyStatus.OK

    # Test Move
    res = oms.move_to(10.0, 20.0, 30.0, f=100)
    print(f"Move result: {res}")
    assert res == SerialInterface.ReplyStatus.OK

    # Test Read Position
    x, y, z = oms.read_current_position()
    print(f"Position: {x}, {y}, {z}")
    # Note: Mock might not update position instantly if moved with buffer logic?
    # But mock implementation updates position in send_command for G0/G1
    assert x == 10.0 and y == 20.0 and z == 30.0

    # Test Calibration
    res, data = oms.calibrate_joint(0, save_result=False)
    print(f"Calibration result: {res}")
    print(f"Calibration data type: {type(data)}")
    assert res == SerialInterface.ReplyStatus.OK
    assert hasattr(data, 'motor_angles')
    assert len(data.motor_angles) > 0

    print("Mock Interface Test Passed.")

def test_real_interface_failure():
    print("\n--- Testing Real Interface Failure ---")
    oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True, mock=False)

    try:
        oms.connect('INVALID_PORT', baud_rate=9600)
        print("Error: Should have failed to connect.")
    except DeviceNotFoundError as e:
        print(f"Caught expected exception: {e}")
    except Exception as e:
        print(f"Caught unexpected exception: {e}")
        raise

    print("Real Interface Failure Test Passed.")

if __name__ == "__main__":
    test_mock_interface()
    test_real_interface_failure()
