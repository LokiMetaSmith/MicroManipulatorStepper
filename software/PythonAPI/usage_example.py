from open_micro_manipulator import OpenMicroStageInterface
from open_micro_manipulator.exceptions import DeviceNotFoundError

# create interface and connect
# Use mock=True for simulation/digital twin
oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True, mock=False)

try:
    oms.connect('/dev/ttyACM0')
except DeviceNotFoundError:
    print("Could not connect to device")

# run this once to calibrate joints
# for i in range(3): oms.calibrate_joint(i, save_result=True)

# home device
oms.home()

# move and wait
oms.move_to(0, 0, 0, f=10)
oms.wait_for_stop()

# print some info
oms.read_device_state_info()