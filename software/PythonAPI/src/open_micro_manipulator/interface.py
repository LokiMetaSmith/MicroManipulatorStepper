import re
import time
import logging
from enum import Enum
from typing import Optional, List, Tuple
import numpy as np
from .serial_interface import SerialInterface
from .mock_serial_interface import MockSerialInterface
from .types import CalibrationResult, DeviceState, JointState

logger = logging.getLogger(__name__)

class OpenMicroStageInterface:
    def __init__(self, show_communication: bool = True, show_log_messages: bool = True, mock: bool = False):
        self.serial = None
        self.workspace_transform = np.eye(4)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False
        self.mock = mock

    def connect(self, port: str, baud_rate: int = 921600) -> None:
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        if self.serial is not None: self.disconnect()

        if self.mock:
            self.serial = MockSerialInterface(port, baud_rate,
                                          log_msg_callback=self.log_msg_callback,
                                          command_msg_callback=self.command_msg_callback,
                                          unsolicited_msg_callback=self.unsolicited_msg_callback)
        else:
            self.serial = SerialInterface(port, baud_rate,
                                          log_msg_callback=self.log_msg_callback,
                                          command_msg_callback=self.command_msg_callback,
                                          unsolicited_msg_callback=self.unsolicited_msg_callback)

        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        logger.info(f"Firmware version: {version_to_str(fw_version)}")
        if fw_version < min_fw_version:
            logger.error(f"Firmware version {version_to_str(fw_version)} incompatible. "
                         f"At least {version_to_str(min_fw_version)} required")
            self.serial = None
        self.disable_message_callbacks = False

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def log_msg_callback(self, log_level: SerialInterface.LogLevel, msg: str) -> None:
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        level = logging.INFO
        if log_level == SerialInterface.LogLevel.DEBUG: level = logging.DEBUG
        elif log_level == SerialInterface.LogLevel.WARNING: level = logging.WARNING
        elif log_level == SerialInterface.LogLevel.ERROR: level = logging.ERROR

        logger.log(level, f"[{log_level.name}] {msg}")

    def command_msg_callback(self, msg: str, reply_status: SerialInterface.ReplyStatus, error_msg: str) -> None:
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                for line in msg.splitlines():
                    logger.debug(f"> {line}")
            if error_msg:
                logger.debug(f"{reply_status.name}: {error_msg}")
            else:
                logger.debug(f"{reply_status.name}")
        else:
            logger.debug(f"{msg.strip()}")

    def unsolicited_msg_callback(self, msg: str) -> None:
        logger.info(msg)

    def set_workspace_transform(self, transform: np.ndarray) -> None:
        self.workspace_transform = transform

    def get_workspace_transform(self) -> np.ndarray:
        return self.workspace_transform

    def read_firmware_version(self) -> Tuple[int, int, int]:
        ok, response = self.serial.send_command("M58")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0, 0, 0

        major, minor, patch = map(int, re.match(r'v(\d+)\.(\d+)\.(\d+)', response).groups())
        return major,minor,patch

    def home(self, axis_list: Optional[List[int]] = None) -> SerialInterface.ReplyStatus:
        """
        Homes one or more axes on the device
        :param axis_list: Optional list of axis indices to home. If None, all axes are homed.
        :return: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        cmd = 'G28'
        axis_chars = ['A', 'B', 'C', 'D', 'E', 'F']
        if axis_list is None:
            axis_list = [i for i in range(len(axis_chars))]

        for axis_idx in axis_list:
            if axis_idx < 0 or axis_idx >= len(axis_chars):
                raise ValueError('Axis index out of range')
            cmd += ' '+axis_chars[axis_idx]

        res, msg = self.serial.send_command(cmd + "\n", 10)
        return res

    def calibrate_joint(self, joint_index: int, save_result: bool) -> Tuple[SerialInterface.ReplyStatus, CalibrationResult]:
        """
        Calibrates the given joint and returns the measured data as a CalibrationResult object.
        :param joint_index:
        :param save_result:
        :return:
        """
        cmd = f"M56 J{joint_index} P"
        if save_result: cmd += ' S'
        res, msg = self.serial.send_command(cmd, 30)

        calibration_data = self._parse_table_data(msg, 3)
        result = CalibrationResult(
            motor_angles=calibration_data[0],
            field_angles=calibration_data[1],
            raw_encoder_counts=calibration_data[2]
        )
        return res, result

    def move_to(self, x: float, y: float, z: float, f: float, move_immediately: bool = False, blocking: bool = True, timeout: float = 1) -> SerialInterface.ReplyStatus:
        """
        Moves the stage to an absolute position with a specified feed rate.
        :param x: Target X position (in workspace coordinates).
        :param y: Target Y position (in workspace coordinates).
        :param z: Target Z position (in workspace coordinates).
        :param f: Feed rate in mm/s.
        :param move_immediately: If True, execution starts without buffering delay.
        :param blocking: If True, waits and retries if the device is busy. If False, returns immediately on 'BUSY'.
        :param timeout: Timeout in seconds for each command attempt.
        :return: Status of the move command (e.g. OK, ERROR, BUSY, TIMEOUT).
        """
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f} F{f:.3f}"
        if move_immediately:
            cmd += " I"

        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def dwell(self, time_s: float, blocking: bool, timeout: float = 1) -> SerialInterface.ReplyStatus:
        cmd = f"G4 S{time_s:.6f}\n"
        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def set_max_acceleration(self, linear_accel: float, angular_accel: float) -> SerialInterface.ReplyStatus:
        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self.serial.send_command(cmd)
        return res

    def wait_for_stop(self, polling_interval_ms: int = 10, disable_callbacks: bool = True) -> SerialInterface.ReplyStatus:
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks: self.disable_message_callbacks = True

        try:
            while True:
                res, msg = self.serial.send_command("M53\n")
                if res != SerialInterface.ReplyStatus.OK: return res
                elif msg.strip() == "1":
                    return SerialInterface.ReplyStatus.OK
                time.sleep(polling_interval_ms / 1000.0)
        finally:
            if disable_callbacks:
                self.disable_message_callbacks = disable_message_callbacks_prev

    def read_current_position(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        ok, response = self.serial.send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        # Match values with NO space between axis letter and number
        match = re.search(
            r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)",
            response
        )
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        return float(x), float(y), float(z)

    def read_encoder_angles(self) -> List[float]:
        ok, response = self.serial.send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []

        angles = []
        for line in response.splitlines():
            # Example format: "Joint 0:  123.456 deg  (raw=789.012)"
            match = re.search(r"Joint\s+\d+:\s+([-+]?\d*\.?\d+)\s+deg", line)
            if match:
                angles.append(float(match.group(1)))
        return angles

    def read_device_state_info(self) -> Optional[DeviceState]:
        res, msg = self.serial.send_command("M57")
        if res != SerialInterface.ReplyStatus.OK:
            return None

        joints = []
        servo_loop_rate = 0.0
        motion_controller_rate = 0.0
        files = []
        reading_files = False

        for line in msg.splitlines():
            stripped = line.strip()
            if not stripped: continue

            if stripped == "ok":
                break

            if stripped.startswith("Joint"):
                # Example: "Joint 0:  is_homed=1  is_calibrated=1  encoder_angle=123.45 deg"
                is_homed = "is_homed=1" in stripped
                is_calibrated = "is_calibrated=1" in stripped

                angle_match = re.search(r"encoder_angle=([-+]?\d*\.?\d+)", stripped)
                angle = float(angle_match.group(1)) if angle_match else 0.0

                joints.append(JointState(is_homed, is_calibrated, angle))

            elif stripped.startswith("Servo Loop:"):
                # "Servo Loop: 10 kHz"
                match = re.search(r"Servo Loop:\s+(\d+)", stripped)
                if match:
                    servo_loop_rate = float(match.group(1))

            elif stripped.startswith("Motion Controler:"):
                # "Motion Controler: 1000 Hz"
                match = re.search(r"Motion Controler:\s+(\d+)", stripped)
                if match:
                    motion_controller_rate = float(match.group(1))

            elif stripped.startswith("Files on flash:"):
                reading_files = True

            elif reading_files:
                files.append(stripped)

        return DeviceState(joints, servo_loop_rate, motion_controller_rate, files)

    def set_servo_parameter(self, pos_kp: float = 150, pos_ki: float = 50000, vel_kp: float = 0.2, vel_ki: float = 100, vel_filter_tc: float = 0.0025) -> SerialInterface.ReplyStatus:
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def enable_motors(self, enable: bool) -> SerialInterface.ReplyStatus:
        cmd = f"M17" if enable else "M18"
        res, msg = self.serial.send_command(cmd, timeout=5)
        return res

    def set_pose(self, x: float, y: float, z: float) -> SerialInterface.ReplyStatus:
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f}" # TODO: A, B ,C
        res, msg = self.serial.send_command(cmd)
        return res

    def send_command(self, cmd: str, timeout_s: float = 5) -> Tuple[SerialInterface.ReplyStatus, str]:
        res, msg = self.serial.send_command(cmd, timeout_s)
        return res, msg

    @staticmethod
    def _parse_table_data(data_string: str, cols: int) -> List[List[float]]:
        # Parse the data
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) != cols:
                continue  # skip malformed lines
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data
