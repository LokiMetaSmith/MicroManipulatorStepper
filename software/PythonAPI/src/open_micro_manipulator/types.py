from dataclasses import dataclass
from typing import List

@dataclass
class CalibrationResult:
    motor_angles: List[float]
    field_angles: List[float]
    raw_encoder_counts: List[float]

@dataclass
class JointState:
    is_homed: bool
    is_calibrated: bool
    encoder_angle: float

@dataclass
class DeviceState:
    joints: List[JointState]
    servo_loop_rate_khz: float
    motion_controller_rate_hz: float
    files_on_flash: List[str]
