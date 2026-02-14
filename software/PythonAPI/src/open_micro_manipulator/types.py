from dataclasses import dataclass
from typing import List

@dataclass
class CalibrationResult:
    motor_angles: List[float]
    field_angles: List[float]
    raw_encoder_counts: List[float]
