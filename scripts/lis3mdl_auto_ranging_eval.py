# SPDX-FileCopyrightText: 2024
# SPDX-License-Identifier: MIT

import time
import math
import board
from adafruit_lis3mdl import LIS3MDL, Range

print("Starting LIS3MDL Auto-Ranging Linear Encoder Evaluation...")

i2c = board.I2C()
sensor = LIS3MDL(i2c)

# ==========================================
# Configuration & Thresholds (in µT)
# ==========================================
POLE_PITCH_MM = 2.0

# Start wide to prevent clipping on boot if the magnet is close
sensor.range = Range.RANGE_16_GAUSS
current_range = Range.RANGE_16_GAUSS

# Tracking variables
last_phase_rad = 0.0
continuous_phase_rad = 0.0
first_read = True

def update_range(mag_x, mag_z, current_range):
    """Evaluates X and Z against thresholds and updates the sensor range."""
    abs_x = abs(mag_x)
    abs_z = abs(mag_z)
    new_range = current_range

    # 4 GAUSS TIER
    if current_range == Range.RANGE_4_GAUSS:
        if abs_x > 360 or abs_z > 360:
            new_range = Range.RANGE_8_GAUSS

    # 8 GAUSS TIER
    elif current_range == Range.RANGE_8_GAUSS:
        if abs_x > 720 or abs_z > 720:
            new_range = Range.RANGE_12_GAUSS
        elif abs_x < 300 and abs_z < 300:
            new_range = Range.RANGE_4_GAUSS

    # 12 GAUSS TIER
    elif current_range == Range.RANGE_12_GAUSS:
        if abs_x > 1080 or abs_z > 1080:
            new_range = Range.RANGE_16_GAUSS
        elif abs_x < 600 and abs_z < 600:
            new_range = Range.RANGE_8_GAUSS

    # 16 GAUSS TIER
    elif current_range == Range.RANGE_16_GAUSS:
        if abs_x > 1440 or abs_z > 1440:
            # Overrange condition! Cannot shift up anymore.
            pass
        elif abs_x < 900 and abs_z < 900:
            new_range = Range.RANGE_12_GAUSS

    # Apply the shift if a change was triggered
    if new_range != current_range:
        sensor.range = new_range
        range_strings = {
            Range.RANGE_4_GAUSS: "4 Gauss",
            Range.RANGE_8_GAUSS: "8 Gauss",
            Range.RANGE_12_GAUSS: "12 Gauss",
            Range.RANGE_16_GAUSS: "16 Gauss"
        }
        print(f"\n[AUTO-RANGE] Shifted to {range_strings[new_range]}")

    return new_range

# ==========================================
# Main Evaluation Loop
# ==========================================
while True:
    mag_x, mag_y, mag_z = sensor.magnetic

    # 1. Run Auto-Ranging Hysteresis Check
    current_range = update_range(mag_x, mag_z, current_range)

    # 2. Check for physical overrange limits
    if current_range == Range.RANGE_16_GAUSS and (abs(mag_x) > 1440 or abs(mag_z) > 1440):
        print("[WARNING: CLIPPING IMMINENT - INCREASE AIR GAP]")
        time.sleep(0.1)
        continue # Skip position math because data is corrupt

    # 3. Calculate phase
    current_phase_rad = math.atan2(mag_z, mag_x)

    if first_read:
        last_phase_rad = current_phase_rad
        first_read = False

    # 4. Handle wrap-around
    delta_phase = current_phase_rad - last_phase_rad
    if delta_phase > math.pi:
        delta_phase -= 2 * math.pi
    elif delta_phase < -math.pi:
        delta_phase += 2 * math.pi

    continuous_phase_rad += delta_phase
    last_phase_rad = current_phase_rad

    # 5. Calculate Position
    distance_mm = (continuous_phase_rad / (2 * math.pi)) * POLE_PITCH_MM

    print(f"Mag(X): {mag_x:7.1f} | Mag(Z): {mag_z:7.1f} || Position: {distance_mm:8.3f} mm")

    # 50Hz polling loop
    time.sleep(0.02)
