#include <Wire.h>

// -----------------------------------------------------------------------------
// Base Interface for Magnet Sensors
// -----------------------------------------------------------------------------
class MagnetSensor {
public:
    virtual ~MagnetSensor() = default;

    // Initialize the sensor (e.g., begin communication)
    virtual void init() = 0;

    // Read the raw absolute angle count, including unwrapping logic
    virtual int32_t read_abs_angle_raw() = 0;

    // Read the absolute angle in radians
    virtual float read_abs_angle() = 0;
};

// -----------------------------------------------------------------------------
// MT6701 Sensor Implementation (I2C)
// -----------------------------------------------------------------------------
class MT6701Sensor : public MagnetSensor {
public:
    using AbsRawAngleType = int32_t;

    MT6701Sensor(TwoWire& wire, uint8_t i2c_addr = 0x06)
        : wire(wire), address(i2c_addr) {
    }

    void init() override {
        wire.begin();
        last_raw_angle = 0;
        abs_raw_angle = 0;
    }

    int32_t read_abs_angle_raw() override {
        wire.beginTransmission(address);
        wire.write(0x03); // ANGLE_H register
        wire.endTransmission(false);

        wire.requestFrom(address, (uint8_t)2);
        if (wire.available() < 2) return -1; // return an error code if not available

        uint8_t angle_h = wire.read();
        uint8_t angle_l = wire.read();

        // 14-bit value: [13:6] from ANGLE_H, [5:0] from ANGLE_L (shifted down by 2)
        int32_t raw_angle = (angle_h << 6) | (angle_l >> 2);
        return update_abs_raw_angle(raw_angle);
    }

    float read_abs_angle() override {
        int32_t raw = read_abs_angle_raw();
        return float(raw) * RAW_TO_RAD;
    }

    void set_hysteresis(uint8_t hyst) {
        if (hyst > 7) return;

        uint8_t hyst2 = (hyst >> 2) & 0x01;
        uint8_t hyst10 = hyst & 0x03;

        // --- Register 0x32 ---
        wire.beginTransmission(address);
        wire.write(0x32);
        wire.endTransmission(false);
        wire.requestFrom(address, (uint8_t)1);
        uint8_t reg32 = wire.read();
        reg32 = (reg32 & 0x7F) | (hyst2 << 7);

        wire.beginTransmission(address);
        wire.write(0x32);
        wire.write(reg32);
        wire.endTransmission();

        // --- Register 0x34 ---
        wire.beginTransmission(address);
        wire.write(0x34);
        wire.endTransmission(false);
        wire.requestFrom(address, (uint8_t)1);
        uint8_t reg34 = wire.read();
        reg34 = (reg34 & 0x3F) | (hyst10 << 6);

        wire.beginTransmission(address);
        wire.write(0x34);
        wire.write(reg34);
        wire.endTransmission();
    }

    // The calculation from the original firmware to accumulate multi-turn rotation
    AbsRawAngleType update_abs_raw_angle(int32_t raw_angle) {
        if (raw_angle >= 0) {
            int32_t half_max = CPR >> 1;
            int32_t d = raw_angle - last_raw_angle;

            if (d > half_max) d -= CPR;
            else if (d < -half_max) d += CPR;

            abs_raw_angle += d;
            last_raw_angle = raw_angle;
        }

        return abs_raw_angle;
    }

private:
    TwoWire& wire;
    uint8_t address;

    AbsRawAngleType abs_raw_angle = 0;
    int32_t last_raw_angle = 0;

    static constexpr int32_t CPR = 16384; // 14-bit resolution
    static constexpr float RAW_TO_RAD = 2 * PI / CPR;
};

// -----------------------------------------------------------------------------
// Arduino Main Sketch
// -----------------------------------------------------------------------------

// You can easily swap this instance out with a different implementation of MagnetSensor
MT6701Sensor sensor(Wire);

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect
    }

    Serial.println("Magnet Sensor Test Script");
    Serial.println("Initializing sensor...");

    sensor.init();

    // Optional: configuring hysteresis matching the original firmware defaults if needed
    // sensor.set_hysteresis(0x04);
}

void loop() {
    // Read the current absolute angle in raw counts (including unwrapped multi-turn)
    int32_t raw_angle = sensor.read_abs_angle_raw();

    // Read the unwrapped absolute angle in radians
    float angle_rad = sensor.read_abs_angle();
    float angle_deg = angle_rad * (180.0 / PI);

    // Output formatting for Serial Plotter / Monitor
    Serial.print("Raw Counts: ");
    Serial.print(raw_angle);
    Serial.print(", Angle(deg): ");
    Serial.println(angle_deg, 2);

    // Adjust delay as needed depending on your required sample rate
    delay(50);
}
