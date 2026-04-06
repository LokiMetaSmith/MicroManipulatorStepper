# Handover Instructions: CLN-SimpleFOC Dual-Loop & CAN Integration

This document outlines the necessary modifications to the `CLN-SimpleFOC` firmware repository to support the distributed motion control architecture for the Open Micro-Manipulator project.

## Architecture Overview
The main centralized kinematic controller has been decoupled from the low-level PID servo loops. The main controller will now send continuous position target streams over a CAN-FD bus to three independent CLN "Smart Motor" boards.

Each CLN board will execute a **Dual-Loop Control** strategy:
1. **Inner Loop (Velocity/Torque):** Uses the CLN board's built-in rotary magnetic encoder for high-bandwidth commutation and velocity control.
2. **Outer Loop (Position):** Uses a remote **MT6835 Linear Magnetic Encoder** (read via SPI directly on the CLN board) to achieve zero-backlash, sub-micron positioning.

## Task 1: Integrate the MT6835 Linear Encoder (SPI)
You must add a hardware SPI driver for the MT6835 to the CLN-SimpleFOC repository and integrate it into the `SimpleFOC_setup.cpp`.

1. **Port the MT6835 Driver:**
   Extract the `MT6835_encoder.h` and `MT6835_encoder.cpp` files from the `firmware/MotionControllerRP/src/hardware/` directory of the main project and place them in the CLN-SimpleFOC repository.
2. **Initialize SPI:**
   Configure a hardware SPI bus on the CLN board's MCU (e.g., ESP32 or STM32) and map the CS, MISO, MOSI, and SCK pins to the external MT6835 sensor.
3. **Instantiate the Sensor:**
   Create an instance of the MT6835 encoder in `SimpleFOC_setup.cpp`.
   ```cpp
   MT6835Encoder linear_encoder(SPI_BUS, CS_PIN);
   linear_encoder.init();
   ```

## Task 2: Implement Dual-Loop Control in SimpleFOC
Modify the standard SimpleFOC initialization to use a cascaded PID loop.

1. The inner SimpleFOC `motor` object should be configured for **Velocity Control**.
   ```cpp
   motor.controller = MotionControlType::velocity;
   motor.linkSensor(&internal_rotary_encoder); // The built-in CLN encoder
   ```
2. Create an outer `PIDController` for the linear position.
   ```cpp
   PIDController PID_outer_position{10.0, 0.0, 0.1, 0, 50.0}; // Tune these!
   ```
3. In the `loopFOC()` function, calculate the position error using the linear encoder and feed it as the target velocity to the inner loop.
   ```cpp
   // Global variable updated by CAN bus
   float target_linear_pos = 0.0f;

   void loopFOC() {
       // Standard inner FOC loop
       motor.loopFOC();

       // Outer loop: Calculate target velocity based on linear position error
       float current_linear_pos = linear_encoder.read_abs_angle(); // Or converted to mm
       float target_velocity = PID_outer_position(target_linear_pos - current_linear_pos);

       // Send to inner loop
       motor.move(target_velocity);
   }
   ```

## Task 3: Implement CAN Bus Communication (SimpleCANio)
The main controller will stream `target_linear_pos` values over the CAN bus. You must add the `SimpleCANio` library dependency to the CLN firmware.

1. **Add Dependency:** Add `SimpleCANio` to `platformio.ini`.
2. **Initialize CAN:**
   Configure the CAN transceiver in `SimpleFOC_setup.cpp` to run at **1 Mbps**.
   ```cpp
   #include "SimpleCANio.h"
   // (Hardware specific CAN instantiation here based on CLN MCU)
   CANio can(RX_PIN, TX_PIN);
   can.begin(1000000);
   ```
3. **Packet Protocol:**
   The CLN board should listen for CAN messages matching its specific Node ID (e.g., Node 1 = `0x101`, Node 2 = `0x102`, Node 3 = `0x103`).
   The payload will be a 4-byte standard `float` representing the target linear position.
   ```cpp
   void runCommander() {
       if (can.available()) {
           CanMsg msg = can.read();
           if (msg.id == MY_NODE_ID && msg.data_length == 4) {
               memcpy(&target_linear_pos, msg.data, 4);
           }
       }
   }
   ```

## Summary
The final architecture should have the CLN board silently maintaining FOC commutation with its internal sensor while constantly chasing the linear position target it receives over the CAN bus using the external MT6835 sensor.
