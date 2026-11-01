## Archived: 2026-10-31

* Standalone Arduino sketches (e.g., test scripts) can be compiled via the command line using `arduino-cli` (e.g., `arduino-cli compile --fqbn arduino:avr:uno <path_to_ino>`).
* Python API tests are located in the `software/PythonAPI/tests` directory.
* Utility scripts (such as those for exporting FreeCAD files) should be placed in the `scripts/` directory at the project root.
* The main controller communicates over CAN-FD using an MCP2518FD SPI-to-CAN-FD controller and the ACAN2517FD library. Remote CLN-SimpleFOC nodes use STM32 microcontrollers with built-in FD-CAN.
* When generating individual STL files from FreeCAD models, composite/assembly files (like `Assembly_MicroManipulator.FCStd`) should be excluded (whitelisted approach) to prevent duplication of parts.
* The main C++ firmware (`firmware/MotionControllerRP`) is built for the Raspberry Pi Pico (RP2040/RP2350) using PlatformIO with the Arduino framework. Compile it using `pio run -e pico -d firmware/MotionControllerRP`.
* The Python API communicates with the hardware firmware using G-code style serial commands (e.g., `G0`, `G28`, `M50`, `M51`, `M57`).
* Exported STL files for 3D printing should be saved to the `exports/` directory.
* The hardware architecture utilizes a distributed CAN-FD setup where remote actuators are 'Smart Motors' running CLN-ClosedLoopNemaDriver hardware and CLN-SimpleFOC firmware. The main controller's firmare primarily handles kinematics, G-code parsing, and CAN message dispatching.
* The Open Micro-Manipulator project consists of a Python API (`software/PythonAPI`) and C++ firmware (`firmware`).
* To set up the Python API development environment, run `pip install -r software/PythonAPI/requirements.txt` and `pip install -e software/PythonAPI`.
* For CAN-FD distributed control, the main controller sends standard CAN-FD frames targeting specific remote node IDs (e.g., 0x101, 0x102) with a payload of a 4-byte standard float representing the target linear position.
* The user prefers TODO lists to be created as Markdown files.
* The MicroManipulatorStepper project utilizes a 'magnetic gearing' mechanism via a multi-pole magnetic ring acting as a high-resolution rotary encoder, paired with an LIS3MDL magnetic sensor.
* To run Python API tests without installing the package in editable mode, use `PYTHONPATH=software/PythonAPI/src python3 -m unittest software/PythonAPI/tests/test_interface.py`.
* When starting a new task, always enter a deep planning mode first: ask clarifying questions using the `request_user_input` and `message_user` tools to achieve absolute certainty of requirements. Do not propose a plan until all questions are answered. Once the plan is created with `set_plan` and approved, execute autonomously without asking further questions.
* To run FreeCAD tests and simulations, navigate to the project root and use the command `freecadcmd tests/freecad_scripts/run_all_tests.py`.
* When executing Python scripts via `freecadcmd`, the `__name__` variable evaluates to `__builtin__` or `FreeCAD` instead of `__main__`. Scripts running tests must use `if __name__ in ('__main__', '__builtin__', 'FreeCAD'):` to execute successfully.
