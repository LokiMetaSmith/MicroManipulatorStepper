# Open Micro-Manipulator Project TODOs

This file lists potential improvements, features, and bug fixes for the Open Micro-Manipulator project.

## High Priority (Bugs)
- [ ] **Fix `wait_for_stop` busy loop**: The `wait_for_stop` function in `software/PythonAPI/src/open_micro_manipulator/interface.py` polls `M53` in a tight loop without sleeping, causing high CPU usage and potential serial communication flooding.
- [ ] **Fix `home` function axis index check**: The check `if 0 > axis_idx >= len(axis_chars):` is logically incorrect and always evaluates to False. It should be `if axis_idx < 0 or axis_idx >= len(axis_chars):`.

## Features & Improvements
- [ ] **Implement `read_encoder_angles`**: The function `read_encoder_angles` in the Python API is currently a stub. It should parse the output of the `M51` command.
- [ ] **Implement `read_device_state_info` parsing**: The function `read_device_state_info` sends `M57` but does not parse or return the device state information.
- [ ] **Add Rotation Support to `set_pose`**: The `set_pose` function only supports translation (X, Y, Z). It should be extended to support rotation (A, B, C) as per the firmware capabilities.
- [ ] **Add Context Manager Support**: Implement `__enter__` and `__exit__` methods in `OpenMicroStageInterface` to allow usage with the `with` statement (automatically closing the connection).
- [ ] **Add Type Hinting and Docstrings**: Improve type hinting and docstrings throughout the Python API for better developer experience.
- [ ] **Add Unit Tests**: Expand the test suite to cover more API functions and edge cases, potentially using `pytest`.
- [ ] **Add CLI Tool**: Create a simple command-line interface (CLI) to control the manipulator without writing Python scripts.

## Firmware
- [ ] **Verify `M51` output format**: Ensure the firmware output for `M51` is consistent and easy to parse.
- [ ] **Add detailed error codes**: Improve error reporting in G-Code responses.

## Documentation
- [ ] **Update API Documentation**: Ensure the README or separate API docs reflect the latest changes and features.
