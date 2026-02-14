# FreeCAD Testing & Simulation

This directory contains Python scripts for testing, verifying, and simulating the FreeCAD models of the Micro-Manipulator project.

## How to Run

These scripts must be run inside the FreeCAD environment, typically via the `freecadcmd` command-line tool.

1.  **Install FreeCAD**: Make sure FreeCAD is installed and in your system path.
2.  **Navigate to Project Root**: `cd MicroManipulatorStepper`
3.  **Run Tests**:
    ```bash
    freecadcmd tests/freecad_scripts/run_all_tests.py
    ```
    This will execute all test cases and report success/failure.

## Scripts

*   `kinematic_model.py`: A Python port of the C++ kinematic model. Used for verifying kinematics against the CAD model.
*   `test_motor_mount.py`: Unit test for the Motor Mount part. Verifies dimensions (Radius).
*   `test_kinematics_validation.py`: Checks if the firmware's kinematic parameters match the CAD dimensions.
*   `setup_fea.py`: Sets up a Finite Element Analysis (FEA) on the motor mount part, applying materials, constraints, and loads.

## Troubleshooting

If you encounter `ModuleNotFoundError: No module named 'FreeCAD'`, ensure you are running the script with `freecadcmd` or `freecad`, NOT standard `python`.
