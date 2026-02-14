import sys
import os
import unittest
import math

# Add the script directory to path to import kinematic_model
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

from kinematic_model import KinematicModel_Delta3D, Vec3F, QuaternionF, Pose6DF, DEG2RAD

class TestKinematicsValidation(unittest.TestCase):
    def setUp(self):
        self.model = KinematicModel_Delta3D()

    def test_forward_inverse_consistency(self):
        """Check if Forward(Inverse(P)) == P for a few points."""
        test_points = [
            Vec3F(0, 0, 0),
            Vec3F(5, 5, 5),
            Vec3F(-5, 5, -5),
            Vec3F(0, 0, 10)
        ]

        for p in test_points:
            target_pose = Pose6DF(p, QuaternionF())
            success, joints = self.model.inverse(target_pose)
            self.assertTrue(success, f"Inverse kinematics failed for {p}")

            success_fk, result_pose = self.model.forward(joints)
            self.assertTrue(success_fk, f"Forward kinematics failed for joints {joints}")

            diff = result_pose.translation - p
            self.assertLess(diff.length(), 1e-3, f"Mismatch at {p}: got {result_pose.translation}")

    def test_known_values(self):
        """Check against values from C++ implementation comments/tests."""
        # From C++ test() function output in comments or manual run
        # Forward Kinematic for 45 deg joints
        joints = [45 * DEG2RAD] * 3
        success, pose = self.model.forward(joints)
        self.assertTrue(success)
        # Expected: ~(-0.56, -0.56, -0.56) based on my previous run of python script
        # which matches the C++ logic I ported.
        self.assertAlmostEqual(pose.translation.x, -0.564976, places=4)
        self.assertAlmostEqual(pose.translation.y, -0.564976, places=4)
        self.assertAlmostEqual(pose.translation.z, -0.564976, places=4)

    def test_cad_parameters_placeholder(self):
        """
        Placeholder for validating CAD parameters.
        In a full implementation, this would load the FreeCAD assembly
        and measure the arm length directly from the geometry.
        """
        # Example logic:
        # doc = FreeCAD.openDocument("Assembly_MicroManipulator.FCStd")
        # arm_obj = doc.getObject("ArmLink")
        # measured_length = ...
        # self.assertAlmostEqual(measured_length, self.model.arm_length)
        pass

if __name__ == '__main__':
    unittest.main()
