import sys
import os
import unittest

# Try to import FreeCAD, or mock it if running in an environment without it (for development verification)
try:
    import FreeCAD
except ImportError:
    print("Warning: FreeCAD module not found. Running in mock mode or failing if strict.")
    FreeCAD = None

class TestMotorMount(unittest.TestCase):
    def setUp(self):
        if FreeCAD is None:
            self.skipTest("FreeCAD module not available")

        # Locate the FCStd file
        # Assuming script is in tests/freecad_scripts/
        # and file is in construction/v3/
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.repo_root = os.path.dirname(os.path.dirname(self.script_dir))
        self.file_path = os.path.join(self.repo_root, "construction", "v3", "MotorMount2_KM.FCStd")

        if not os.path.exists(self.file_path):
            self.fail(f"File not found: {self.file_path}")

        # Open the document
        self.doc_name = "MotorMount2_KM"
        self.doc = FreeCAD.openDocument(self.file_path)

    def tearDown(self):
        if FreeCAD and self.doc:
            FreeCAD.closeDocument(self.doc.Name)

    def test_fillet_radius(self):
        """Verify that Fillet013 has a radius of 1.0 mm."""
        obj = self.doc.getObject("Fillet013")
        self.assertIsNotNone(obj, "Fillet013 object not found in document")

        # Check Radius
        # PropertyQuantityConstraint usually returns a Quantity, which has .Value
        radius = obj.Radius
        if hasattr(radius, "Value"):
            val = radius.Value
        else:
            val = float(radius)

        self.assertAlmostEqual(val, 1.0, places=4, msg="Fillet013 Radius should be 1.0 mm")

    def test_document_properties(self):
        """Verify document metadata."""
        label = self.doc.Label
        # We don't strictly enforce label match if it's empty, but check it exists
        self.assertIsNotNone(label)

if __name__ == '__main__':
    unittest.main()
