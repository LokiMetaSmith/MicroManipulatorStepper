import sys
import os

try:
    import FreeCAD
    import Part
    import ObjectsFem
    import Fem
except ImportError:
    print("Warning: FreeCAD or FEM module not found. Script cannot run outside FreeCAD environment.")
    sys.exit(0)

def setup_fea():
    # Load Document
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(script_dir))
    file_path = os.path.join(repo_root, "construction", "v3", "MotorMount2_KM.FCStd")

    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        return

    doc = FreeCAD.openDocument(file_path)
    print(f"Loaded document: {doc.Name}")

    # Create Analysis
    analysis = ObjectsFem.makeAnalysis(doc, "MechanicalAnalysis")

    # Create Solver (CalculiX)
    solver = ObjectsFem.makeSolverCalculixCcxTools(doc, "CalculiX")
    analysis.Member += [solver]

    # Create Material (PLA)
    material = ObjectsFem.makeMaterialSolid(doc, "SolidMaterial")
    mat = material.Material
    mat["Name"] = "PLA"
    mat["YoungsModulus"] = "3500 MPa"
    mat["PoissonRatio"] = "0.36"
    mat["Density"] = "1250 kg/m^3"
    material.Material = mat
    analysis.Member += [material]

    # Find the main body
    # Assuming the visible object is the one we want to simulate
    # Based on XML, Fillet013 was visible=True, Fillet012 visible=false?
    # Usually the last feature in the tree is the final shape.
    # Let's find the Tip of the Body if possible, or just the object named "Fillet013"
    target_obj = doc.getObject("Fillet013")
    if not target_obj:
        print("Error: Target object 'Fillet013' not found.")
        return

    print(f"Target object for FEA: {target_obj.Label}")

    # Add Fixed Constraint (Base)
    # Strategy: Find face with lowest Z centroid (bottom face)
    faces = target_obj.Shape.Faces
    bottom_face_index = -1
    min_z = float('inf')

    for i, face in enumerate(faces):
        center = face.CenterOfMass
        if center.z < min_z:
            min_z = center.z
            bottom_face_index = i

    if bottom_face_index != -1:
        fixed_constraint = ObjectsFem.makeConstraintFixed(doc, "FixedBase")
        fixed_constraint.References = [(target_obj, f"Face{bottom_face_index+1}")]
        analysis.Member += [fixed_constraint]
        print(f"Added fixed constraint on Face{bottom_face_index+1} (Z={min_z})")

    # Add Force Constraint (Motor Load)
    # Strategy: Find cylindrical faces with radius ~1.6mm (M3 clearance holes)
    mounting_faces = []
    for i, face in enumerate(faces):
        if face.Surface.TypeId == 'Part::GeomCylinder':
            radius = face.Surface.Radius
            if 1.5 < radius < 1.7: # M3 clearance ~1.6mm
                mounting_faces.append(f"Face{i+1}")

    if mounting_faces:
        force_constraint = ObjectsFem.makeConstraintForce(doc, "MotorLoad")
        force_constraint.References = [(target_obj, face_name) for face_name in mounting_faces]
        force_constraint.Force = 10.0 # 10 Newtons
        force_constraint.Direction = (0, 0, -1) # Downwards
        analysis.Member += [force_constraint]
        print(f"Added force constraint on {len(mounting_faces)} faces: {mounting_faces}")
    else:
        print("Warning: No mounting holes found for force application.")

    # Save
    output_path = os.path.join(script_dir, "MotorMount2_KM_FEA.FCStd")
    doc.saveAs(output_path)
    print(f"Saved FEA setup to {output_path}")

if __name__ == "__main__":
    setup_fea()
