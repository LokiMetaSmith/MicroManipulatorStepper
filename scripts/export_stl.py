import sys
import os
import argparse

# Check if running within FreeCAD environment
try:
    import FreeCAD
    import Mesh
    import MeshPart
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False

def get_whitelisted_files():
    """Returns a list of FreeCAD files to export."""
    # List of files to export.
    # We use paths relative to the project root.
    return [
        "construction/micro_manipulator/CrimpBead.FCStd",
        "construction/micro_manipulator/MotorHorn.FCStd",
        "construction/micro_manipulator/MT6835.FCStd",
        "construction/micro_manipulator/MotorMount.FCStd",
        "construction/micro_manipulator/JointBall.FCStd",
        "construction/micro_manipulator/BallJointPlate.FCStd",
        "construction/micro_manipulator/BaseBlock.FCStd",
        "construction/micro_manipulator/StepperMotorNema17.FCStd",
        "construction/micro_manipulator/EncoderMagnetArray.FCStd",
        "construction/micro_manipulator/EncoderMagnet.FCStd",
        "construction/micro_manipulator/RubberBand.FCStd",
        "construction/micro_manipulator/RubberBandCollet.FCStd",
        "construction/micro_manipulator/EndEffector.FCStd",
        "construction/micro_manipulator/LinkageRod.FCStd",
        # Tooling
        "construction/tools/rod_embossing_tools/rod_sharper/BladeCarrier.FCStd",
        "construction/tools/rod_embossing_tools/rod_sharper/RodGuide.FCStd",
        "construction/tools/rod_embossing_tools/rod_lenth_setting_press/RodGrindingJig_A.FCStd",
        "construction/tools/rod_embossing_tools/rod_lenth_setting_press/rod_collet.FCStd",
        "construction/tools/rod_embossing_tools/rod_lenth_setting_press/caliper_clamp.FCStd",
        "construction/tools/rod_embossing_tools/rod_lenth_setting_press/RodGrindingJig_B.FCStd",
        "construction/tools/rod_embossing_tools/rod_lenth_setting_press/rod_guide.FCStd"
    ]

def export_file_to_stls(filepath, output_dir, linear_deflection=None, angular_deflection=None):
    """
    Exports the individual parts of a FreeCAD file to the output directory.
    """
    if not os.path.exists(filepath):
        print(f"Error: File not found: {filepath}")
        return

    print(f"Processing: {filepath}")

    try:
        doc = FreeCAD.openDocument(filepath)
    except Exception as e:
        print(f"Error opening {filepath}: {e}")
        return

    filename_base = os.path.splitext(os.path.basename(filepath))[0]

    export_count = 0

    # Process all objects in the document
    for obj in doc.Objects:
        # Check if the object has a Shape that is valid
        if hasattr(obj, "Shape") and obj.Shape.isValid():
            # Check if it's a type we want to export (Body or Feature)
            if obj.isDerivedFrom("PartDesign::Body") or obj.isDerivedFrom("Part::Feature"):

                # Check if this object is inside a PartDesign::Body (we only want the top-level Body)
                is_internal_feature = False
                if hasattr(obj, "InList") and obj.InList:
                    for parent in obj.InList:
                        if parent.isDerivedFrom("PartDesign::Body"):
                            is_internal_feature = True
                            break

                # If it's an internal feature of a body, skip it
                if is_internal_feature and not obj.isDerivedFrom("PartDesign::Body"):
                    continue

                # Setup output file path
                safe_obj_name = obj.Name.replace("/", "_").replace("\\", "_")
                stl_filename = f"{filename_base}_{safe_obj_name}.stl"
                stl_filepath = os.path.join(output_dir, stl_filename)

                print(f"  -> Exporting object '{obj.Name}' to '{stl_filepath}'")

                try:
                    # Apply custom mesh parameters if provided
                    if linear_deflection is not None and angular_deflection is not None:
                        mesh = MeshPart.meshFromShape(
                            Shape=obj.Shape,
                            LinearDeflection=linear_deflection,
                            AngularDeflection=angular_deflection,
                            Relative=False
                        )
                        mesh.write(stl_filepath)
                    else:
                        # Default export
                        Mesh.export([obj], stl_filepath)
                    export_count += 1
                except Exception as e:
                    print(f"  -> Warning: Export failed for {obj.Name}: {e}")

    if export_count == 0:
        print(f"  -> No valid parts found to export in {filepath}")

    FreeCAD.closeDocument(doc.Name)

def main():
    parser = argparse.ArgumentParser(description="Export whitelisted FreeCAD files to STL format.")
    parser.add_argument("--outdir", default="exports", help="Output directory for STL files (default: exports)")
    parser.add_argument("--linear-deflection", type=float, default=None, help="Linear deflection for mesh generation (e.g., 0.1)")
    parser.add_argument("--angular-deflection", type=float, default=None, help="Angular deflection for mesh generation (e.g., 0.1)")

    # We parse remaining args as freecadcmd passes some args to the script
    args, unknown = parser.parse_known_args()

    if not HAS_FREECAD:
        print("Error: FreeCAD python modules not found. Please run this script using freecadcmd:")
        print("       freecadcmd scripts/export_stl.py [args]")
        sys.exit(1)

    # Ensure output directory exists
    os.makedirs(args.outdir, exist_ok=True)

    print(f"Output directory: {os.path.abspath(args.outdir)}")
    if args.linear_deflection or args.angular_deflection:
        print(f"Using custom mesh settings: linear={args.linear_deflection}, angular={args.angular_deflection}")
    else:
        print("Using default mesh settings.")

    whitelist = get_whitelisted_files()

    print(f"Found {len(whitelist)} files in the whitelist.")

    for filepath in whitelist:
        export_file_to_stls(
            filepath,
            args.outdir,
            linear_deflection=args.linear_deflection,
            angular_deflection=args.angular_deflection
        )

    print("Export complete.")

if __name__ in ("__main__", "__builtin__", "FreeCAD"):
    main()
