"""Convert the final solid stored in a FreeCAD .FCStd file to STEP (no FreeCAD needed).

FreeCAD keeps every feature's shape as an OpenCASCADE BRep inside the zip archive;
the PartDesign body's own Shape is the final part. gmsh (pip install gmsh) embeds
OpenCASCADE, so it can read that BRep and write a STEP file in millimetres.

    python fcstd_to_step.py part.FCStd SHAPE_FILE.brp out.step
"""
import os
import sys
import tempfile
import zipfile

import gmsh


def convert(fcstd, member, out_step):
    with zipfile.ZipFile(fcstd) as z, tempfile.TemporaryDirectory() as tmp:
        brp = os.path.join(tmp, "shape.brep")
        with open(brp, "wb") as f:
            f.write(z.read(member))
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Geometry.OCCScaling", 1.0)
        gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
        gmsh.model.add("part")
        ents = gmsh.model.occ.importShapes(brp, highestDimOnly=True)
        gmsh.model.occ.synchronize()
        box = gmsh.model.getBoundingBox(-1, -1)
        vols = gmsh.model.getEntities(3)
        vol = sum(gmsh.model.occ.getMass(3, t) for _, t in vols)
        gmsh.write(out_step)
        gmsh.finalize()
    print(f"{os.path.basename(out_step)}: {len(ents)} entities, {len(vols)} solids, "
          f"volume {vol / 1e3:.1f} cm3, bbox min ({box[0]:.1f}, {box[1]:.1f}, {box[2]:.1f}) "
          f"max ({box[3]:.1f}, {box[4]:.1f}, {box[5]:.1f}) mm")


if __name__ == "__main__":
    convert(*sys.argv[1:4])
