"""List the shape-carrying objects of a FreeCAD .FCStd file.

Prints every object that stores a Shape (file name inside the archive), its type,
label, whether it is visible in the GUI document and, for PartDesign bodies, the Tip.
Used to pick the final solid of each upkie_parts file before converting it to STEP.

    python fcstd_inspect.py path/to/part.FCStd
"""
import sys
import zipfile
import xml.etree.ElementTree as ET


def inspect(path):
    z = zipfile.ZipFile(path)
    doc = ET.fromstring(z.read("Document.xml"))
    gui = {}
    if "GuiDocument.xml" in z.namelist():
        groot = ET.fromstring(z.read("GuiDocument.xml"))
        for vp in groot.iter("ViewProvider"):
            name = vp.get("name")
            for prop in vp.iter("Property"):
                if prop.get("name") == "Visibility":
                    b = prop.find("Bool")
                    if b is not None:
                        gui[name] = b.get("value")
    types = {}
    objs = doc.find("Objects")
    for o in objs.iter("Object"):
        types[o.get("name")] = o.get("type")
    sizes = {i.filename: i.file_size for i in z.infolist()}
    for od in doc.find("ObjectData").iter("Object"):
        name = od.get("name")
        label, shape, tip, group = None, None, None, None
        for prop in od.iter("Property"):
            pn = prop.get("name")
            if pn == "Label":
                s = prop.find("String")
                label = s.get("value") if s is not None else None
            elif pn == "Shape":
                p = prop.find("Part")
                if p is not None:
                    shape = p.get("file")
            elif pn == "Tip":
                l = prop.find("Link")
                tip = l.get("value") if l is not None else None
        if shape or types.get(name, "").startswith("PartDesign::Body"):
            print(f"{name:24s} {types.get(name, ''):28s} label={label!s:28s} vis={gui.get(name)} "
                  f"shape={shape} ({sizes.get(shape, 0)} B) tip={tip}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print("==", p)
        inspect(p)
