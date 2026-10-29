"""OpenCascade helpers for the quote parts (python -m pip install cadquery-ocp).

Fusion is not needed: the parts are built from the exported STEP (`../export/parts_step`) and from
`../fusion/params.py`.  Lengths in mm.
"""
import contextlib
import math
import os
import sys

from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder
from OCP.Bnd import Bnd_Box
from OCP.GCPnts import GCPnts_TangentialDeflection
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Line, GeomAbs_Plane
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf


# ----------------------------------------------------------------------------- read / write
def read_step(path):
    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise IOError(path)
    r.TransferRoots()
    return r.OneShape()


def solids(shape):
    out, ex = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        out.append(TopoDS.Solid(ex.Current()))
        ex.Next()
    return out


def write_step(shape, path, name):
    """STEP AP214 in mm with the product name set (ASCII)."""
    Interface_Static.SetCVal_s('write.step.unit', 'MM')
    doc = TDocStd_Document(TCollection_ExtendedString('XmlOcaf'))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    lab = tool.AddShape(shape, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    w = STEPCAFControl_Writer()
    w.SetNameMode(True)
    with _quiet():
        w.Transfer(doc, STEPControl_AsIs)
        ok = w.Write(str(path)) == IFSelect_RetDone
    if not ok:
        raise IOError(path)


@contextlib.contextmanager
def _quiet():
    """Silence the STEP writer's statistics (printed by OCCT straight to the C stdout)."""
    sys.stdout.flush()
    keep = os.dup(1)
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    try:
        yield
    finally:
        os.dup2(keep, 1)
        os.close(null)
        os.close(keep)


# ----------------------------------------------------------------------------- primitives (axis-aligned)
AX = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}


def _pt(axis, a, b, t):
    """Point with coordinate t along `axis` and (a, b) on the other two axes in x, y, z order."""
    i = 'xyz'.index(axis)
    p = [a, b]
    p.insert(i, t)
    return gp_Pnt(*p)


def cyl(axis, a, b, t0, t1, r):
    """Cylinder radius r along `axis` from t0 to t1, centre (a, b) on the other two axes."""
    return BRepPrimAPI_MakeCylinder(gp_Ax2(_pt(axis, a, b, t0), gp_Dir(*AX[axis])), r, t1 - t0).Shape()


def cone(axis, a, b, t0, t1, r0, r1):
    return BRepPrimAPI_MakeCone(gp_Ax2(_pt(axis, a, b, t0), gp_Dir(*AX[axis])), r0, r1, t1 - t0).Shape()


def box(x0, x1, y0, y1, z0, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def fuse(a, b):
    return BRepAlgoAPI_Fuse(a, b).Shape()


def cut(a, b):
    return BRepAlgoAPI_Cut(a, b).Shape()


def common(a, b):
    return BRepAlgoAPI_Common(a, b).Shape()


def clean(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, False)
    u.Build()
    return u.Shape()


def chamfer_edges(shape, dist, pred):
    """Equal-distance chamfer on the straight edges where pred(p0, p1) is true."""
    mk = BRepFilletAPI_MakeChamfer(shape)
    ex, n = TopExp_Explorer(shape, TopAbs_EDGE), 0
    while ex.More():
        e = TopoDS.Edge(ex.Current())
        c = BRepAdaptor_Curve(e)
        if c.GetType() == GeomAbs_Line:
            p0, p1 = c.Value(c.FirstParameter()), c.Value(c.LastParameter())
            if pred((p0.X(), p0.Y(), p0.Z()), (p1.X(), p1.Y(), p1.Z())):
                mk.Add(dist, e)
                n += 1
        ex.Next()
    assert n, 'no edge matched'
    return mk.Shape()


def transform(shape, rows):
    """rows = 3 x 4 matrix [R | t] (proper rotation)."""
    t = gp_Trsf()
    t.SetValues(*rows[0], *rows[1], *rows[2])
    return BRepBuilderAPI_Transform(shape, t, True).Shape()


# ----------------------------------------------------------------------------- measuring
def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)
    a, c = b.CornerMin(), b.CornerMax()
    return (a.X(), c.X(), a.Y(), c.Y(), a.Z(), c.Z())


def volume(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    return g.Mass()


# ----------------------------------------------------------------------------- 2D projection
def _polyline(edge, defl=0.02):
    c = BRepAdaptor_Curve(edge)
    if c.GetType() == GeomAbs_Line:
        ps = [c.Value(c.FirstParameter()), c.Value(c.LastParameter())]
    else:
        d = GCPnts_TangentialDeflection(c, math.radians(4), defl)
        ps = [d.Value(i) for i in range(1, d.NbPoints() + 1)]
    return [(p.X(), p.Y()) for p in ps]


def _edges(compound):
    out = []
    if compound is None or compound.IsNull():
        return out
    ex = TopExp_Explorer(compound, TopAbs_EDGE)
    while ex.More():
        out.append(_polyline(TopoDS.Edge(ex.Current())))
        ex.Next()
    return out


def project(shape, toward_viewer, xdir):
    """Hidden-line projection. toward_viewer: unit vector from the part to the viewer; xdir: screen right.
    Returns (visible, hidden) lists of 2D polylines in screen coordinates (screen up = toward_viewer x xdir)."""
    algo = HLRBRep_Algo()
    algo.Add(shape)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(*toward_viewer), gp_Dir(*xdir))))
    algo.Update()
    algo.Hide()
    h = HLRBRep_HLRToShape(algo)
    vis = _edges(h.VCompound()) + _edges(h.OutLineVCompound())
    hid = _edges(h.HCompound()) + _edges(h.OutLineHCompound())
    return vis, hid


def section_faces(shape, axis, value):
    """Planar faces of `shape` lying on the plane axis = value, as lists of 3D point loops (outer + holes)."""
    i = 'xyz'.index(axis)
    out = []
    fe = TopExp_Explorer(shape, TopAbs_FACE)
    while fe.More():
        f = TopoDS.Face(fe.Current())
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Plane:
            pl = a.Plane()
            d, o = pl.Axis().Direction(), pl.Location()
            if abs(abs((d.X(), d.Y(), d.Z())[i]) - 1) < 1e-6 and abs((o.X(), o.Y(), o.Z())[i] - value) < 1e-6:
                loops = []
                we = TopExp_Explorer(f, TopAbs_WIRE)
                while we.More():
                    loop, wx = [], BRepTools_WireExplorer(TopoDS.Wire(we.Current()), f)
                    while wx.More():
                        e = wx.Current()
                        c = BRepAdaptor_Curve(e)
                        if c.GetType() == GeomAbs_Line:
                            us = [c.FirstParameter(), c.LastParameter()]
                        else:
                            us = [c.FirstParameter() + (c.LastParameter() - c.FirstParameter()) * k / 48 for k in range(49)]
                        pts = [(p.X(), p.Y(), p.Z()) for p in (c.Value(u) for u in us)]
                        if wx.Orientation() == TopAbs_REVERSED:
                            pts.reverse()
                        loop.extend(pts)
                        wx.Next()
                    loops.append(loop)
                    we.Next()
                out.append(loops)
        fe.Next()
    return out
