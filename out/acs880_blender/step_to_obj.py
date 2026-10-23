import sys, json
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SHELL, TopAbs_FACE
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
import numpy as np
src, out = sys.argv[1], sys.argv[2]
r = STEPControl_Reader(); assert r.ReadFile(src) == 1
r.TransferRoots(); shape = r.OneShape()
BRepMesh_IncrementalMesh(shape, 0.5, False, 0.3, True)
shells = []
ex = TopExp_Explorer(shape, TopAbs_SHELL)
while ex.More():
    sh = TopoDS.Shell(ex.Current()); V=[]; F=[]
    fe = TopExp_Explorer(sh, TopAbs_FACE)
    while fe.More():
        f = TopoDS.Face(fe.Current()); loc = TopLoc_Location()
        t = BRep_Tool.Triangulation_s(f, loc)
        if t is not None:
            base = len(V); tr = loc.Transformation()
            for i in range(1, t.NbNodes()+1):
                p = t.Node(i).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
            rev = f.Orientation().value == 1
            for i in range(1, t.NbTriangles()+1):
                a,b,c = t.Triangle(i).Get()
                a,b,c = base+a-1, base+b-1, base+c-1
                F.append((a,c,b) if rev else (a,b,c))
        fe.Next()
    if F: shells.append((np.array(V), np.array(F)))
    ex.Next()
print(len(shells), "shells")
with open(out, "w") as o:
    vo = 1
    for i,(V,F) in enumerate(shells):
        o.write(f"o shell_{i:03d}\n")
        for v in V: o.write("v %.4f %.4f %.4f\n" % tuple(v/1000.0))  # mm -> m
        for f in F+vo: o.write("f %d %d %d\n" % tuple(f))
        vo += len(V)
meta=[]
for i,(V,F) in enumerate(shells):
    lo,hi=V.min(0)/1000,V.max(0)/1000
    meta.append(dict(i=i,tris=len(F),lo=lo.round(3).tolist(),hi=hi.round(3).tolist()))
pass
for m in sorted(meta,key=lambda m:-m['tris'])[:25]: print(m)
allv=np.vstack([v for v,_ in shells])/1000; print("bbox",allv.min(0).round(3),allv.max(0).round(3), "tris", sum(len(f) for _,f in shells))
