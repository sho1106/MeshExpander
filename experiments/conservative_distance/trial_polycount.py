"""
H3' (cheap): the face-normal MERGE ANGLE is the built-in poly-count knob.
On a dense curved shape, sweep merge angle and measure:
  - output face count (lower = fewer polys)
  - output volume (proxy for over-expansion / looseness; larger = looser)
  - conservativeness (all original vertices remain inside the expanded hull)
This quantifies the poly-count vs accuracy tradeoff that already exists.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import poc
me = poc.me


def uv_sphere(R=10.0, nlat=20, nlon=30):
    V = [[0, 0, R], [0, 0, -R]]
    for i in range(1, nlat):
        th = np.pi * i / nlat
        for j in range(nlon):
            ph = 2*np.pi*j/nlon
            V.append([R*np.sin(th)*np.cos(ph), R*np.sin(th)*np.sin(ph), R*np.cos(th)])
    V = np.array(V, float)
    F = []
    def idx(i, j): return 2 + (i-1)*nlon + (j % nlon)
    for j in range(nlon):
        F.append([0, idx(1, j), idx(1, j+1)])
        F.append([1, idx(nlat-1, j+1), idx(nlat-1, j)])
    for i in range(1, nlat-1):
        for j in range(nlon):
            a, b, c, d = idx(i, j), idx(i, j+1), idx(i+1, j+1), idx(i+1, j)
            F += [[a, b, c], [a, c, d]]
    return V, np.array(F, np.int32)


def volume(v, f):
    s = 0.0
    for t in f:
        a, b, c = v[t[0]], v[t[1]], v[t[2]]
        s += np.dot(a, np.cross(b, c))
    return abs(s) / 6.0


def all_inside(orig_v, hull_v):
    return max(poc.gjk_distance(p[None, :], hull_v) for p in orig_v) <= 1e-6


def main():
    print("="*70, flush=True)
    print("H3' poly-count vs accuracy: merge-angle sweep on a UV-sphere (d=1)", flush=True)
    print("="*70, flush=True)
    v, f = uv_sphere(R=10.0, nlat=20, nlon=30)
    print(f"  input: {len(v)} verts, {len(f)} faces; d=1.0", flush=True)
    print(f"  {'mergeDeg':>8s} {'outFaces':>8s} {'outVol':>12s} {'conservative':>12s}", flush=True)
    for deg in [5, 10, 20, 40, 60, 80]:
        exp = me.BoxExpander(deg)
        try:
            out = exp.expand(me.Mesh.from_arrays(v, f), 1.0)
        except RuntimeError as e:
            print(f"  {deg:8d} {'BLOCKED':>8s}  (too many directions: {str(e)[:40]}...)", flush=True)
            continue
        ov, of = out.vertices, out.faces
        cons = all_inside(v, ov)
        print(f"  {deg:8d} {len(of):8d} {volume(ov, of):12.1f} {str(cons):>12s}", flush=True)
    print("\nDONE", flush=True)


if __name__ == "__main__":
    main()
