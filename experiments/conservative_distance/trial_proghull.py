"""
H3: Progressive-Hull poly reduction (conservative).
A carved model is a convex polytope = intersection of half-spaces. REMOVING a
half-space only ENLARGES it -> stays a conservative superset. Greedily drop the
half-space whose removal adds least volume; trace (facets vs volume) to see how
many faces we can shed for a small, bounded over-expansion.
"""
import sys, os
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
sys.path.insert(0, os.path.dirname(__file__))
import poc
me = poc.me


def uv_sphere(R=10.0, nlat=20, nlon=30):
    V = [[0, 0, R], [0, 0, -R]]
    for i in range(1, nlat):
        th = np.pi*i/nlat
        for j in range(nlon):
            ph = 2*np.pi*j/nlon
            V.append([R*np.sin(th)*np.cos(ph), R*np.sin(th)*np.sin(ph), R*np.cos(th)])
    V = np.array(V, float)
    F = []
    def idx(i, j): return 2 + (i-1)*nlon + (j % nlon)
    for j in range(nlon):
        F.append([0, idx(1, j), idx(1, j+1)]); F.append([1, idx(nlat-1, j+1), idx(nlat-1, j)])
    for i in range(1, nlat-1):
        for j in range(nlon):
            a, b, c, d = idx(i, j), idx(i, j+1), idx(i+1, j+1), idx(i+1, j)
            F += [[a, b, c], [a, c, d]]
    return V, np.array(F, np.int32)


def carved_halfspaces(v, f, d, merge_deg=20):
    """Carve (expand) then return unique outward half-spaces [n|c] (n.x <= c)."""
    out = me.BoxExpander(merge_deg).expand(me.Mesh.from_arrays(v.astype(float), f.astype(np.int32)), d)
    hull = ConvexHull(out.vertices)
    # scipy eq: [a,b,c,d] with a.x + d <= 0  ->  n=[a,b,c], offset c = -d
    H = {}
    for eq in hull.equations:
        n = eq[:3]; c = -eq[3]
        key = tuple(np.round(n / (np.linalg.norm(n) + 1e-15), 4))
        if key not in H:
            H[key] = (n / (np.linalg.norm(n) + 1e-15), c / (np.linalg.norm(n) + 1e-15))
    return list(H.values()), out.vertices


def poly_from_halfspaces(H, interior):
    A = np.array([h[0] for h in H]); b = np.array([h[1] for h in H])
    hs = np.hstack([A, -b[:, None]])           # a.x - c <= 0
    hi = HalfspaceIntersection(hs, interior)
    pts = hi.intersections
    ch = ConvexHull(pts)
    return ch.volume, len(ch.vertices), ch.simplices.shape[0]


def main():
    print("="*70, flush=True)
    print("H3 — progressive-hull conservative poly reduction (sphere, d=1)", flush=True)
    print("="*70, flush=True)
    v, f = uv_sphere()
    H, verts = carved_halfspaces(v, f, 1.0, merge_deg=15)
    interior = verts.mean(0)
    vol0, nv0, nf0 = poly_from_halfspaces(H, interior)
    print(f"  start: {len(H)} half-spaces, {nf0} faces, vol={vol0:.1f}", flush=True)
    print(f"  {'facets':>7s} {'faces':>6s} {'vol':>10s} {'vol%':>7s}", flush=True)

    cur = list(H)
    targets = {len(cur), 64, 48, 32, 24, 16, 12, 10, 8}
    while len(cur) > 6:
        # find the half-space whose removal adds least volume
        best_i, best_vol = None, np.inf
        for i in range(len(cur)):
            trial = cur[:i] + cur[i+1:]
            try:
                vol, _, _ = poly_from_halfspaces(trial, interior)
            except Exception:
                continue
            if vol < best_vol:
                best_vol, best_i = vol, i
        if best_i is None:
            break
        cur = cur[:best_i] + cur[best_i+1:]
        if len(cur) in targets:
            vol, nv, nf = poly_from_halfspaces(cur, interior)
            print(f"  {len(cur):7d} {nf:6d} {vol:10.1f} {100*(vol/vol0-1):+6.1f}%", flush=True)
    print("\nDONE  (volume only grows => always conservative superset)", flush=True)


if __name__ == "__main__":
    main()
