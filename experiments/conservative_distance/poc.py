"""
Proof-of-concept: certified conservative distance (lower bound) with early-out,
built on MeshExpander's outer-hull guarantee.

Thesis to falsify:
  (1) VALIDITY: distance(outer_hull_A, outer_hull_B) <= true_distance(A,B)  ALWAYS.
  (2) MONOTONE: d_L0 <= d_L1 <= true   (coarse hull -> finer pieces tightens the lower bound).
  (3) EARLY-OUT: for a safety margin m, many "is clearance >= m?" queries resolve
      at the cheap L0 level (1 GJK) instead of L1 (pieces x pieces GJK).

Leaf distance = self-contained GJK (validated against analytic box cases).
Outer hulls   = MeshExpander carving with d~0 (conservative convex hull / pieces).

Run:  PYTHONPATH=python python experiments/conservative_distance/poc.py
"""
import sys, os, time, itertools
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
import meshexpander as me

rng = np.random.default_rng(0)

# ---------------------------------------------------------------------------
# GJK distance between two convex polytopes (vertex sets). Self-contained.
# ---------------------------------------------------------------------------
def _closest_to_origin(pts):
    """Closest point of the convex hull of <=4 points to the origin.
    Returns (closest_point, kept_indices). Brute force over sub-simplices."""
    n = len(pts)
    P = [np.asarray(p, float) for p in pts]

    # tetra: origin inside?
    if n == 4:
        a, b, c, d = P
        M = np.array([b - a, c - a, d - a]).T
        try:
            uvw = np.linalg.solve(M, -a)
            u, v, w = uvw
            t = 1 - u - v - w
            if t >= -1e-12 and u >= -1e-12 and v >= -1e-12 and w >= -1e-12:
                return np.zeros(3), [0, 1, 2, 3]
        except np.linalg.LinAlgError:
            pass

    best_cp, best_idx, best_d2 = None, None, np.inf
    idxs = range(n)
    # size-1
    for i in idxs:
        cp = P[i]; d2 = cp @ cp
        if d2 < best_d2: best_cp, best_idx, best_d2 = cp, [i], d2
    # size-2
    for i, j in itertools.combinations(idxs, 2):
        a, b = P[i], P[j]; ab = b - a; L2 = ab @ ab
        if L2 < 1e-18: continue
        t = (-a @ ab) / L2
        if -1e-12 <= t <= 1 + 1e-12:
            cp = a + np.clip(t, 0, 1) * ab; d2 = cp @ cp
            if d2 < best_d2: best_cp, best_idx, best_d2 = cp, [i, j], d2
    # size-3
    for i, j, k in itertools.combinations(idxs, 3):
        a, b, c = P[i], P[j], P[k]; e1 = b - a; e2 = c - a; p = -a
        A = np.array([[e1 @ e1, e1 @ e2], [e1 @ e2, e2 @ e2]])
        try:
            uv = np.linalg.solve(A, np.array([e1 @ p, e2 @ p]))
        except np.linalg.LinAlgError:
            continue
        u, v = uv; t = 1 - u - v
        if u >= -1e-12 and v >= -1e-12 and t >= -1e-12:
            cp = a + u * e1 + v * e2; d2 = cp @ cp
            if d2 < best_d2: best_cp, best_idx, best_d2 = cp, [i, j, k], d2
    return best_cp, best_idx


def gjk_distance(A, B, max_iter=64, tol=1e-10):
    """Distance between convex hulls of vertex sets A (n,3), B (m,3). 0 if overlap."""
    A = np.asarray(A, float); B = np.asarray(B, float)

    def support(d):
        return A[np.argmax(A @ d)] - B[np.argmax(B @ (-d))]

    simplex = [support(np.array([1.0, 0.0, 0.0]))]
    v = simplex[0]
    for _ in range(max_iter):
        d = -v
        if v @ v < tol * tol:
            return 0.0                      # origin in Minkowski diff -> overlap
        w = support(d)
        if (v @ v) - (v @ w) <= tol * (v @ v):
            return float(np.linalg.norm(v))  # converged
        simplex.append(w)
        cp, keep = _closest_to_origin(simplex)
        if cp is None:
            return float(np.linalg.norm(v))
        if len(keep) == 4:
            return 0.0
        simplex = [simplex[i] for i in keep]
        v = cp
    return float(np.linalg.norm(v))


# ---------------------------------------------------------------------------
# Test geometry
# ---------------------------------------------------------------------------
def box(cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = np.array([[cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
                  [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
                  [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
                  [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz]], float)
    f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3]], np.int32)
    return v, f


def lshape(ox=0.0, oy=0.0, s=1.0):
    """L-shaped prism; notch (empty) in the +x,+y quadrant. centered-ish at (ox,oy)."""
    v = np.array([
        [-1, -1, 1], [1, -1, 1], [1, 0, 1], [0, 0, 1], [0, 1, 1], [-1, 1, 1],
        [-1, -1, -1], [1, -1, -1], [1, 0, -1], [0, 0, -1], [0, 1, -1], [-1, 1, -1],
    ], float) * s
    v[:, 0] += ox; v[:, 1] += oy
    f = np.array([[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5],
                  [6, 8, 7], [6, 9, 8], [6, 10, 9], [6, 11, 10],
                  [0, 6, 7], [0, 7, 1], [1, 7, 8], [1, 8, 2], [2, 8, 9], [2, 9, 3],
                  [3, 9, 10], [3, 10, 4], [4, 10, 11], [4, 11, 5], [5, 11, 6], [5, 6, 0]], np.int32)
    return v, f


def surface_samples(v, f, n=4000):
    """Dense area-weighted samples on the triangle mesh surface (for approx true dist)."""
    tris = v[f]
    e1 = tris[:, 1] - tris[:, 0]; e2 = tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    prob = area / area.sum()
    pick = rng.choice(len(f), size=n, p=prob)
    u = rng.random(n); w = rng.random(n)
    over = u + w > 1; u[over] = 1 - u[over]; w[over] = 1 - w[over]
    return tris[pick, 0] + (u[:, None] * e1[pick] + w[:, None] * e2[pick])


def true_distance_approx(vA, fA, vB, fB, n=4000):
    """Approx true min surface distance via dense sampling (>= true; tight when dense)."""
    SA = surface_samples(vA, fA, n); SB = surface_samples(vB, fB, n)
    # min pairwise distance (chunked)
    best = np.inf
    for i in range(0, len(SA), 256):
        chunk = SA[i:i + 256]
        d = np.linalg.norm(chunk[:, None, :] - SB[None, :, :], axis=2)
        best = min(best, d.min())
    return float(best)


# ---------------------------------------------------------------------------
# Conservative outer hulls via MeshExpander (d ~ 0)
# ---------------------------------------------------------------------------
D0 = 1e-9  # ~0 expansion: carving yields the conservative convex hull

def hull_L0(v, f):
    ov, of = me.expand_np(v.astype(np.float64), f.astype(np.int32), d=D0)
    return ov  # vertex set of the single conservative convex hull

def hull_L1_pieces(v, f, boxes):
    """Carve each covering box -> a conservative convex piece. boxes: list of (min,max)."""
    mesh = me.Mesh.from_arrays(v.astype(np.float64), f.astype(np.int32))
    exp = me.BoxExpander()
    pieces = []
    for bmin, bmax in boxes:
        b = me.AlignedBox3d(np.array(bmin, float), np.array(bmax, float))
        r = exp.expand_box(b, mesh, D0)
        if not r.empty():
            pieces.append(r.vertices)
    return pieces


def lshape_cover(ox, oy, s=1.0):
    """2-box cover of the L (bottom strip + left strip), in world coords."""
    z = (-1.05 * s, 1.05 * s)
    bottom = ([ox - 1.05*s, oy - 1.05*s, z[0]], [ox + 1.05*s, oy + 0.02*s, z[1]])
    left   = ([ox - 1.05*s, oy - 1.05*s, z[0]], [ox + 0.02*s, oy + 1.05*s, z[1]])
    return [bottom, left]


# ---------------------------------------------------------------------------
# Conservative-distance query with early-out
# ---------------------------------------------------------------------------
def query(A_L0, B_L0, A_L1, B_L1, margin):
    t0 = time.perf_counter()
    d0 = gjk_distance(A_L0, B_L0)
    if d0 >= margin:
        return dict(decided="SAFE", level="L0", lower=d0, gjk_calls=1,
                    t=time.perf_counter() - t0)
    # refine
    d1 = np.inf; calls = 1
    for pa in A_L1:
        for pb in B_L1:
            d1 = min(d1, gjk_distance(pa, pb)); calls += 1
    decided = "SAFE" if d1 >= margin else "WITHIN_MARGIN"
    return dict(decided=decided, level="L1", lower=max(d0, d1), d0=d0, d1=d1,
                gjk_calls=calls, t=time.perf_counter() - t0)


# ===========================================================================
def main():
    print("="*72)
    print("STEP 1 — validate GJK against analytic box cases")
    print("="*72)
    A, _ = box(0, 0, 0, 1, 1, 1)
    cases = [
        ("gap 1.0 along x", box(2.0, 0, 0, 1, 1, 1), 1.0),
        ("gap 0.5 along x", box(1.5, 0, 0, 1, 1, 1), 0.5),
        ("diagonal corner", box(2.0, 2.0, 0, 1, 1, 1), np.sqrt(1.0**2 + 1.0**2)),
        ("overlapping",     box(0.5, 0, 0, 1, 1, 1), 0.0),
    ]
    ok = True
    for name, (B, _fb), expected in cases:
        d = gjk_distance(A, B)
        good = abs(d - expected) < 1e-6
        ok &= good
        print(f"  {name:18s} gjk={d:.6f} expected={expected:.6f} {'OK' if good else 'FAIL'}")
    if not ok:
        print("  GJK INVALID — aborting."); return
    print("  -> GJK validated.\n")

    print("="*72)
    print("STEP 2 — VALIDITY + MONOTONE on a concave 'notch' scenario")
    print("="*72)
    # A = L-shape (notch empty in +x,+y). B = box sitting OUT IN FRONT of the notch,
    # far from A's actual material but close to A's convex hull (which fills the notch).
    vA, fA = lshape(0, 0, 1.0)
    A0 = hull_L0(vA, fA)
    A1 = hull_L1_pieces(vA, fA, lshape_cover(0, 0, 1.0))
    print(f"  A: L0 hull verts={len(A0)}, L1 pieces={len(A1)}")
    # conservativeness sanity: every original A vertex inside L0 hull?
    # (point inside convex hull of A0 via max-margin check using hull faces is omitted;
    #  MeshExpander already guarantees Cov%=100. We rely on that + GJK exactness.)

    for desc, (bx, by) in [("box near notch (x=2.0,y=2.0)", (2.0, 2.0)),
                           ("box near notch (x=1.6,y=1.6)", (1.6, 1.6))]:
        vB, fB = box(bx, by, 0, 0.6, 0.6, 1.0)
        B0 = hull_L0(vB, fB); B1 = hull_L1_pieces(vB, fB, [
            ([bx-0.32, by-0.32, -0.55], [bx+0.32, by+0.32, 0.55])])
        d0 = gjk_distance(A0, B0)
        d1 = min(gjk_distance(pa, pb) for pa in A1 for pb in B1)
        dt = true_distance_approx(vA, fA, vB, fB)
        v1 = d0 <= dt + 1e-6
        v2 = d1 <= dt + 1e-6
        mono = d0 <= d1 + 1e-6
        print(f"  {desc}")
        print(f"    d_L0={d0:.4f}  d_L1={d1:.4f}  true~={dt:.4f}   "
              f"(L0<=true:{v1}, L1<=true:{v2}, d0<=d1:{mono})")
        print(f"    -> certified clearance lower bound = {max(d0,d1):.4f}; "
              f"L1 recovered { (d1-d0):.4f} of slack")

    print()
    print("="*72)
    print("STEP 3 — EARLY-OUT rate + cost (random scene, margin query)")
    print("="*72)
    margin = 0.5
    # precompute A (L-shape at origin)
    N = 200
    decided_L0 = 0; calls_L0 = 0; calls_total = 0; safe = 0; within = 0
    valid = 0
    t_sum = 0.0
    for _ in range(N):
        bx = rng.uniform(-4, 4); by = rng.uniform(-4, 4); bz = rng.uniform(-1.5, 1.5)
        vB, fB = box(bx, by, bz, 0.6, 0.6, 0.6)
        B0 = hull_L0(vB, fB)
        B1 = hull_L1_pieces(vB, fB, [([bx-0.32, by-0.32, bz-0.32],
                                      [bx+0.32, by+0.32, bz+0.32])])
        r = query(A0, B0, A1, B1, margin)
        t_sum += r["t"]; calls_total += r["gjk_calls"]
        if r["level"] == "L0":
            decided_L0 += 1; calls_L0 += r["gjk_calls"]
        if r["decided"] == "SAFE": safe += 1
        else: within += 1
        # sanity: reported lower bound <= approx true (necessary condition)
        dt = true_distance_approx(vB, fB, vA, fA, n=1500)
        if r["lower"] <= dt + 1e-6: valid += 1

    print(f"  queries={N}, margin={margin}")
    print(f"  resolved at L0 (1 GJK, early-out): {decided_L0}/{N} = {100*decided_L0/N:.0f}%")
    print(f"  avg GJK calls/query = {calls_total/N:.2f}  (L0-only would be 1.0)")
    print(f"  lower-bound <= approx-true held: {valid}/{N}")
    print(f"  SAFE={safe}, WITHIN_MARGIN={within}")
    print(f"  avg query time = {1e3*t_sum/N:.3f} ms (pure-Python GJK; ratio matters, not abs)")


if __name__ == "__main__":
    main()
