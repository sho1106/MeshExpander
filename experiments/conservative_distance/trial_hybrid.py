"""
H7: "CoACD cut structure + conservative carving leaf".
For each CoACD piece, take its AABB and CARVE the original mesh within it (d~0),
giving a guaranteed-conservative piece. Question: does this give max-outside=0
(certified) at CoACD-comparable tightness, across axis / rotated / curved shapes?
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import poc
import coacd
coacd.set_log_level("error")
rng = np.random.default_rng(7)


def torus(R=1.5, r=0.55, nt=28, nph=14):
    V = []
    for i in range(nt):
        th = 2*np.pi*i/nt
        for j in range(nph):
            ph = 2*np.pi*j/nph
            V.append([(R+r*np.cos(ph))*np.cos(th), (R+r*np.cos(ph))*np.sin(th), r*np.sin(ph)])
    V = np.array(V, float)
    F = []
    for i in range(nt):
        for j in range(nph):
            a = i*nph+j; b = ((i+1) % nt)*nph+j
            c = ((i+1) % nt)*nph+((j+1) % nph); d = i*nph+((j+1) % nph)
            F += [[a, b, c], [a, c, d]]
    return V, np.array(F, np.int32)


def rot_z(v, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return v @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T


def coacd_pieces(v, f, th=0.05):
    m = coacd.Mesh(v.astype(np.float64), f.astype(np.int32))
    return [np.asarray(p[0], float) for p in coacd.run_coacd(m, threshold=th, merge=True, seed=0)]


def carve_in_aabbs(v, f, piece_sets, pad=1e-4):
    """For each piece's AABB, conservatively carve the original mesh (d~0)."""
    mesh = me_mesh(v, f)
    exp = poc.me.BoxExpander()
    out = []
    for P in piece_sets:
        mn = P.min(0) - pad; mx = P.max(0) + pad
        b = poc.me.AlignedBox3d(mn.astype(float), mx.astype(float))
        r = exp.expand_box(b, mesh, 1e-9)
        if not r.empty():
            out.append(r.vertices)
    return out


def me_mesh(v, f):
    return poc.me.Mesh.from_arrays(v.astype(np.float64), f.astype(np.int32))


def max_outside(orig_v, pieces):
    return max(min(poc.gjk_distance(p[None, :], pc) for pc in pieces) for p in orig_v)


def eval_pieces(vA, fA, pieces, probes):
    slack = []
    for (B, true) in probes:
        d = min(poc.gjk_distance(pc, B) for pc in pieces)
        if d <= true + 1e-6:
            slack.append(true - d)
    return dict(pieces=len(pieces), max_out=max_outside(vA, pieces),
                slack=float(np.mean(slack)) if slack else float('nan'),
                pct_safe=100*len(slack)/len(probes))


def make_probes(vA, fA, n, spread):
    out = []
    for _ in range(n):
        c = rng.uniform(-spread, spread, 3); c[2] *= 0.5
        vB, fB = poc.box(c[0], c[1], c[2], 0.6, 0.6, 0.6)
        out.append((poc.hull_L0(vB, fB), poc.true_distance_approx(vA, fA, vB, fB, n=1200)))
    return out


def run(name, vA, fA, spread):
    print(f"\n### {name}", flush=True)
    probes = make_probes(vA, fA, 80, spread)
    cd = coacd_pieces(vA, fA, 0.05)
    hyb = carve_in_aabbs(vA, fA, cd)
    rc = eval_pieces(vA, fA, cd, probes)
    rh = eval_pieces(vA, fA, hyb, probes)
    print(f"  {'method':28s} {'pieces':>6s} {'maxOutside':>11s} {'%safe':>6s} {'meanSlack':>10s}", flush=True)
    print(f"  {'CoACD raw':28s} {rc['pieces']:6d} {rc['max_out']:11.5f} {rc['pct_safe']:5.0f}% {rc['slack']:10.4f}", flush=True)
    print(f"  {'CoACD-AABB + carving (H7)':28s} {rh['pieces']:6d} {rh['max_out']:11.5f} {rh['pct_safe']:5.0f}% {rh['slack']:10.4f}", flush=True)


def main():
    print("="*78, flush=True)
    print("H7 — CoACD cut structure + conservative carving leaf", flush=True)
    print("="*78, flush=True)
    vL, fL = poc.lshape(0, 0, 1.0)
    run("axis-aligned L", vL, fL, 3.5)
    vR = rot_z(vL, 35.0); run("rotated L (35deg)", vR, fL, 3.5)
    vT, fT = torus(); run("torus (curved, concave)", vT, fT, 3.2)
    print("\nDONE", flush=True)


if __name__ == "__main__":
    main()
