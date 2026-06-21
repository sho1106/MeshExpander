"""
H7b: carve each CoACD piece in its ORIENTED (PCA) frame instead of an axis box.
Rigid transform preserves containment -> still certified; oriented box should
hug oriented/curved pieces better -> recover tightness lost by H7 (axis box).
Compare CoACD-raw vs H7(axis) vs H7b(PCA) on rotated L and torus.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import poc
import trial_hybrid as th   # reuse torus, rot_z, coacd_pieces, carve_in_aabbs, eval_pieces, make_probes
me = poc.me


def pca_frame(P):
    c = P.mean(0)
    X = P - c
    _, V = np.linalg.eigh(X.T @ X)      # columns = eigenvectors (ascending)
    R = V                                # world->frame: xf = (x-c) @ R
    if np.linalg.det(R) < 0:             # keep right-handed
        R = R.copy(); R[:, 0] = -R[:, 0]
    return c, R


def carve_in_obbs(v, f, piece_sets, pad=1e-4):
    """Carve original mesh inside each piece's PCA-oriented box; return hull verts in WORLD."""
    out = []
    for P in piece_sets:
        c, R = pca_frame(P)
        Vf = (v - c) @ R                          # original mesh in piece frame
        Pf = (P - c) @ R
        mn = Pf.min(0) - pad; mx = Pf.max(0) + pad
        meshf = me.Mesh.from_arrays(Vf.astype(np.float64), f.astype(np.int32))
        b = me.AlignedBox3d(mn.astype(float), mx.astype(float))
        r = me.BoxExpander().expand_box(b, meshf, 1e-9)
        if not r.empty():
            out.append(r.vertices @ R.T + c)      # back to world
    return out


def run(name, vA, fA, spread):
    print(f"\n### {name}", flush=True)
    probes = th.make_probes(vA, fA, 80, spread)
    cd = th.coacd_pieces(vA, fA, 0.05)
    h_axis = th.carve_in_aabbs(vA, fA, cd)
    h_obb = carve_in_obbs(vA, fA, cd)
    for label, pcs in [("CoACD raw", cd), ("H7 axis-box carve", h_axis),
                       ("H7b PCA-box carve", h_obb)]:
        r = th.eval_pieces(vA, fA, pcs, probes)
        print(f"  {label:22s} pieces={r['pieces']:3d} maxOut={r['max_out']:.5f} "
              f"%safe={r['pct_safe']:.0f} slack={r['slack']:.4f}", flush=True)


def main():
    print("="*70, flush=True)
    print("H7b — oriented (PCA) carving leaf", flush=True)
    print("="*70, flush=True)
    vL, fL = poc.lshape(0, 0, 1.0)
    run("rotated L (35deg)", th.rot_z(vL, 35.0), fL, 3.5)
    vT, fT = th.torus(); run("torus (curved)", vT, fT, 3.2)
    print("\nDONE", flush=True)


if __name__ == "__main__":
    main()
