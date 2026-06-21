"""
Decisive comparison: is the distance a *certified safe lower bound*?

  MeshExpander (outer-hull carving)  vs  CoACD (approximate convex decomposition)

Claim under test:
  - MeshExpander pieces CONTAIN the original (conservative) => distance <= true ALWAYS (safe).
  - CoACD pieces APPROXIMATE the original (cut through it) => distance can EXCEED true
    (unsafe over-estimate: reports more clearance than exists).

Distance leaf = the POC's validated GJK (apples-to-apples, no FCL needed).

Run: PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/compare_coacd.py
"""
import sys, os, time
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import poc  # gjk_distance, box, lshape, hull_L0, hull_L1_pieces, lshape_cover, true_distance_approx

import coacd
coacd.set_log_level("error")
rng = np.random.default_rng(1)


def coacd_pieces(v, f, threshold=0.05):
    m = coacd.Mesh(v.astype(np.float64), f.astype(np.int32))
    parts = coacd.run_coacd(m, threshold=threshold, merge=True, seed=0)
    return [np.asarray(p[0], float) for p in parts]  # list of vertex arrays


def point_outside_dist(p, pieces):
    """0 if p is inside the union of convex pieces, else min distance to the union."""
    return min(poc.gjk_distance(p[None, :], pc) for pc in pieces)


def coverage(orig_v, pieces):
    outside = np.array([point_outside_dist(p, pieces) for p in orig_v])
    covered = int((outside <= 1e-6).sum())
    return covered, len(orig_v), float(outside.max())


def union_distance(pieces, B):
    return min(poc.gjk_distance(pc, B) for pc in pieces)


def main():
    print("=" * 74)
    print("CONSERVATIVENESS COMPARISON  —  MeshExpander vs CoACD  (L-shape)")
    print("=" * 74)
    vA, fA = poc.lshape(0, 0, 1.0)

    # --- decompositions ---
    me_L0 = [poc.hull_L0(vA, fA)]                       # 1 conservative hull
    me_L1 = poc.hull_L1_pieces(vA, fA, poc.lshape_cover(0, 0, 1.0))  # 2 conservative pieces
    t0 = time.perf_counter(); cd = coacd_pieces(vA, fA, threshold=0.05)
    t_cd = time.perf_counter() - t0

    print(f"  pieces:  MeshExpander L0={len(me_L0)}  L1={len(me_L1)}   CoACD={len(cd)} "
          f"(decomp {t_cd*1e3:.0f} ms)")

    print("\n  -- vertex coverage of the ORIGINAL mesh (conservative ⇔ 100%, 0 outside) --")
    for name, pieces in [("MeshExpander L0", me_L0), ("MeshExpander L1", me_L1),
                         ("CoACD", cd)]:
        cov, tot, mx = coverage(vA, pieces)
        flag = "CONSERVATIVE" if cov == tot and mx <= 1e-6 else "NOT conservative"
        print(f"    {name:16s}: {cov}/{tot} verts inside, max-outside={mx:.4f}  -> {flag}")

    print("\n" + "=" * 74)
    print("DISTANCE SAFETY  —  is reported distance a lower bound on the true distance?")
    print("=" * 74)
    N = 150
    margin_violations_me = 0
    margin_violations_cd = 0
    over_cd_max = 0.0
    under_me_max = 0.0
    me_valid = cd_valid = 0
    for _ in range(N):
        bx = rng.uniform(-3.5, 3.5); by = rng.uniform(-3.5, 3.5); bz = rng.uniform(-1.2, 1.2)
        vB, fB = poc.box(bx, by, bz, 0.6, 0.6, 0.6)
        B = poc.hull_L0(vB, fB)                       # B is convex; its hull == itself
        true = poc.true_distance_approx(vA, fA, vB, fB, n=1500)
        d_me = union_distance(me_L1, B)
        d_cd = union_distance(cd, B)
        # safe lower bound  <=> reported <= true
        if d_me <= true + 1e-6: me_valid += 1
        else: under_me_max = max(under_me_max, d_me - true)
        if d_cd <= true + 1e-6: cd_valid += 1
        else: over_cd_max = max(over_cd_max, d_cd - true)

    print(f"  probes={N}")
    print(f"  MeshExpander L1: reported <= true (SAFE) in {me_valid}/{N}   "
          f"max unsafe over-estimate={under_me_max:.4f}")
    print(f"  CoACD          : reported <= true (SAFE) in {cd_valid}/{N}   "
          f"max unsafe over-estimate={over_cd_max:.4f}")
    print("\n  Interpretation:")
    print("   - MeshExpander should be 100% SAFE (never reports more clearance than real).")
    print("   - CoACD, being non-conservative, can over-estimate clearance => unsafe for")
    print("     safety-critical separation checks. That gap is the MeshExpander wedge.")


if __name__ == "__main__":
    main()
