"""
Honest diagnostic: certification (safety) AND tightness, across CoACD thresholds.

For each method/decomposition, over many probe placements, measure:
  - pieces
  - max vertex of ORIGINAL outside the decomposition (conservativeness magnitude)
  - % of probes where reported distance <= true   (safe lower bound)
  - max over-estimate (true - reported < 0 => unsafe clearance)
  - mean slack = true - reported on safe probes (tightness; smaller = tighter)

Two shapes: axis-aligned L (MeshExpander's home turf) and a ROTATED L
(box-carving's weak case) to expose the tradeoff.
"""
import sys, os, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import poc
import coacd
coacd.set_log_level("error")
rng = np.random.default_rng(2)


def coacd_pieces(v, f, threshold):
    m = coacd.Mesh(v.astype(np.float64), f.astype(np.int32))
    return [np.asarray(p[0], float) for p in
            coacd.run_coacd(m, threshold=threshold, merge=True, seed=0)]


def max_outside(orig_v, pieces):
    return max(min(poc.gjk_distance(p[None, :], pc) for pc in pieces) for p in orig_v)


def union_distance(pieces, B):
    return min(poc.gjk_distance(pc, B) for pc in pieces)


def evaluate(vA, fA, pieces, probes):
    safe = 0; over = 0.0; slack = []
    for (vB, fB, B, true) in probes:
        d = union_distance(pieces, B)
        if d <= true + 1e-6:
            safe += 1; slack.append(true - d)
        else:
            over = max(over, d - true)
    return dict(pieces=len(pieces),
                max_out=max_outside(vA, pieces),
                pct_safe=100*safe/len(probes),
                max_over=over,
                mean_slack=float(np.mean(slack)) if slack else float('nan'))


def make_probes(vA, fA, n, spread):
    probes = []
    for _ in range(n):
        c = rng.uniform(-spread, spread, 3); c[2] *= 0.4
        vB, fB = poc.box(c[0], c[1], c[2], 0.6, 0.6, 0.6)
        B = poc.hull_L0(vB, fB)
        true = poc.true_distance_approx(vA, fA, vB, fB, n=1500)
        probes.append((vB, fB, B, true))
    return probes


def rot_z(v, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return v @ R.T


def run_shape(title, vA, fA, me_pieces, n=120, spread=3.5):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)
    probes = make_probes(vA, fA, n, spread)
    rows = []
    r = evaluate(vA, fA, me_pieces, probes); r["name"] = "MeshExpander"; rows.append(r)
    for th in [0.02, 0.05, 0.10, 0.20]:
        cd = coacd_pieces(vA, fA, th)
        r = evaluate(vA, fA, cd, probes); r["name"] = f"CoACD th={th}"; rows.append(r)
    print(f"  {'method':16s} {'pieces':>6s} {'maxOutside':>11s} {'%safe':>7s} "
          f"{'maxOver(unsafe)':>16s} {'meanSlack(tight)':>16s}")
    for r in rows:
        print(f"  {r['name']:16s} {r['pieces']:6d} {r['max_out']:11.5f} "
              f"{r['pct_safe']:6.0f}% {r['max_over']:16.5f} {r['mean_slack']:16.4f}")


def main():
    # 1) axis-aligned L (MeshExpander home turf): L1 = 2-box carve
    vA, fA = poc.lshape(0, 0, 1.0)
    me_axis = poc.hull_L1_pieces(vA, fA, poc.lshape_cover(0, 0, 1.0))
    run_shape("SHAPE 1: axis-aligned L-shape", vA, fA, me_axis)

    # 2) rotated L (box-carving weak case): MeshExpander L0 single hull (axis cover invalid)
    vR = rot_z(poc.lshape(0, 0, 1.0)[0], 35.0); fR = poc.lshape(0, 0, 1.0)[1]
    me_rot = [poc.hull_L0(vR, fR)]  # only the guaranteed single hull (loose)
    run_shape("SHAPE 2: rotated L-shape (35°) — MeshExpander forced to single L0 hull",
              vR, fR, me_rot)


if __name__ == "__main__":
    main()
