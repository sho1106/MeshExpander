# Can conservative carving give a better collision / distance primitive?

*An honest exploration log. Short answer: not for general collision — but it
yields a **certified-conservative distance lower bound**, and a useful
**conservative simplification** spin-off (now shipped as `ProgressiveHull`).*

This note records a hypothesis-driven investigation into whether MeshExpander's
**outer-hull guarantee** can produce a faster or safer collision/distance
primitive than the standard approach (approximate convex decomposition + GJK).
We report the negative results too, because the negative results are the point.

Method: PDCA + a hypothesis tree (`experiments/exploration/tree.json`). Each node
is a falsifiable hypothesis with a cheap experiment; we prune refuted branches
and expand confirmed ones. All numbers below are reproducible from the scripts in
`experiments/conservative_distance/`.

---

## 1. The one real property: a *certified* distance lower bound

MeshExpander expands a part into an **outer hull** `Â ⊇ A` (every input vertex is
contained — `Cov% = 100`). For two parts with hulls `Â ⊇ A`, `B̂ ⊇ B`:

```
dist(Â, B̂) ≤ dist(A, B)
```

i.e. the distance between the hulls is a **lower bound on the true distance**
(bigger sets can only be closer). If each hull is within one-sided Hausdorff `ε`
of its part, the bound is two-sided:

```
dist(A, B) − (εA + εB) ≤ dist(Â, B̂) ≤ dist(A, B)
```

So MeshExpander can return a **certified interval** `[d_hull, d_hull + εA + εB]`
whose width you control with the expansion/precision knob. For safety use
(ISO/TS 15066 speed-and-separation, control-barrier functions, conservative
time-of-impact), an under-estimate of clearance is the *safe* direction, and
"certified" — not "empirically usually right" — is what matters.

This is genuinely nice. The question was whether it beats the alternatives in
practice. It mostly does not.

---

## 2. What we tested, and what the data said

Leaf distance = a self-contained GJK (validated against analytic box cases).
Outer hulls = carving with `d ≈ 0`. Baseline = **CoACD** (collision-aware
approximate convex decomposition, SIGGRAPH 2022) + the same GJK. Metrics:
`max-outside` (max distance of an original vertex outside the decomposition;
`0` = certified conservative), `slack` (mean `true − reported`; smaller =
tighter), `%safe` (reported ≤ true).

### 2.1 "Only we are safe" — refuted

CoACD in convex-hull mode is **effectively conservative**: the convex hull of
each part over-covers, so its distance is *also* ~a lower bound. Observed
safety violations were ~`1e-4` (preprocessing remesh noise), i.e. negligible vs
any real safety margin.

| L-shape, 150 probes | reported ≤ true | max over-estimate |
|---|---|---|
| MeshExpander hull | **150 / 150** | 0 |
| CoACD | 116 / 150 | **1e-4** |

So the differentiator collapses from "they are unsafe" to "ours is *provably*
conservative, theirs is *empirically nearly* so". Real, but narrow.

### 2.2 Tightness — CoACD wins off-axis

The honest comparison is tightness at equal pieces. Carving is competitive only
on **axis-aligned** geometry:

| shape | CoACD slack | carving-leaf slack | note |
|---|---|---|---|
| axis-aligned L | 0.0131 | **0.0120** | carving wins, and is certified (max-outside 0) |
| rotated L (35°) | 0.0155 | 0.0302 | carving ~2× looser |
| torus (curved) | 0.0224 | 0.0341 | carving ~1.5× looser |

Cause: the carving leaf clips an **axis-aligned** box; it cannot hug an oriented
piece. We tried carving in each piece's **PCA-oriented** frame (hypothesis H7b) —
it did **not** help (rotated-L slack got *worse*, 0.0446), because the looseness
comes from capturing extra faces in the box region, not from box orientation.

### 2.3 Carving doesn't like curves

On a tessellated sphere the carving direction count explodes — at a 5° merge
angle it exceeded the engine's `C(n,3)` guard (496 > 256 half-spaces). Carving is
built for **flat, prismatic** faces, not smooth surfaces.

### 2.4 The spin-off that worked: conservative simplification

A carved model is an intersection of half-spaces; **removing one only enlarges
it**, so greedy least-volume removal reduces face count while staying a
conservative superset (volume monotonically grows):

| facets kept | faces | over-expansion |
|---|---|---|
| (start) | 336 | — |
| 48 | 178 | +4.7% |
| 32 | **110** | **+8.4%** |
| 16 | 52 | +24% |

This is more efficient than the face-normal merge-angle knob and is now shipped
as `ProgressiveHull` / `BoxExpander::expandSimplified`.

---

## 3. Conclusion

- **No general collision/distance wedge.** CoACD + GJK is already
  ~conservative and tighter on non-prismatic geometry; building a carving-based
  engine to beat it is not worth it.
- **Where carving wins:** **prismatic / axis-aligned (machining) geometry**,
  where it is both certified-conservative *and* competitive/tighter, with a
  bounded-low polygon count.
- **The certified lower-bound / interval** is a real, narrow asset for
  safety-critical use where a *proof* (not "usually right") is required.
- **Shipped result:** conservative `ProgressiveHull` simplification.

## Reproduce

```
PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/poc.py          # certified lower bound + early-out
PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/diag.py          # safety + tightness vs CoACD
PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/trial_hybrid.py  # carving leaf in CoACD cells (H7)
PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/trial_proghull.py# progressive-hull reduction
```
Requires the built `meshexpander` module plus `numpy`, `scipy`, `coacd`. The
hypothesis tree with every node's status and evidence is in
`experiments/exploration/tree.json`.

## References
- CoACD — Wei et al., *Approximate Convex Decomposition for 3D Meshes with
  Collision-Aware Concavity and Tree Search*, SIGGRAPH 2022. arXiv:2205.02961.
- *Progressive Convex Hull Simplification* (conservative, dual/H-rep), arXiv:2604.14468.
