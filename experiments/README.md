# experiments/

Research scratch behind the design notes in `docs/research/`. These are
exploratory scripts (not part of the library or its test suite), kept for
reproducibility and honesty about what was tried.

## Layout
- `exploration/tree.json` — the hypothesis tree (each node: a falsifiable
  hypothesis, its cheap test, status, and evidence). The index of the whole
  investigation.
- `conservative_distance/` — Python trials comparing MeshExpander's conservative
  outer hulls against CoACD for distance/collision, plus the progressive-hull
  poly-reduction trial. Findings written up in
  [`docs/research/conservative-distance.md`](../docs/research/conservative-distance.md).

## Running
```
PYTHONUTF8=1 PYTHONPATH=python python experiments/conservative_distance/<script>.py
```
Dependencies: the built `meshexpander` Python module (see project README) plus
`numpy`, `scipy`, and `coacd`. `poc.py` holds shared helpers (a self-contained
GJK, mesh generators, hull builders); the other scripts import it.
