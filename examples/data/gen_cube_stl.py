"""Generate a binary STL of a unit cube (10mm) — bundled sample for the Quick Start.

Run:  python examples/data/gen_cube_stl.py
Produces: examples/data/cube.stl  (binary STL, 12 triangles)
"""
import struct
from pathlib import Path

# 10mm cube centred at origin
s = 5.0
v = [
    (-s, -s, -s), (s, -s, -s), (s, s, -s), (-s, s, -s),  # bottom
    (-s, -s,  s), (s, -s,  s), (s, s,  s), (-s, s,  s),  # top
]
# 12 triangles (two per face), CCW outward
tris = [
    (0, 3, 2), (0, 2, 1),  # -z
    (4, 5, 6), (4, 6, 7),  # +z
    (0, 1, 5), (0, 5, 4),  # -y
    (3, 7, 6), (3, 6, 2),  # +y
    (1, 2, 6), (1, 6, 5),  # +x
    (0, 4, 7), (0, 7, 3),  # -x
]

out = Path(__file__).with_name("cube.stl")
with out.open("wb") as f:
    f.write(b"MeshExpander sample cube".ljust(80, b"\0"))
    f.write(struct.pack("<I", len(tris)))
    for a, b, c in tris:
        f.write(struct.pack("<3f", 0.0, 0.0, 0.0))  # normal (ignored by reader)
        for idx in (a, b, c):
            f.write(struct.pack("<3f", *v[idx]))
        f.write(struct.pack("<H", 0))
print(f"wrote {out} ({out.stat().st_size} bytes, {len(tris)} triangles)")
