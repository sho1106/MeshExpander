#!/usr/bin/env python3
"""
ply_to_stl.py -- Convert ASCII PLY mesh files to binary STL.

Designed for the Stanford Bunny PLY files from:
  http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz

Usage:
  python3 scripts/ply_to_stl.py <input.ply> <output.stl>
  python3 scripts/ply_to_stl.py --all <bunny_dir> <output_dir>

The --all form converts all bun_zipper*.ply files found under <bunny_dir>
to binary STL, writing bunny.stl / bunny_res2.stl / bunny_res3.stl /
bunny_res4.stl into <output_dir>.
"""

import os
import struct
import sys


# ---------------------------------------------------------------------------
# PLY parser (ASCII format only)
# ---------------------------------------------------------------------------

def read_ply(path):
    """Parse ASCII PLY. Returns (vertices, faces).
    vertices: list of (x, y, z) float tuples
    faces:    list of (i0, i1, i2) int tuples  (quads split into 2 triangles)
    """
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = fh.readlines()

    # --- header ---
    n_verts = 0
    n_faces = 0
    n_vert_props = 0   # number of per-vertex property lines
    header_end = 0
    in_vertex = False
    in_face = False

    for i, raw in enumerate(lines):
        tok = raw.strip().split()
        if not tok:
            continue
        kw = tok[0]
        if kw == "end_header":
            header_end = i + 1
            break
        if kw == "element":
            in_vertex = (tok[1] == "vertex")
            in_face   = (tok[1] == "face")
            if in_vertex:
                n_verts = int(tok[2])
            elif in_face:
                n_faces = int(tok[2])
        elif kw == "property":
            if in_vertex:
                n_vert_props += 1

    # --- vertices ---
    vertices = []
    for j in range(n_verts):
        parts = lines[header_end + j].split()
        vertices.append((float(parts[0]), float(parts[1]), float(parts[2])))

    # --- faces ---
    faces = []
    offset = header_end + n_verts
    for j in range(n_faces):
        parts = lines[offset + j].split()
        n = int(parts[0])
        idx = [int(parts[k + 1]) for k in range(n)]
        if n == 3:
            faces.append((idx[0], idx[1], idx[2]))
        elif n == 4:                     # quad -> 2 triangles
            faces.append((idx[0], idx[1], idx[2]))
            faces.append((idx[0], idx[2], idx[3]))
        # n > 4: skip (shouldn't appear in bunny)

    return vertices, faces


# ---------------------------------------------------------------------------
# Binary STL writer
# ---------------------------------------------------------------------------

def write_binary_stl(path, vertices, faces, label="mesh"):
    """Write binary STL (80-byte header + n_tris + per-triangle data)."""
    header = label.encode("ascii")[:80].ljust(80, b"\x00")
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(faces)))
        for f in faces:
            v0, v1, v2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
            fh.write(struct.pack("<fff", 0.0, 0.0, 0.0))  # normal (ignored)
            fh.write(struct.pack("<fff", *v0))
            fh.write(struct.pack("<fff", *v1))
            fh.write(struct.pack("<fff", *v2))
            fh.write(struct.pack("<H", 0))               # attribute


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def convert_one(src, dst):
    print(f"  {os.path.basename(src)} -> {os.path.basename(dst)}", flush=True)
    verts, faces = read_ply(src)
    print(f"    {len(verts):,} vertices, {len(faces):,} faces", flush=True)
    write_binary_stl(dst, verts, faces, label=os.path.basename(src))


def convert_all(bunny_dir, out_dir):
    """Convert all bun_zipper*.ply in bunny_dir to binary STL in out_dir."""
    os.makedirs(out_dir, exist_ok=True)

    # Map PLY filename -> STL output name
    name_map = {
        "bun_zipper.ply":      "bunny.stl",
        "bun_zipper_res2.ply": "bunny_res2.stl",
        "bun_zipper_res3.ply": "bunny_res3.stl",
        "bun_zipper_res4.ply": "bunny_res4.stl",
    }

    found = False
    for ply_name, stl_name in name_map.items():
        src = os.path.join(bunny_dir, ply_name)
        if not os.path.isfile(src):
            print(f"  [skip] {ply_name} not found")
            continue
        dst = os.path.join(out_dir, stl_name)
        convert_one(src, dst)
        found = True

    if not found:
        print(f"ERROR: no bun_zipper*.ply found under {bunny_dir}", file=sys.stderr)
        sys.exit(1)


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)

    if args[0] == "--all":
        if len(args) < 3:
            print("Usage: ply_to_stl.py --all <bunny_dir> <output_dir>",
                  file=sys.stderr)
            sys.exit(1)
        convert_all(args[1], args[2])
    else:
        if len(args) < 2:
            print("Usage: ply_to_stl.py <input.ply> <output.stl>",
                  file=sys.stderr)
            sys.exit(1)
        os.makedirs(os.path.dirname(os.path.abspath(args[1])), exist_ok=True)
        convert_one(args[0], args[1])


if __name__ == "__main__":
    main()
