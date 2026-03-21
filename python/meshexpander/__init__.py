"""MeshExpander — conservative 3D mesh expansion.

Quick start
-----------
>>> import meshexpander as me

>>> # Expand an STL file
>>> result = me.expand_file("model.stl", d=0.002, cell_size=0.005)
>>> me.write_stl("expanded.stl", result)

>>> # NumPy API
>>> import numpy as np
>>> out_verts, out_faces = me.expand_np(verts, faces, d=0.002)

>>> # Multi-part assembly API
>>> parts = [me.Mesh.from_arrays(v1, f1), me.Mesh.from_arrays(v2, f2)]
>>> parts = me.merge_contained(parts)           # merge nested sub-parts
>>> result = me.expand_assembly_merged(parts, d=0.002)
>>> me.write_stl("expanded.stl", result)
"""

from .meshexpander_core import (  # noqa: F401
    Mesh,
    RobustSlicer,
    ConservativeExpander,
    AssemblyExpander,
    AssemblyExpanderOptions,
    read_stl,
    write_stl,
    expand_file,
    expand_np,
    is_convex,
    merge_contained,
    expand_assembly,
    expand_assembly_merged,
)

__all__ = [
    "Mesh",
    "RobustSlicer",
    "ConservativeExpander",
    "AssemblyExpander",
    "AssemblyExpanderOptions",
    "read_stl",
    "write_stl",
    "expand_file",
    "expand_np",
    "is_convex",
    "merge_contained",
    "expand_assembly",
    "expand_assembly_merged",
]
