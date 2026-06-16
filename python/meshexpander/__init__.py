"""MeshExpander — conservative 3D mesh expansion.

Quick start
-----------
>>> import meshexpander as me

>>> # Expand an STL file (d is in the model's own units)
>>> result = me.expand_file("model.stl", d=1.0)
>>> me.write_stl("expanded.stl", result)

>>> # NumPy API — d may be a scalar or per-axis [dx, dy, dz]
>>> import numpy as np
>>> out_verts, out_faces = me.expand_np(verts, faces, d=1.0)

>>> # Multi-part assembly API (Assimp IO — requires MESHEXPANDER_BUILD_IO=ON)
>>> parts = me.load_assembly("assembly.dae")   # auto-split by scene graph
>>> parts = me.merge_contained(parts)          # merge nested sub-parts
>>> result = me.expand_assembly_merged(parts, d=1.0)
>>> me.write_stl("expanded.stl", result)
"""

from .meshexpander_core import (  # noqa: F401
    Mesh,
    AlignedBox3d,
    BoxExpander,
    AssemblyExpander,
    AssemblyExpanderOptions,
    HAS_IO,
    read_stl,
    write_stl,
    expand_file,
    expand_np,
    merge_contained,
    expand_assembly,
    expand_assembly_merged,
)

__all__ = [
    "Mesh",
    "AlignedBox3d",
    "BoxExpander",
    "AssemblyExpander",
    "AssemblyExpanderOptions",
    "HAS_IO",
    "read_stl",
    "write_stl",
    "expand_file",
    "expand_np",
    "merge_contained",
    "expand_assembly",
    "expand_assembly_merged",
]

# load_assembly is only available when the package was built with Assimp (HAS_IO=True)
if HAS_IO:
    from .meshexpander_core import load_assembly  # noqa: F401
    __all__.append("load_assembly")
