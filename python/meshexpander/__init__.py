"""MeshExpander — conservative 3D mesh expansion.

Quick start
-----------
>>> import meshexpander as me

>>> # Expand an STL file
>>> result = me.expand_file("model.stl", d=0.002)
>>> me.write_stl("expanded.stl", result)

>>> # NumPy API
>>> import numpy as np
>>> out_verts, out_faces = me.expand_np(verts, faces, d=0.002)

>>> # Multi-part assembly API (Assimp IO — requires MESHEXPANDER_BUILD_IO=ON)
>>> parts = me.load_assembly("assembly.dae")   # auto-split by scene graph
>>> parts = me.merge_contained(parts)          # merge nested sub-parts
>>> result = me.expand_assembly_merged(parts, d=0.002)
>>> me.write_stl("expanded.stl", result)
"""

from .meshexpander_core import (  # noqa: F401
    Mesh,
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
