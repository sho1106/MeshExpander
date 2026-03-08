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

>>> # Class API
>>> slicer = me.RobustSlicer.with_cell_size(0.005)
>>> expanded = slicer.expand_merged(mesh, d=0.002)
"""

from .meshexpander_core import (  # noqa: F401
    Mesh,
    RobustSlicer,
    ConservativeExpander,
    read_stl,
    write_stl,
    expand_file,
    expand_np,
)

__all__ = [
    "Mesh",
    "RobustSlicer",
    "ConservativeExpander",
    "read_stl",
    "write_stl",
    "expand_file",
    "expand_np",
]
