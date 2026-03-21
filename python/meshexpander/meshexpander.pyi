"""Type stubs for meshexpander C++ extension."""

from typing import Sequence
import numpy as np
from numpy.typing import NDArray

class Mesh:
    """3D mesh with vertices and triangular faces."""

    vertices: list[list[float]]
    faces: list[list[int]]

    def __init__(
        self,
        vertices: Sequence[Sequence[float]],
        faces: Sequence[Sequence[int]],
    ) -> None: ...

class ConservativeExpander:
    """Conservative expander for convex shapes (face-normal mode)."""

    def __init__(self) -> None: ...
    def expand(self, mesh: Mesh, d: float) -> Mesh:
        """Expand mesh by distance d. Returns single closed polyhedron."""
        ...

class RobustSlicer:
    """Conservative expander for concave shapes via solid voxelization."""

    def __init__(self, resolution: int = 64, safety_margin: float = 1e-4) -> None: ...

    @classmethod
    def with_cell_size(cls, cell_size: float, safety_margin: float = 1e-4) -> "RobustSlicer":
        """Create slicer with explicit voxel cell size instead of resolution."""
        ...

    def expand(self, mesh: Mesh, d: float) -> Mesh:
        """Expand mesh (returns first box; use expand_multi for full result)."""
        ...

    def expand_multi(self, mesh: Mesh, d: float) -> list[Mesh]:
        """Expand concave mesh. Returns list of independent closed polyhedra."""
        ...

    def expand_merged(self, mesh: Mesh, d: float) -> Mesh:
        """Expand and merge all boxes into a single mesh object."""
        ...

def read_stl(path: str) -> Mesh:
    """Read binary or ASCII STL file."""
    ...

def write_stl(path: str, mesh: Mesh) -> None:
    """Write mesh as binary STL file."""
    ...

def expand_file(
    path: str,
    d: float,
    cell_size: float | None = None,
    resolution: int = 64,
) -> list[Mesh]:
    """High-level: read STL, expand with RobustSlicer, return result meshes."""
    ...

def expand_np(
    vertices: NDArray[np.float64],
    faces: NDArray[np.int32],
    d: float,
    cell_size: float | None = None,
    resolution: int = 64,
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    """NumPy API: expand mesh, return (vertices, faces) arrays of merged result."""
    ...
