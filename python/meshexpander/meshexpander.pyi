"""Type stubs for meshexpander C++ extension."""

from typing import Sequence
import numpy as np
from numpy.typing import NDArray


class Mesh:
    """3D mesh with vertices and triangular faces."""

    vertices: NDArray[np.float64]   # shape (N, 3)
    faces:    NDArray[np.int32]     # shape (M, 3)

    def __init__(self) -> None: ...
    def num_vertices(self) -> int: ...
    def num_faces(self) -> int: ...
    def empty(self) -> bool: ...

    def to_arrays(self) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
        """Returns (vertices ndarray[N,3], faces ndarray[M,3])."""
        ...

    @staticmethod
    def from_arrays(
        vertices: NDArray[np.float64],
        faces: NDArray[np.int32],
    ) -> "Mesh":
        """Create Mesh from numpy arrays."""
        ...

    def __repr__(self) -> str: ...


class ConservativeExpander:
    """Conservative expander for convex shapes (face-normal mode)."""

    def __init__(self) -> None: ...
    def expand(self, mesh: Mesh, d: float) -> Mesh:
        """Expand convex mesh by distance d. Returns single closed polyhedron."""
        ...


class RobustSlicer:
    """Conservative expander for concave shapes via solid voxelization."""

    def __init__(
        self,
        resolution: int = 64,
        face_normal_merge_deg: float = 20.0,
    ) -> None: ...

    @classmethod
    def with_cell_size(
        cls,
        cell_size: float,
        face_normal_merge_deg: float = 20.0,
    ) -> "RobustSlicer":
        """Create slicer with explicit world-space voxel cell size."""
        ...

    def expand(self, mesh: Mesh, d: float) -> Mesh:
        """Expand mesh (IExpander compat — returns first polytope)."""
        ...

    def expand_multi(self, mesh: Mesh, d: float) -> list[Mesh]:
        """Expand concave mesh into a list of independent closed polytopes."""
        ...

    def expand_merged(self, mesh: Mesh, d: float) -> Mesh:
        """Expand and merge all polytopes into one STL-ready mesh."""
        ...


class AssemblyExpanderOptions:
    """Options controlling AssemblyExpander behaviour."""

    resolution: int                 # Voxel resolution for concave parts [default: 64]
    cell_size_world: float          # Fixed voxel cell size (overrides resolution when > 0)
    face_normal_merge_deg: float    # Angle threshold for merging near-parallel normals
    convex_tol: float               # Tolerance for isConvex() test

    def __init__(self) -> None: ...


class AssemblyExpander:
    """Conservative expansion for multi-part 3D assemblies.

    Each part is expanded independently using the optimal algorithm:
    - Convex parts  → ConservativeExpander (single polytope, low polygon count)
    - Concave parts → RobustSlicer         (voxel-based, concavity-aware)

    Examples
    --------
    >>> exp = AssemblyExpander()
    >>> parts = [mesh_a, mesh_b]
    >>> parts = AssemblyExpander.merge_contained(parts)
    >>> result = exp.expand_merged(parts, d=0.002)
    """

    def __init__(self, options: AssemblyExpanderOptions | None = None) -> None: ...

    def expand(self, parts: list[Mesh], d: float) -> list[Mesh]:
        """Expand each part independently. Returns one Mesh per input part."""
        ...

    def expand_merged(self, parts: list[Mesh], d: float) -> Mesh:
        """Expand all parts and concatenate into a single multi-body mesh."""
        ...

    @staticmethod
    def merge_contained(
        parts: list[Mesh],
        tolerance: float = 1e-6,
    ) -> list[Mesh]:
        """Merge parts whose bounding box is fully contained within another part's."""
        ...

    @staticmethod
    def is_convex(mesh: Mesh, tol: float = 1e-6) -> bool:
        """Return True if mesh is approximately convex."""
        ...


# ── STL I/O ──────────────────────────────────────────────────────────────────

def read_stl(path: str) -> Mesh:
    """Read binary STL file, return Mesh."""
    ...

def write_stl(
    path: str,
    mesh: Mesh,
    header: str = "MeshExpander output",
) -> None:
    """Write Mesh to binary STL file."""
    ...


# ── Single-mesh convenience ───────────────────────────────────────────────────

def expand_file(
    input_path: str,
    d: float,
    cell_size: float = 0.005,
    output_path: str = "",
) -> Mesh:
    """Read STL → expand with RobustSlicer → optionally write → return Mesh."""
    ...

def expand_np(
    vertices: NDArray[np.float64],
    faces: NDArray[np.int32],
    d: float,
    cell_size: float = 0.005,
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    """Expand mesh from numpy arrays. Returns (vertices[N2,3], faces[M2,3])."""
    ...


# ── Multi-part assembly convenience ──────────────────────────────────────────

def is_convex(mesh: Mesh, tol: float = 1e-6) -> bool:
    """Return True if mesh is approximately convex."""
    ...

def merge_contained(
    parts: list[Mesh],
    tolerance: float = 1e-6,
) -> list[Mesh]:
    """Merge parts whose bounding box is fully contained within another part's."""
    ...

def expand_assembly(
    parts: list[Mesh],
    d: float,
    resolution: int = 64,
    cell_size_world: float = 0.0,
    face_normal_merge_deg: float = 20.0,
) -> list[Mesh]:
    """Expand each part independently. Returns list[Mesh]."""
    ...

def expand_assembly_merged(
    parts: list[Mesh],
    d: float,
    resolution: int = 64,
    cell_size_world: float = 0.0,
    face_normal_merge_deg: float = 20.0,
) -> Mesh:
    """Expand all parts and merge into one multi-body Mesh."""
    ...
