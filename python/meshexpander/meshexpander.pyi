"""Type stubs for meshexpander C++ extension."""

from typing import Optional
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


class BoxExpander:
    """削り出し法コア機能 (Carving Expansion Core).

    Expands a box by d and carves it with local face normals from the mesh
    faces that overlap the box, producing a single closed convex polytope.
    """

    def __init__(self, face_normal_merge_deg: float = 20.0) -> None: ...

    def expand(self, mesh: Mesh, d: float) -> Mesh:
        """Expand mesh using its own AABB as the initial box."""
        ...

    def expand_box(self, box, mesh: Mesh, d: float) -> Mesh:
        """Expand a specific AABB using nearby face normals from mesh."""
        ...


class AssemblyExpanderOptions:
    """Options controlling AssemblyExpander behaviour."""

    face_normal_merge_deg: float
    """Angle threshold for merging near-parallel face normals (degrees)."""
    concavity_tol: float
    """Concave support: split until each region's concavity is below this
    tolerance (world units). 0 keeps single-convex behaviour (default)."""
    max_convex_pieces: int
    """Concave support: upper bound on convex pieces per part. 1 = single
    convex (default); >1 enables concavity-driven box decomposition."""

    def __init__(self) -> None: ...


class AssemblyExpander:
    """Conservative expansion for multi-part 3D assemblies.

    By default each part is expanded with BoxExpander (削り出し法) into a single
    convex polytope. Setting concavity_tol>0 or max_convex_pieces>1 enables
    concavity-driven box decomposition so concave parts are expanded as a union
    of convex pieces (fewer wasted volume, still conservative).

    Examples
    --------
    >>> opts = AssemblyExpanderOptions()
    >>> opts.max_convex_pieces = 8
    >>> exp = AssemblyExpander(opts)
    >>> parts = AssemblyExpander.merge_contained([mesh_a, mesh_b])
    >>> result = exp.expand_merged(parts, d=0.002)
    """

    def __init__(self, options: Optional[AssemblyExpanderOptions] = None) -> None: ...

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


# ── Build feature flags ───────────────────────────────────────────────────────

HAS_IO: bool
"""True if the package was built with MESHEXPANDER_BUILD_IO=ON (Assimp available).
load_assembly() is only available when HAS_IO is True."""


# ── Assembly file I/O (requires HAS_IO=True) ─────────────────────────────────

def load_assembly(path: str) -> list[Mesh]:
    """Load assembly file via Assimp, return one Mesh per scene-graph part.

    Supported formats: DAE, FBX, OBJ, GLTF, STL, and all Assimp-supported formats.
    Node transforms are accumulated so all vertices are in world space.

    Raises
    ------
    RuntimeError
        If the file cannot be read or the package was built without Assimp.
    """
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
    output_path: str = "",
) -> Mesh:
    """Read STL → expand with BoxExpander → optionally write → return Mesh."""
    ...

def expand_np(
    vertices: NDArray[np.float64],
    faces: NDArray[np.int32],
    d: float,
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    """Expand mesh from numpy arrays. Returns (vertices[N2,3], faces[M2,3])."""
    ...


# ── Multi-part assembly convenience ──────────────────────────────────────────

def merge_contained(
    parts: list[Mesh],
    tolerance: float = 1e-6,
) -> list[Mesh]:
    """Merge parts whose bounding box is fully contained within another part's."""
    ...

def expand_assembly(
    parts: list[Mesh],
    d: float,
    face_normal_merge_deg: float = 20.0,
    concavity_tol: float = 0.0,
    max_convex_pieces: int = 1,
) -> list[Mesh]:
    """Expand each part independently. Returns list[Mesh].

    Set concavity_tol>0 or max_convex_pieces>1 for concave parts.
    """
    ...

def expand_assembly_merged(
    parts: list[Mesh],
    d: float,
    face_normal_merge_deg: float = 20.0,
    concavity_tol: float = 0.0,
    max_convex_pieces: int = 1,
) -> Mesh:
    """Expand all parts and merge into one multi-body Mesh.

    Set concavity_tol>0 or max_convex_pieces>1 for concave parts.
    """
    ...
