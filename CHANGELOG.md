# Changelog

All notable changes to MeshExpander are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to
follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] — 2026-06-16

First public release: a conservative 3D mesh expansion library for machining
clearance models, with a C++17 core and Python bindings.

### Added
- **Core carving algorithm** (`BoxExpander`): face-normal box carving producing a
  single closed convex polytope that provably contains the input + distance `d`.
- **Anisotropic per-axis expansion** — `expand(mesh, [dx, dy, dz])` (C++ and
  Python). Offsets each axis-aligned face by its own component (ellipsoid support
  `‖n⊙d‖`); a uniform vector reduces exactly to the scalar ball expansion.
- **Concavity-driven decomposition** (`AssemblyExpander` + `BoxPartitioner`):
  `max_convex_pieces` / `concavity_tol` split a part into convex boxes.
- **Multi-part assemblies** (`AssemblyExpander`) with `mergeContained` for nested
  sub-features (holes, bosses).
- **CMake package** — `install()` + `MeshExpanderConfig.cmake`, so consumers can
  `find_package(MeshExpander)` and link `MeshExpander::mesh_expander`.
- **Python bindings**: `AlignedBox3d` (makes `BoxExpander.expand_box` callable),
  NumPy in/out, type stubs (`.pyi`), `HAS_IO` feature flag.
- Optional Assimp IO layer (`-DMESHEXPANDER_BUILD_IO=ON`) for STEP/OBJ/FBX and a
  CLI (`meshexpander_cli`).
- English README with a `日本語` switcher; `CONTRIBUTING`, `SECURITY`,
  `CODE_OF_CONDUCT`; bundled binary-STL sample (`examples/data/cube.stl`).

### Changed
- **Scale robustness**: each part is now normalized to a canonical size before
  clipping, so the absolute tolerances stay scale-relative. Conservativeness is
  verified from micrometer to kilometer scale (previously micron-scale inputs
  collapsed to an empty result). The library is now effectively unit-agnostic.
- **`mergeContained`** now requires the child's centroid to lie inside the parent
  mesh (point-in-mesh test), in addition to AABB containment — a part sitting in a
  parent's hollow/cavity is no longer wrongly absorbed.
- `read_stl` / `expand_file` now raise a clear error on ASCII or invalid STL
  instead of returning an empty mesh silently.

### Fixed
- `StlReader` now actually rejects ASCII STL via file-size validation (previously
  a no-op that could parse ASCII as garbage).
- `ClippingEngine` guards the O(n³) vertex enumeration against pathological
  half-space counts (clear exception instead of a hang/OOM).
- CLI (`src/cli/main.cpp`) compiled against removed options (`resolution`,
  `isConvex`); updated to the current `max_convex_pieces` / `concavity_tol` API.
- Documentation drift: removed stale `RobustSlicer` / `VoxelGrid` / "solid
  voxelization" references from the Japanese README, `pyproject.toml`, GIF titles,
  and code comments to match the shipped `BoxExpander` / `BoxPartitioner` design.

[0.1.0]: https://github.com/sho1106/MeshExpander/releases/tag/v0.1.0
