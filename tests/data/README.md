# Test Data

This directory holds large mesh files used by integration tests.
These files are NOT committed to the repository.

## Stanford Bunny (bunny.stl)

Used by: `tests/integration/test_stanford_bunny.cpp`

### Download (Option A — MeshLab)

1. Download `bun_zipper_res4.ply` from the Stanford 3D Scanning Repository:
   http://graphics.stanford.edu/data/3Dscanrep/bunny.tar.gz

2. Open in MeshLab and export as binary STL:
   File → Export Mesh As → bunny.stl  (format: Binary STL)

3. Place at: `<cmake_build_dir>/tests/data/bunny.stl`
   or set environment variable: `BUNNY_STL=<absolute_path_to_bunny.stl>`

### Download (Option B — Python + Open3D)

```python
import open3d as o3d
m = o3d.io.read_triangle_mesh("bun_zipper_res4.ply")
o3d.io.write_triangle_mesh("tests/data/bunny.stl", m)
```

### Expected mesh properties (bun_zipper_res4)

| Property | Value |
|---|---|
| Vertices | 35,947 |
| Faces | 69,451 |
| AABB X | ~0.165 m |
| AABB Y | ~0.130 m |
| AABB Z | ~0.101 m |
| Units | meters (original scanner units) |

### Running the test

```bash
# After placing bunny.stl in the build/tests/data/ directory:
cd build/Release
./integration_tests --gtest_filter=StanfordBunny*

# Or with an absolute path:
BUNNY_STL=/path/to/bunny.stl ./integration_tests --gtest_filter=StanfordBunny*
```

If the file is missing the test outputs `[  SKIPPED ]` automatically.
