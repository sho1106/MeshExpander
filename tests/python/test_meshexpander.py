"""
Python-level integration tests for the meshexpander package.

Run with:
    pip install -e .   (or build the wheel first)
    pytest tests/python/ -v
"""

import math
import tempfile
import os
import pytest
import numpy as np

import meshexpander as me


# ── Geometry helpers ──────────────────────────────────────────────────────────

def make_cube_arrays(half: float, offset=(0.0, 0.0, 0.0)):
    ox, oy, oz = offset
    verts = np.array([
        [-half+ox, -half+oy, -half+oz],
        [ half+ox, -half+oy, -half+oz],
        [ half+ox,  half+oy, -half+oz],
        [-half+ox,  half+oy, -half+oz],
        [-half+ox, -half+oy,  half+oz],
        [ half+ox, -half+oy,  half+oz],
        [ half+ox,  half+oy,  half+oz],
        [-half+ox,  half+oy,  half+oz],
    ], dtype=np.float64)
    faces = np.array([
        [0,2,1],[0,3,2], [4,5,6],[4,6,7],
        [0,1,5],[0,5,4], [2,3,7],[2,7,6],
        [1,2,6],[1,6,5], [0,4,7],[0,7,3],
    ], dtype=np.int32)
    return verts, faces


def make_cube_mesh(half: float, offset=(0.0, 0.0, 0.0)) -> me.Mesh:
    v, f = make_cube_arrays(half, offset)
    return me.Mesh.from_arrays(v, f)


# ── Mesh class ────────────────────────────────────────────────────────────────

class TestMesh:
    def test_from_arrays_round_trip(self):
        v, f = make_cube_arrays(1.0)
        mesh = me.Mesh.from_arrays(v, f)
        assert mesh.num_vertices() == 8
        assert mesh.num_faces() == 12
        assert not mesh.empty()

    def test_to_arrays(self):
        v, f = make_cube_arrays(2.0)
        mesh = me.Mesh.from_arrays(v, f)
        v2, f2 = mesh.to_arrays()
        assert v2.shape == (8, 3)
        assert f2.shape == (12, 3)
        np.testing.assert_allclose(v2, v, atol=1e-12)

    def test_empty_mesh(self):
        mesh = me.Mesh()
        assert mesh.empty()
        assert mesh.num_vertices() == 0
        assert mesh.num_faces() == 0

    def test_repr(self):
        mesh = make_cube_mesh(1.0)
        r = repr(mesh)
        assert "vertices=8" in r
        assert "faces=12" in r


# ── STL I/O ───────────────────────────────────────────────────────────────────

class TestStlIO:
    def test_write_read_round_trip(self):
        mesh = make_cube_mesh(3.0)
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
            path = f.name
        try:
            me.write_stl(path, mesh)
            loaded = me.read_stl(path)
            assert not loaded.empty()
            assert loaded.num_faces() == 12
        finally:
            os.unlink(path)

    def test_bounding_box_preserved(self):
        v, f = make_cube_arrays(5.0)
        mesh = me.Mesh.from_arrays(v, f)
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            path = tmp.name
        try:
            me.write_stl(path, mesh)
            loaded = me.read_stl(path)
            v2, _ = loaded.to_arrays()
            np.testing.assert_allclose(v2.min(axis=0), [-5, -5, -5], atol=1e-4)
            np.testing.assert_allclose(v2.max(axis=0), [ 5,  5,  5], atol=1e-4)
        finally:
            os.unlink(path)


# ── BoxExpander ───────────────────────────────────────────────────────────────

class TestBoxExpander:
    def test_cube_expands_conservatively(self):
        cube = make_cube_mesh(2.0)
        exp = me.BoxExpander()
        result = exp.expand(cube, d=0.1)
        assert not result.empty()
        v, _ = result.to_arrays()
        assert v.min() <= -2.0
        assert v.max() >= 2.0

    def test_expansion_is_larger_than_source(self):
        cube = make_cube_mesh(1.0)
        exp = me.BoxExpander()
        result = exp.expand(cube, d=0.5)
        v, _ = result.to_arrays()
        assert v.max() > 1.0


# ── merge_contained ───────────────────────────────────────────────────────────

class TestMergeContained:
    def test_nested_cubes_merged(self):
        outer = make_cube_mesh(5.0)
        inner = make_cube_mesh(1.0)   # fully inside outer
        merged = me.merge_contained([outer, inner])
        assert len(merged) == 1, "inner should be absorbed into outer"

    def test_separate_cubes_not_merged(self):
        a = make_cube_mesh(2.0, offset=(-10, 0, 0))
        b = make_cube_mesh(2.0, offset=( 10, 0, 0))
        result = me.merge_contained([a, b])
        assert len(result) == 2, "non-overlapping parts should not be merged"

    def test_static_method_same_result(self):
        outer = make_cube_mesh(5.0)
        inner = make_cube_mesh(1.0)
        r1 = me.merge_contained([outer, inner])
        r2 = me.AssemblyExpander.merge_contained([outer, inner])
        assert len(r1) == len(r2)


# ── AssemblyExpander ──────────────────────────────────────────────────────────

class TestAssemblyExpander:
    def test_expand_returns_one_per_part(self):
        a = make_cube_mesh(2.0, (-8, 0, 0))
        b = make_cube_mesh(2.0, ( 8, 0, 0))
        exp = me.AssemblyExpander()
        result = exp.expand([a, b], d=0.1)
        assert len(result) == 2

    def test_each_part_vertex_covered(self):
        d = 0.5
        a = make_cube_mesh(3.0, (-8, 0, 0))
        b = make_cube_mesh(2.0, ( 8, 0, 0))
        exp = me.AssemblyExpander()
        expanded = exp.expand([a, b], d=d)

        for src, out in zip([a, b], expanded):
            sv, _ = src.to_arrays()
            ov, _ = out.to_arrays()
            lo = ov.min(axis=0)
            hi = ov.max(axis=0)
            assert np.all(sv >= lo - 1e-5), "source vertex outside expanded bounding box"
            assert np.all(sv <= hi + 1e-5), "source vertex outside expanded bounding box"

    def test_expand_merged_valid_mesh(self):
        a = make_cube_mesh(2.0, (-6, 0, 0))
        b = make_cube_mesh(2.0, ( 6, 0, 0))
        exp = me.AssemblyExpander()
        result = exp.expand_merged([a, b], d=0.2)
        assert not result.empty()
        v, f = result.to_arrays()
        assert np.all(np.isfinite(v))
        assert f.min() >= 0
        assert f.max() < result.num_vertices()

    def test_options(self):
        opts = me.AssemblyExpanderOptions()
        opts.face_normal_merge_deg = 15.0
        exp = me.AssemblyExpander(opts)
        cube = make_cube_mesh(2.0)
        result = exp.expand_merged([cube], d=0.1)
        assert not result.empty()

    def test_per_part_tighter_than_single_mesh(self):
        """Per-part expansion volume < single-mesh expansion for far-apart parts."""
        d = 0.5
        a = make_cube_mesh(2.0, (-10, 0, 0))
        b = make_cube_mesh(2.0, ( 10, 0, 0))
        exp = me.AssemblyExpander()

        # Single-mesh bounding box volume
        single = exp.expand_merged([a, b], d=d)
        sv, _ = single.to_arrays()
        single_vol = float(np.prod(sv.max(axis=0) - sv.min(axis=0)))

        # Per-part sum of bounding box volumes
        per_part = exp.expand([a, b], d=d)
        per_part_vol = sum(
            float(np.prod(m.to_arrays()[0].max(axis=0) - m.to_arrays()[0].min(axis=0)))
            for m in per_part
        )

        assert per_part_vol < single_vol, (
            f"Per-part vol ({per_part_vol:.2f}) should be < single-mesh vol ({single_vol:.2f})"
        )


# ── Free function wrappers ─────────────────────────────────────────────────────

class TestFreeFunctions:
    def test_expand_assembly_returns_list(self):
        a = make_cube_mesh(2.0, (-5, 0, 0))
        b = make_cube_mesh(2.0, ( 5, 0, 0))
        result = me.expand_assembly([a, b], d=0.1)
        assert isinstance(result, list)
        assert len(result) == 2

    def test_expand_assembly_merged_returns_mesh(self):
        a = make_cube_mesh(2.0, (-5, 0, 0))
        b = make_cube_mesh(2.0, ( 5, 0, 0))
        result = me.expand_assembly_merged([a, b], d=0.1)
        assert isinstance(result, me.Mesh)
        assert not result.empty()

    def test_expand_np(self):
        v, f = make_cube_arrays(2.0)
        out_v, out_f = me.expand_np(v, f, d=0.1)
        assert out_v.shape[1] == 3
        assert out_f.shape[1] == 3
        assert out_v.shape[0] > 0

    def test_expand_file_stl(self):
        mesh = make_cube_mesh(2.0)
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            inp = tmp.name
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            out = tmp.name
        try:
            me.write_stl(inp, mesh)
            result = me.expand_file(inp, d=0.1, output_path=out)
            assert not result.empty()
            assert os.path.getsize(out) > 0
        finally:
            os.unlink(inp)
            os.unlink(out)
