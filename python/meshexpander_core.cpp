// ---------------------------------------------------------------------------
// meshexpander_core.cpp — pybind11 Python bindings for MeshExpander
// ---------------------------------------------------------------------------
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "expander/Mesh.hpp"
#include "expander/RobustSlicer.hpp"
#include "expander/ConservativeExpander.hpp"
#include "expander/AssemblyExpander.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

namespace py = pybind11;
using namespace expander;

// MSVC workaround: complex template args with '|' inside lambdas cause C2059.
// Typedef them here so lambda parameters use simple names.
using DblArray = py::array_t<double,  (int)(py::array::c_style | py::array::forcecast)>;
using IntArray = py::array_t<int32_t, (int)(py::array::c_style | py::array::forcecast)>;

// ── helpers ──────────────────────────────────────────────────────────────────

static std::pair<py::array_t<double>, py::array_t<int32_t>>
mesh_to_arrays(const Mesh& m) {
    py::array_t<double>  verts({(size_t)m.numVertices(), (size_t)3});
    py::array_t<int32_t> faces({(size_t)m.numFaces(),    (size_t)3});
    {
        auto v = verts.mutable_unchecked<2>();
        for (int i = 0; i < m.numVertices(); ++i) {
            v(i,0) = m.vertices(i,0);
            v(i,1) = m.vertices(i,1);
            v(i,2) = m.vertices(i,2);
        }
    }
    {
        auto f = faces.mutable_unchecked<2>();
        for (int i = 0; i < m.numFaces(); ++i) {
            f(i,0) = m.faces[i][0];
            f(i,1) = m.faces[i][1];
            f(i,2) = m.faces[i][2];
        }
    }
    return {verts, faces};
}

static Mesh arrays_to_mesh(DblArray verts, IntArray faces) {
    if (verts.ndim() != 2 || verts.shape(1) != 3)
        throw std::runtime_error("vertices must be shape (N, 3)");
    if (faces.ndim() != 2 || faces.shape(1) != 3)
        throw std::runtime_error("faces must be shape (M, 3)");

    Mesh m;
    m.vertices.resize((int)verts.shape(0), 3);
    {
        auto v = verts.unchecked<2>();
        for (int i = 0; i < (int)verts.shape(0); ++i) {
            m.vertices(i,0) = v(i,0);
            m.vertices(i,1) = v(i,1);
            m.vertices(i,2) = v(i,2);
        }
    }
    {
        auto f = faces.unchecked<2>();
        m.faces.reserve((size_t)faces.shape(0));
        for (int i = 0; i < (int)faces.shape(0); ++i)
            m.faces.push_back({(int)f(i,0), (int)f(i,1), (int)f(i,2)});
    }
    return m;
}

// ── module ───────────────────────────────────────────────────────────────────

PYBIND11_MODULE(meshexpander_core, mod) {
    mod.doc() = "MeshExpander — conservative 3D mesh expansion";

    // ── Mesh ──────────────────────────────────────────────────────────────────
    py::class_<Mesh>(mod, "Mesh",
        "Triangulated 3D mesh: Nx3 vertices + Mx3 face indices.")
        .def(py::init<>())
        .def("num_vertices", &Mesh::numVertices, "Number of vertices")
        .def("num_faces",    &Mesh::numFaces,    "Number of triangles")
        .def("empty",        &Mesh::empty,       "True if the mesh has no vertices")

        // vertices property: Mesh.vertices → ndarray[N,3]
        .def_property("vertices",
            [](const Mesh& m) -> py::array_t<double> {
                py::array_t<double> a({(size_t)m.numVertices(), (size_t)3});
                auto v = a.mutable_unchecked<2>();
                for (int i = 0; i < m.numVertices(); ++i)
                    for (int j = 0; j < 3; ++j) v(i,j) = m.vertices(i,j);
                return a;
            },
            [](Mesh& m, DblArray a) {
                if (a.ndim() != 2 || a.shape(1) != 3)
                    throw std::runtime_error("vertices must be shape (N, 3)");
                auto v = a.unchecked<2>();
                m.vertices.resize((int)a.shape(0), 3);
                for (int i = 0; i < (int)a.shape(0); ++i)
                    for (int j = 0; j < 3; ++j) m.vertices(i,j) = v(i,j);
            },
            "Vertex positions as ndarray[N, 3] float64")

        // faces property: Mesh.faces → ndarray[M,3]
        .def_property("faces",
            [](const Mesh& m) -> py::array_t<int32_t> {
                py::array_t<int32_t> a({(size_t)m.numFaces(), (size_t)3});
                auto f = a.mutable_unchecked<2>();
                for (int i = 0; i < m.numFaces(); ++i)
                    for (int j = 0; j < 3; ++j) f(i,j) = m.faces[i][j];
                return a;
            },
            [](Mesh& m, IntArray a) {
                if (a.ndim() != 2 || a.shape(1) != 3)
                    throw std::runtime_error("faces must be shape (M, 3)");
                auto f = a.unchecked<2>();
                m.faces.clear();
                m.faces.reserve((size_t)a.shape(0));
                for (int i = 0; i < (int)a.shape(0); ++i)
                    m.faces.push_back({(int)f(i,0), (int)f(i,1), (int)f(i,2)});
            },
            "Triangle indices as ndarray[M, 3] int32")

        // to_arrays: returns (verts, faces) tuple
        .def("to_arrays", &mesh_to_arrays,
             "Returns (vertices: ndarray[N,3], faces: ndarray[M,3])")

        // from_arrays: static factory
        .def_static("from_arrays", &arrays_to_mesh,
                    py::arg("vertices"), py::arg("faces"),
                    "Create Mesh from numpy arrays")

        .def("__repr__", [](const Mesh& m) {
            return "<Mesh vertices=" + std::to_string(m.numVertices()) +
                   " faces=" + std::to_string(m.numFaces()) + ">";
        });

    // ── RobustSlicer ──────────────────────────────────────────────────────────
    py::class_<RobustSlicer>(mod, "RobustSlicer", R"pbdoc(
        Conservative expansion for concave (non-convex) meshes.

        Uses solid voxelization + top-down box partitioning + per-box
        local half-space expansion to guarantee every surface point is
        covered by margin d.

        Examples
        --------
        >>> slicer = RobustSlicer.with_cell_size(0.005)   # 5 mm cells
        >>> result = slicer.expand_merged(mesh, d=0.001)   # expand by 1 mm
    )pbdoc")
        .def(py::init<int, double>(),
             py::arg("resolution") = 64,
             py::arg("face_normal_merge_deg") = 20.0,
             "Adaptive-resolution constructor (cell = aabb.maxDim / resolution)")
        .def_static("with_cell_size", &RobustSlicer::withCellSize,
                    py::arg("cell_size"),
                    py::arg("face_normal_merge_deg") = 20.0,
                    "Fixed world-space voxel cell size constructor")
        .def("expand_multi", &RobustSlicer::expandMulti,
             py::arg("mesh"), py::arg("d"),
             "Expand into a list of closed convex polytopes (one per box)")
        .def("expand_merged", &RobustSlicer::expandMerged,
             py::arg("mesh"), py::arg("d"),
             "Expand and merge all polytopes into one STL-ready mesh")
        .def("expand", &RobustSlicer::expand,
             py::arg("mesh"), py::arg("d"),
             "Expand (IExpander compat — returns first polytope)");

    // ── ConservativeExpander ──────────────────────────────────────────────────
    py::class_<ConservativeExpander>(mod, "ConservativeExpander", R"pbdoc(
        Conservative expansion for convex meshes via half-space intersection.
    )pbdoc")
        .def(py::init<>())
        .def("expand", &ConservativeExpander::expand,
             py::arg("mesh"), py::arg("d"),
             "Expand a convex mesh by distance d");

    // ── STL I/O ───────────────────────────────────────────────────────────────
    mod.def("read_stl",  &StlReader::read,
            py::arg("path"),
            "Read binary STL file, return Mesh.");
    mod.def("write_stl", &StlWriter::write,
            py::arg("path"), py::arg("mesh"),
            py::arg("header") = "MeshExpander output",
            "Write Mesh to binary STL file.");

    // ── AssemblyExpander ──────────────────────────────────────────────────────

    py::class_<AssemblyExpander::Options>(mod, "AssemblyExpanderOptions",
        "Options controlling AssemblyExpander behaviour.")
        .def(py::init<>())
        .def_readwrite("resolution", &AssemblyExpander::Options::resolution,
            "Voxel resolution for concave parts (cell = aabb.maxDim / resolution)")
        .def_readwrite("cell_size_world", &AssemblyExpander::Options::cellSizeWorld,
            "Fixed voxel cell size in world units (overrides resolution when > 0)")
        .def_readwrite("face_normal_merge_deg",
            &AssemblyExpander::Options::faceNormalMergeDeg,
            "Angle threshold for merging near-parallel face normals (degrees)")
        .def_readwrite("convex_tol", &AssemblyExpander::Options::convexTol,
            "Tolerance for isConvex() test (world units)");

    py::class_<AssemblyExpander>(mod, "AssemblyExpander", R"pbdoc(
        Conservative expansion for multi-part 3D assemblies.

        Each part is expanded independently using the optimal algorithm:
        - Convex parts  → ConservativeExpander (single polytope, low polygon count)
        - Concave parts → RobustSlicer         (voxel-based, concavity-aware)

        Examples
        --------
        >>> exp = AssemblyExpander()
        >>> parts = [mesh_a, mesh_b]
        >>> parts = AssemblyExpander.merge_contained(parts)
        >>> result = exp.expand_merged(parts, d=0.002)
    )pbdoc")
        .def(py::init<>(), "Default options (resolution=64)")
        .def(py::init<AssemblyExpander::Options>(), py::arg("options"),
             "Construct with explicit options")
        .def("expand", &AssemblyExpander::expand,
             py::arg("parts"), py::arg("d"),
             "Expand each part independently. Returns one Mesh per input part.")
        .def("expand_merged", &AssemblyExpander::expandMerged,
             py::arg("parts"), py::arg("d"),
             "Expand all parts and concatenate into a single multi-body mesh.")
        .def_static("merge_contained", &AssemblyExpander::mergeContained,
             py::arg("parts"), py::arg("tolerance") = 1e-6,
             "Merge parts whose bounding box is fully contained within another part's.")
        .def_static("is_convex", &AssemblyExpander::isConvex,
             py::arg("mesh"), py::arg("tol") = 1e-6,
             "Return True if the mesh is approximately convex.");

    // ── Free functions (assembly) ─────────────────────────────────────────────

    mod.def("is_convex",
        [](const Mesh& mesh, double tol) {
            return AssemblyExpander::isConvex(mesh, tol);
        },
        py::arg("mesh"), py::arg("tol") = 1e-6,
        "Return True if mesh is approximately convex.");

    mod.def("merge_contained",
        [](const std::vector<Mesh>& parts, double tol) {
            return AssemblyExpander::mergeContained(parts, tol);
        },
        py::arg("parts"), py::arg("tolerance") = 1e-6,
        "Merge parts whose bounding box is fully contained within another part's.");

    mod.def("expand_assembly",
        [](const std::vector<Mesh>& parts, double d,
           int resolution, double cellSizeWorld, double faceNormalMergeDeg) {
            AssemblyExpander::Options opts;
            opts.resolution          = resolution;
            opts.cellSizeWorld       = cellSizeWorld;
            opts.faceNormalMergeDeg  = faceNormalMergeDeg;
            return AssemblyExpander(opts).expand(parts, d);
        },
        py::arg("parts"),
        py::arg("d"),
        py::arg("resolution")           = 64,
        py::arg("cell_size_world")      = 0.0,
        py::arg("face_normal_merge_deg") = 20.0,
        "Expand each part independently. Returns list[Mesh].");

    mod.def("expand_assembly_merged",
        [](const std::vector<Mesh>& parts, double d,
           int resolution, double cellSizeWorld, double faceNormalMergeDeg) {
            AssemblyExpander::Options opts;
            opts.resolution          = resolution;
            opts.cellSizeWorld       = cellSizeWorld;
            opts.faceNormalMergeDeg  = faceNormalMergeDeg;
            return AssemblyExpander(opts).expandMerged(parts, d);
        },
        py::arg("parts"),
        py::arg("d"),
        py::arg("resolution")           = 64,
        py::arg("cell_size_world")      = 0.0,
        py::arg("face_normal_merge_deg") = 20.0,
        "Expand all parts and merge into one multi-body Mesh.");

    // ── Convenience functions ─────────────────────────────────────────────────
    mod.def("expand_file",
        [](const std::string& input_path, double d, double cell_size,
           const std::string& output_path) -> Mesh {
            Mesh input = StlReader::read(input_path);
            if (input.empty())
                throw std::runtime_error("Failed to read STL: " + input_path);
            auto slicer = RobustSlicer::withCellSize(cell_size);
            Mesh result = slicer.expandMerged(input, d);
            if (!output_path.empty())
                StlWriter::write(output_path, result);
            return result;
        },
        py::arg("input_path"),
        py::arg("d"),
        py::arg("cell_size")   = 0.005,
        py::arg("output_path") = "",
        "Expand an STL file by distance d. Optionally write result to output_path.");

    mod.def("expand_np",
        [](DblArray verts, IntArray faces, double d, double cell_size)
           -> std::pair<py::array_t<double>, py::array_t<int32_t>> {
            return mesh_to_arrays(
                RobustSlicer::withCellSize(cell_size)
                    .expandMerged(arrays_to_mesh(verts, faces), d));
        },
        py::arg("vertices"),
        py::arg("faces"),
        py::arg("d"),
        py::arg("cell_size") = 0.005,
        "Expand mesh from numpy arrays. Returns (vertices ndarray[N2,3], faces ndarray[M2,3]).");
}
