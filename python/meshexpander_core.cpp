// ---------------------------------------------------------------------------
// meshexpander_core.cpp — pybind11 Python bindings for MeshExpander
// ---------------------------------------------------------------------------
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "expander/Mesh.hpp"
#include "expander/BoxExpander.hpp"
#include "expander/AssemblyExpander.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

#ifdef MESHEXPANDER_HAS_IO
#include "io/AssimpLoader.hpp"
#endif

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

    // ── BoxExpander ───────────────────────────────────────────────────────────
    py::class_<BoxExpander>(mod, "BoxExpander", R"pbdoc(
        削り出し法コア機能 (Carving Expansion Core).

        Given a box and a mesh, expands the box by d and then carves it
        using local face normals from mesh faces that overlap the box:
          1. expandedBox = box ± d
          2. collectFaces(mesh, box)
          3. merge near-parallel normals
          4. D_i = max(local_vertices . n_i) + d
          5. ClippingEngine::clip(expandedBox, half-spaces) → convex polytope

        When called with only a mesh (convenience overload), the mesh's own
        AABB is used as the box. This is the default algorithm used by
        AssemblyExpander for each part.

        Examples
        --------
        >>> exp = BoxExpander()
        >>> result = exp.expand(mesh, d=0.002)   # mesh AABB as box
    )pbdoc")
        .def(py::init<double>(),
             py::arg("face_normal_merge_deg") = 20.0,
             "Constructor. face_normal_merge_deg: threshold for merging near-parallel normals.")
        .def("expand",
             py::overload_cast<const Mesh&, double>(&BoxExpander::expand),
             py::arg("mesh"), py::arg("d"),
             "Expand mesh using its own AABB as the initial box.")
        .def("expand_box",
             py::overload_cast<const Eigen::AlignedBox3d&, const Mesh&, double>(
                 &BoxExpander::expand, py::const_),
             py::arg("box"), py::arg("mesh"), py::arg("d"),
             "Expand a specific box using nearby face normals from mesh.");

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
        .def_readwrite("face_normal_merge_deg",
            &AssemblyExpander::Options::faceNormalMergeDeg,
            "Angle threshold for merging near-parallel face normals (degrees)");

    py::class_<AssemblyExpander>(mod, "AssemblyExpander", R"pbdoc(
        Conservative expansion for multi-part 3D assemblies.

        All parts are expanded with BoxExpander (削り出し法) regardless of convexity.
        Uses mesh AABB as the initial box, then carves with local face normals.
        Part boundaries come from the file's mesh structure (one mesh = one part).

        Examples
        --------
        >>> exp = AssemblyExpander()
        >>> parts = [mesh_a, mesh_b]
        >>> parts = AssemblyExpander.merge_contained(parts)
        >>> result = exp.expand_merged(parts, d=0.002)
    )pbdoc")
        .def(py::init<>(), "Default options")
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
             "Merge parts whose bounding box is fully contained within another part's.");

    // ── Free functions (assembly) ─────────────────────────────────────────────

    mod.def("merge_contained",
        [](const std::vector<Mesh>& parts, double tol) {
            return AssemblyExpander::mergeContained(parts, tol);
        },
        py::arg("parts"), py::arg("tolerance") = 1e-6,
        "Merge parts whose bounding box is fully contained within another part's.");

    mod.def("expand_assembly",
        [](const std::vector<Mesh>& parts, double d, double faceNormalMergeDeg) {
            AssemblyExpander::Options opts;
            opts.faceNormalMergeDeg = faceNormalMergeDeg;
            return AssemblyExpander(opts).expand(parts, d);
        },
        py::arg("parts"),
        py::arg("d"),
        py::arg("face_normal_merge_deg") = 20.0,
        "Expand each part independently. Returns list[Mesh].");

    mod.def("expand_assembly_merged",
        [](const std::vector<Mesh>& parts, double d, double faceNormalMergeDeg) {
            AssemblyExpander::Options opts;
            opts.faceNormalMergeDeg = faceNormalMergeDeg;
            return AssemblyExpander(opts).expandMerged(parts, d);
        },
        py::arg("parts"),
        py::arg("d"),
        py::arg("face_normal_merge_deg") = 20.0,
        "Expand all parts and merge into one multi-body Mesh.");

    // ── Assembly file I/O (requires MESHEXPANDER_HAS_IO / Assimp) ────────────
#ifdef MESHEXPANDER_HAS_IO
    mod.def("load_assembly",
        [](const std::string& path) {
            expander::io::AssimpLoader loader;
            return loader.load(path);
        },
        py::arg("path"),
        "Load assembly file (DAE, FBX, OBJ, STL, …) via Assimp.\n"
        "Returns list[Mesh] — one Mesh per part in the scene graph.\n"
        "Node transforms are accumulated so all vertices are in world space.\n"
        "Requires package built with MESHEXPANDER_BUILD_IO=ON.");

    mod.attr("HAS_IO") = true;
#else
    mod.attr("HAS_IO") = false;
#endif

    // ── Convenience functions ─────────────────────────────────────────────────
    mod.def("expand_file",
        [](const std::string& input_path, double d,
           const std::string& output_path) -> Mesh {
            Mesh input = StlReader::read(input_path);
            if (input.empty())
                throw std::runtime_error("Failed to read STL: " + input_path);
            BoxExpander exp;
            Mesh result = exp.expand(input, d);
            if (!output_path.empty())
                StlWriter::write(output_path, result);
            return result;
        },
        py::arg("input_path"),
        py::arg("d"),
        py::arg("output_path") = "",
        "Expand an STL file by distance d using BoxExpander (削り出し法). "
        "Optionally write result to output_path.");

    mod.def("expand_np",
        [](DblArray verts, IntArray faces, double d)
           -> std::pair<py::array_t<double>, py::array_t<int32_t>> {
            return mesh_to_arrays(
                BoxExpander().expand(arrays_to_mesh(verts, faces), d));
        },
        py::arg("vertices"),
        py::arg("faces"),
        py::arg("d"),
        "Expand mesh from numpy arrays using BoxExpander (削り出し法). "
        "Returns (vertices ndarray[N2,3], faces ndarray[M2,3]).");
}
