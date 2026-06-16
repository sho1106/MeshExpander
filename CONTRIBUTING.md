# Contributing to MeshExpander

Thanks for your interest in improving MeshExpander! This is a small, focused
library (conservative 3D mesh expansion for machining clearance models), so the
contribution process is lightweight.

## Ways to contribute

- **Report a bug** — open a GitHub issue with a minimal repro (ideally a small
  binary STL or a NumPy snippet) and the expected vs actual result.
- **Request a feature** — open an issue describing the machining/geometry use
  case. Because the core guarantee is *conservative containment*, please note
  whether the request preserves that property.
- **Submit a PR** — see below.

## Development setup

```bash
git clone https://github.com/sho1106/MeshExpander.git
cd MeshExpander
cmake -S . -B build
cmake --build build --config Release --target check   # builds + runs all tests
```

For the Python bindings:

```bash
pip install scikit-build-core pybind11
pip install -e .
pytest tests/python
```

The optional STEP/OBJ/FBX I/O layer is built with `-DMESHEXPANDER_BUILD_IO=ON`
(pulls in Assimp via FetchContent).

## Pull request checklist

- [ ] All tests pass: `cmake --build build --config Release --target check`
- [ ] New behavior is covered by a unit or integration test under `tests/`
- [ ] The **conservativeness guarantee** is preserved (output always contains
      input + `d`); if a change affects accuracy, update the benchmark tables
- [ ] Public API changes are reflected in both the C++ headers and the Python
      `.pyi` stub (`python/meshexpander/meshexpander.pyi`)
- [ ] Docs updated (`README.md` and `docs/README_JA.md` are kept in sync)

## Code style

- C++17, Eigen for linear algebra. Match the surrounding style; MSVC `/W4` and
  GCC/Clang `-Wall -Wextra -Wpedantic` must stay clean.
- Keep the core library dependency-light (Eigen only). Heavy or optional
  dependencies belong behind a CMake `option()` (as the IO layer is).

## Tests

New shapes go in `tests/integration/` and are registered in `CMakeLists.txt`.
Accuracy is measured by **Cov%** (fraction of input vertices contained) and
**Exp%** (fraction of face-normal `d`-probes contained); both should stay at
their documented targets.

## License

By contributing you agree that your contributions are licensed under the
project's [MIT License](LICENSE).
