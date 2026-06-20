cmake_minimum_required(VERSION 3.16)
project(MeshExpander VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---------- Eigen ----------
# Prefer a system/installed Eigen3 (gives an IMPORTED target that survives
# install/export). Fall back to FetchContent for a zero-setup dev build.
find_package(Eigen3 3.4 QUIET)
if(NOT TARGET Eigen3::Eigen)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE
    )
    # Suppress Eigen's own test suite from registering with CTest
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING      OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    set(BUILD_TESTING      ON  CACHE BOOL "" FORCE)
endif()

# NOTE: Assimp (for STEP/OBJ/FBX I/O) is fetched only when MESHEXPANDER_BUILD_IO=ON;
#       see the "IO Library" section below. The core links Eigen only.

# ---------- Google Test ----------
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---------- Core Library ----------
add_library(mesh_expander STATIC
    src/BoxExpander.cpp
    src/ClippingEngine.cpp
    src/ProgressiveHull.cpp
    src/BoxPartitioner.cpp
    src/AssemblyExpander.cpp
)
target_include_directories(mesh_expander PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Eigen is a build-time requirement; wrapped in BUILD_INTERFACE so the exported
# target does not hard-reference the Eigen target (which may be FetchContent's,
# not installable). MeshExpanderConfig re-attaches Eigen3::Eigen via find_dependency.
target_link_libraries(mesh_expander PUBLIC
    $<BUILD_INTERFACE:Eigen3::Eigen>
)
if(MSVC)
    # /utf-8: ソースとリテラルをUTF-8として扱う (CP932誤認識を防ぐ)
    add_compile_options(/utf-8)
    target_compile_options(mesh_expander PRIVATE /utf-8 /W4)
else()
    target_compile_options(mesh_expander PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Position-independent code so the static core can be linked into the pybind11
# shared module on Linux (otherwise: "ld: final link failed: bad value").
set_target_properties(mesh_expander PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------- Install / export (find_package(MeshExpander)) ----------
option(MESHEXPANDER_INSTALL "Generate install rules and a CMake package config" ON)
if(MESHEXPANDER_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(TARGETS mesh_expander
        EXPORT  MeshExpanderTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    set(MESHEXPANDER_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/MeshExpander)
    install(EXPORT MeshExpanderTargets
        FILE      MeshExpanderTargets.cmake
        NAMESPACE MeshExpander::
        DESTINATION ${MESHEXPANDER_CMAKEDIR}
    )

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/MeshExpanderConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/MeshExpanderConfig.cmake
        INSTALL_DESTINATION ${MESHEXPANDER_CMAKEDIR}
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/MeshExpanderConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/MeshExpanderConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/MeshExpanderConfigVersion.cmake
        DESTINATION ${MESHEXPANDER_CMAKEDIR}
    )
endif()

# ---------- IO Library (optional, requires Assimp) ----------
# Provides AssimpLoader / AssimpExporter implementing IModelLoader / IModelExporter.
# Replaceable with any other loader/exporter by implementing the same interfaces.
#
#   cmake -DMESHEXPANDER_BUILD_IO=ON ...
option(MESHEXPANDER_BUILD_IO "Build Assimp-based IO layer (AssimpLoader/AssimpExporter)" OFF)
if(MESHEXPANDER_BUILD_IO)
    FetchContent_Declare(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG        v5.3.1
        GIT_SHALLOW    TRUE
    )
    set(ASSIMP_BUILD_TESTS        OFF CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
    set(ASSIMP_INSTALL            OFF CACHE BOOL "" FORCE)
    set(ASSIMP_NO_EXPORT          OFF CACHE BOOL "" FORCE)
    set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(assimp)

    add_library(mesh_expander_io STATIC
        src/io/AssimpLoader.cpp
        src/io/AssimpExporter.cpp
    )
    target_include_directories(mesh_expander_io PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(mesh_expander_io PUBLIC mesh_expander assimp)
    if(MSVC)
        target_compile_options(mesh_expander_io PRIVATE /utf-8 /W4)
    else()
        target_compile_options(mesh_expander_io PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

# ---------- CLI tool (requires IO layer) ----------
option(MESHEXPANDER_BUILD_CLI "Build meshexpander_cli (requires MESHEXPANDER_BUILD_IO=ON)" OFF)
if(MESHEXPANDER_BUILD_CLI)
    if(NOT MESHEXPANDER_BUILD_IO)
        message(FATAL_ERROR "MESHEXPANDER_BUILD_CLI requires MESHEXPANDER_BUILD_IO=ON")
    endif()
    add_executable(meshexpander_cli src/cli/main.cpp)
    target_link_libraries(meshexpander_cli PRIVATE mesh_expander_io)
    target_include_directories(meshexpander_cli PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    if(MSVC)
        target_compile_options(meshexpander_cli PRIVATE /utf-8 /W4)
    else()
        target_compile_options(meshexpander_cli PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endif()

# ---------- Tests ----------
enable_testing()

add_executable(unit_tests
    tests/unit/test_box_expander.cpp
    tests/unit/test_math_utils.cpp
    tests/unit/test_clipping_engine.cpp
    tests/unit/test_progressive_hull.cpp
    tests/unit/test_box_partitioner.cpp
    tests/unit/test_assembly_expander.cpp
    tests/unit/test_stl_reader.cpp
    tests/unit/test_robustness.cpp
)
target_link_libraries(unit_tests PRIVATE
    mesh_expander
    GTest::gtest_main
)

add_executable(integration_tests
    tests/integration/test_shape_expansion.cpp
    tests/integration/test_cad_shapes.cpp
    tests/integration/test_assembly_expansion.cpp
    tests/integration/test_complex_assembly.cpp
    tests/integration/test_complex_shapes.cpp
    tests/integration/test_concave_expansion.cpp
    tests/integration/test_scale_robustness.cpp
)
target_link_libraries(integration_tests PRIVATE
    mesh_expander
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
gtest_discover_tests(integration_tests)

# ---------- IO layer tests (optional, requires Assimp) ----------
if(MESHEXPANDER_BUILD_IO)
    add_executable(io_tests
        tests/io/test_assimp_io.cpp
    )
    target_link_libraries(io_tests PRIVATE
        mesh_expander_io
        GTest::gtest_main
    )
    target_include_directories(io_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    if(MSVC)
        target_compile_options(io_tests PRIVATE /utf-8 /W4)
    else()
        target_compile_options(io_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    gtest_discover_tests(io_tests)
endif()

# Custom target that runs only MeshExpander tests (excludes Eigen's own test suite)
if(MESHEXPANDER_BUILD_IO)
    add_custom_target(check
        COMMAND $<TARGET_FILE:unit_tests>
        COMMAND $<TARGET_FILE:integration_tests>
        COMMAND $<TARGET_FILE:io_tests>
        DEPENDS unit_tests integration_tests io_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running MeshExpander tests (core + IO)"
    )
else()
    add_custom_target(check
        COMMAND $<TARGET_FILE:unit_tests>
        COMMAND $<TARGET_FILE:integration_tests>
        DEPENDS unit_tests integration_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running MeshExpander tests"
    )
endif()

# ---------- Python bindings (optional, requires pybind11) ----------
option(MESHEXPANDER_BUILD_PYTHON "Build Python bindings via pybind11" OFF)
if(MESHEXPANDER_BUILD_PYTHON)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(meshexpander_core
        python/meshexpander_core.cpp
    )
    target_link_libraries(meshexpander_core PRIVATE mesh_expander)
    target_include_directories(meshexpander_core PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    if(MSVC)
        target_compile_options(meshexpander_core PRIVATE /utf-8 /W4)
    endif()

    # When both IO and Python are enabled, expose load_assembly() via Assimp
    if(MESHEXPANDER_BUILD_IO)
        target_link_libraries(meshexpander_core PRIVATE mesh_expander_io)
        target_compile_definitions(meshexpander_core PRIVATE MESHEXPANDER_HAS_IO)
    endif()

    if(SKBUILD)
        # Building a wheel: install the extension into the meshexpander package
        # (the pure-Python files come from [tool.scikit-build] wheel.packages).
        install(TARGETS meshexpander_core DESTINATION meshexpander)
    else()
        # Local in-tree development: copy the module next to __init__.py so
        # `PYTHONPATH=python` imports work without installing.
        add_custom_command(TARGET meshexpander_core POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:meshexpander_core>
                ${CMAKE_CURRENT_SOURCE_DIR}/python/meshexpander/
            COMMENT "Copying meshexpander_core to python/meshexpander/"
        )
    endif()
endif()
