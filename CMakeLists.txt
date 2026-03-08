cmake_minimum_required(VERSION 3.16)
project(MeshExpander VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ---------- Eigen ----------
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

# ---------- Assimp (for future file I/O; not yet wired into core lib) ----------
# NOTE: Assimp is declared here but NOT linked into mesh_expander until
#       the AssetIO layer is implemented. The STL export currently uses
#       the lightweight built-in StlWriter (include/expander/StlWriter.hpp).
# FetchContent_Declare(
#     assimp
#     GIT_REPOSITORY https://github.com/assimp/assimp.git
#     GIT_TAG        v5.3.1
#     GIT_SHALLOW    TRUE
# )
# set(ASSIMP_BUILD_TESTS        OFF CACHE BOOL "" FORCE)
# set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
# set(ASSIMP_INSTALL            OFF CACHE BOOL "" FORCE)
# set(ASSIMP_NO_EXPORT          ON  CACHE BOOL "" FORCE)
# set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(assimp)

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

# ---------- Library ----------
add_library(mesh_expander STATIC
    src/ConservativeExpander.cpp
    src/ClippingEngine.cpp
    src/VoxelGrid.cpp
    src/RobustSlicer.cpp
)
target_include_directories(mesh_expander PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mesh_expander PUBLIC
    Eigen3::Eigen
)
if(MSVC)
    # /utf-8: ソースとリアルアウトをUTF-8として扱う (CP932誤認識を防ぐ)
    add_compile_options(/utf-8)
    target_compile_options(mesh_expander PRIVATE /W4)
else()
    target_compile_options(mesh_expander PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------- Tests ----------
enable_testing()

add_executable(unit_tests
    tests/unit/test_conservative_expander.cpp
    tests/unit/test_math_utils.cpp
    tests/unit/test_clipping_engine.cpp
    tests/unit/test_voxel_grid.cpp
    tests/unit/test_robust_slicer.cpp
)
target_link_libraries(unit_tests PRIVATE
    mesh_expander
    GTest::gtest_main
)

add_executable(integration_tests
    tests/integration/test_shape_expansion.cpp
    tests/integration/test_concave_expansion.cpp
    tests/integration/test_stanford_bunny.cpp
    tests/integration/test_cad_shapes.cpp
)
target_link_libraries(integration_tests PRIVATE
    mesh_expander
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
gtest_discover_tests(integration_tests)

# Custom target that runs only MeshExpander tests (excludes Eigen's own test suite)
add_custom_target(check
    COMMAND $<TARGET_FILE:unit_tests>
    COMMAND $<TARGET_FILE:integration_tests>
    DEPENDS unit_tests integration_tests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running MeshExpander tests"
)
