cmake_minimum_required(VERSION 3.16)
project(MeshExpanderVisualize CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- MeshExpander ----------
# Option A: ビルド済みツリーを直接参照（ワークスペースからビルドした場合）
#   cmake -DMeshExpander_SOURCE=<repo>/projects/MeshExpander \
#         -DMeshExpander_BUILD=<repo>/projects/MeshExpander/build ...
set(MeshExpander_SOURCE "" CACHE PATH "MeshExpander source root (contains include/)")
set(MeshExpander_BUILD  "" CACHE PATH "MeshExpander build directory (contains mesh_expander.lib/.a)")

if(NOT MeshExpander_SOURCE OR NOT MeshExpander_BUILD)
    message(FATAL_ERROR
        "MeshExpander のパスを指定してください:\n"
        "  -DMeshExpander_SOURCE=<repo>/projects/MeshExpander\n"
        "  -DMeshExpander_BUILD=<repo>/projects/MeshExpander/build"
    )
endif()

# Eigen (MeshExpander が使う)
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    # MeshExpander のビルドツリーに展開されている Eigen を流用
    set(eigen_src "${MeshExpander_BUILD}/_deps/eigen-src")
    if(EXISTS "${eigen_src}")
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${eigen_src}")
    else()
        message(FATAL_ERROR "Eigen3 が見つかりません。find_package(Eigen3) または MeshExpander のビルドツリーを確認してください。")
    endif()
endif()

# mesh_expander ライブラリ (ビルド済み .lib/.a を直接リンク)
add_library(mesh_expander STATIC IMPORTED)
if(WIN32)
    set(_lib_path "${MeshExpander_BUILD}/Release/mesh_expander.lib")
    if(NOT EXISTS "${_lib_path}")
        set(_lib_path "${MeshExpander_BUILD}/mesh_expander.lib")
    endif()
else()
    set(_lib_path "${MeshExpander_BUILD}/libmesh_expander.a")
endif()
set_target_properties(mesh_expander PROPERTIES
    IMPORTED_LOCATION "${_lib_path}"
    INTERFACE_INCLUDE_DIRECTORIES "${MeshExpander_SOURCE}/include"
    INTERFACE_LINK_LIBRARIES Eigen3::Eigen
)

# ---------- Open3D ----------
# インストール済みの Open3D を検索
# Windows:  pip install open3d でインストールされた Open3D の cmake ディレクトリを渡す
#   cmake -DOpen3D_DIR="C:/ProgramData/miniforge3/Lib/site-packages/open3d/cmake" ...
# Linux:    cmake -DOpen3D_DIR=/usr/local/lib/cmake/Open3D ...
find_package(Open3D REQUIRED)

# ---------- 実行ファイル ----------
add_executable(visualize visualize.cpp)

target_link_libraries(visualize PRIVATE
    mesh_expander
    Open3D::Open3D
)

if(MSVC)
    target_compile_options(visualize PRIVATE /utf-8 /W3)
endif()
