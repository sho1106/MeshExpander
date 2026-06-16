"""
visualize_single.py — 単体メッシュ: 元モデルと膨張後を重ねて表示

使い方:
    python visualize_single.py model.stl
    python visualize_single.py model.stl --d 0.002 --side-by-side
    # 凹形状: 近似凸分解 (concavity 駆動の空間ボックス分割)
    python visualize_single.py model.stl --d 0.002 --max-pieces 8

依存:
    pip install open3d meshexpander
"""

import argparse
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d が見つかりません。  pip install open3d")

try:
    import meshexpander as me
except ImportError:
    sys.exit("meshexpander が見つかりません。ビルド後に  pip install .")


def me_to_o3d(mesh) -> o3d.geometry.TriangleMesh:
    m = o3d.geometry.TriangleMesh()
    m.vertices  = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    m.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces))
    m.compute_vertex_normals()
    return m


def main():
    parser = argparse.ArgumentParser(description="単体メッシュ可視化")
    parser.add_argument("input", help="入力 STL ファイル")
    parser.add_argument("--d", type=float, default=1.0,
                        help="expansion distance (same units as the model)")
    parser.add_argument("--max-pieces", type=int, default=1,
                        help="凹分解の最大ピース数 (1=単一凸, >1 で concavity 駆動分割)")
    parser.add_argument("--concavity-tol", type=float, default=0.0,
                        help="concavity 許容値 [m] (>0 で分割を有効化)")
    parser.add_argument("--side-by-side", action="store_true",
                        help="元メッシュを左にオフセットして横並び表示")
    args = parser.parse_args()

    print(f"読み込み: {args.input}")
    src = me.read_stl(args.input)

    if args.max_pieces > 1 or args.concavity_tol > 0.0:
        print(f"膨張中: d={args.d}, max_pieces={args.max_pieces}, "
              f"concavity_tol={args.concavity_tol} (凹分解) ...")
        opts = me.AssemblyExpanderOptions()
        opts.max_convex_pieces = args.max_pieces
        opts.concavity_tol     = args.concavity_tol
        exp = me.AssemblyExpander(opts).expand([src], args.d)[0]
    else:
        print(f"膨張中: d={args.d} (単一凸 BoxExpander) ...")
        exp = me.BoxExpander().expand(src, args.d)
    print("完了")

    orig_o3d = me_to_o3d(src)
    exp_o3d  = me_to_o3d(exp)
    orig_o3d.paint_uniform_color([0.2, 0.4, 0.9])   # 青: 元
    exp_o3d.paint_uniform_color([0.2, 0.8, 0.3])     # 緑: 膨張後

    wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(exp_o3d)
    wireframe.paint_uniform_color([0.0, 0.5, 0.0])

    if args.side_by_side:
        bbox   = orig_o3d.get_axis_aligned_bounding_box()
        offset = (bbox.get_extent()[0] + args.d * 5) * 1.3
        orig_o3d.translate([-offset, 0, 0])

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(args.d * 10, 0.01))

    print("Q / Esc で終了")
    o3d.visualization.draw_geometries(
        [orig_o3d, exp_o3d, wireframe, axes],
        window_name="MeshExpander — 青:元  緑:膨張後",
        width=1280, height=720, mesh_show_back_face=True,
    )


if __name__ == "__main__":
    main()
