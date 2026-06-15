"""
visualize_assembly.py — マルチパートアセンブリ可視化

各パーツを独立に膨張させてカラーコードで並べて表示する。

使い方:
    # デモ (引数なし: 合成アセンブリを自動生成)
    python visualize_assembly.py

    # 複数 STL ファイルを別々のパーツとして読み込み
    python visualize_assembly.py part_a.stl part_b.stl part_c.stl

    # オプション
    python visualize_assembly.py part_a.stl part_b.stl --d 0.002 --side-by-side

依存:
    pip install open3d meshexpander
"""

import argparse
import sys
import colorsys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("open3d が見つかりません。  pip install open3d")

try:
    import meshexpander as me
except ImportError:
    sys.exit("meshexpander が見つかりません。ビルド後に  pip install .")


# ---------------------------------------------------------------------------
# 合成デモアセンブリ生成
# ---------------------------------------------------------------------------

def _box_mesh(cx, cy, cz, sx, sy, sz) -> me.Mesh:
    """中心 (cx,cy,cz)・サイズ (sx,sy,sz) の直方体メッシュ"""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    verts = np.array([
        [cx-hx, cy-hy, cz-hz], [cx+hx, cy-hy, cz-hz],
        [cx+hx, cy+hy, cz-hz], [cx-hx, cy+hy, cz-hz],
        [cx-hx, cy-hy, cz+hz], [cx+hx, cy-hy, cz+hz],
        [cx+hx, cy+hy, cz+hz], [cx-hx, cy+hy, cz+hz],
    ], dtype=float)
    faces = np.array([
        [0,2,1],[0,3,2],  # -Z
        [4,5,6],[4,6,7],  # +Z
        [0,1,5],[0,5,4],  # -Y
        [2,3,7],[2,7,6],  # +Y
        [0,4,7],[0,7,3],  # -X
        [1,2,6],[1,6,5],  # +X
    ], dtype=np.int32)
    return me.Mesh.from_arrays(verts, faces)


def _cylinder_mesh(cx, cy, cz, r, h, n=32) -> me.Mesh:
    """Z軸方向の円柱"""
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    top_verts = np.column_stack([cx + r * np.cos(angles),
                                  cy + r * np.sin(angles),
                                  np.full(n, cz + h / 2)])
    bot_verts = np.column_stack([cx + r * np.cos(angles),
                                  cy + r * np.sin(angles),
                                  np.full(n, cz - h / 2)])
    top_c = np.array([[cx, cy, cz + h / 2]])
    bot_c = np.array([[cx, cy, cz - h / 2]])

    verts = np.vstack([top_verts, bot_verts, top_c, bot_c])  # 2n+2 vertices
    top_ci = 2 * n
    bot_ci = 2 * n + 1

    faces = []
    for i in range(n):
        j = (i + 1) % n
        # 側面 (2 triangle)
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])
        # 上蓋
        faces.append([top_ci, j, i])
        # 下蓋
        faces.append([bot_ci, n + i, n + j])

    return me.Mesh.from_arrays(verts, np.array(faces, dtype=np.int32))


def make_demo_assembly() -> list:
    """
    デモ用アセンブリ:
      Part 0 — 大きな直方体ベース (凸)
      Part 1 — 円柱ポスト x2 (凸)
      Part 2 — L 字ブラケット (凹: 2直方体を並べて作る)
    """
    parts = []

    # ベース直方体
    parts.append(_box_mesh(0, 0, -0.01, 0.12, 0.08, 0.02))

    # 円柱ポスト (左右2本)
    for cx in [-0.04, 0.04]:
        parts.append(_cylinder_mesh(cx, 0, 0.02, 0.008, 0.04))

    # L 字ブラケット (2直方体の合体)
    v1, f1 = np.asarray(_box_mesh(-0.06, 0,  0.025, 0.02, 0.06, 0.05).vertices), \
             np.asarray(_box_mesh(-0.06, 0,  0.025, 0.02, 0.06, 0.05).faces)
    v2, f2 = np.asarray(_box_mesh(-0.06, 0.04, 0.04, 0.02, 0.06, 0.02).vertices), \
             np.asarray(_box_mesh(-0.06, 0.04, 0.04, 0.02, 0.06, 0.02).faces)
    f2_offset = f2 + len(v1)
    v_all = np.vstack([v1, v2])
    f_all = np.vstack([f1, f2_offset])
    parts.append(me.Mesh.from_arrays(v_all, f_all.astype(np.int32)))

    return parts


# ---------------------------------------------------------------------------
# ユーティリティ
# ---------------------------------------------------------------------------

def palette(n: int) -> list:
    return [colorsys.hsv_to_rgb(i / max(n, 1), 0.7, 0.9) for i in range(n)]


def me_to_o3d(mesh, color=None) -> o3d.geometry.TriangleMesh:
    m = o3d.geometry.TriangleMesh()
    m.vertices  = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    m.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces))
    m.compute_vertex_normals()
    if color is not None:
        m.paint_uniform_color(color)
    return m


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="マルチパートアセンブリ可視化")
    parser.add_argument("inputs", nargs="*",
                        help="STL ファイルのリスト (省略するとデモアセンブリを使用)")
    parser.add_argument("--d", type=float, default=0.002, help="膨張距離 [m]")
    parser.add_argument("--max-pieces", type=int, default=1,
                        help="凹分解の最大ピース数/部品 (1=単一凸, >1 で concavity 駆動分割)")
    parser.add_argument("--concavity-tol", type=float, default=0.0,
                        help="concavity 許容値 [m] (>0 で分割を有効化)")
    parser.add_argument("--side-by-side", action="store_true",
                        help="元アセンブリを左、膨張後を右に横並び表示")
    args = parser.parse_args()

    # --- パーツ読み込み ---
    if args.inputs:
        if len(args.inputs) == 1 and not args.inputs[0].lower().endswith('.stl'):
            # 単一アセンブリファイル (DAE/FBX/OBJ 等) → Assimp で自動分割
            if not me.HAS_IO:
                sys.exit("load_assembly() はビルド時に MESHEXPANDER_BUILD_IO=ON が必要です。")
            print(f"読み込み (Assimp): {args.inputs[0]}")
            parts = me.load_assembly(args.inputs[0])
        else:
            # 複数 STL → パーツとして扱う
            parts = [me.read_stl(p) for p in args.inputs]
        print(f"  {len(args.inputs[0] if len(args.inputs)==1 else args.inputs)} → {len(parts)} パーツ")
    else:
        parts = make_demo_assembly()
        print(f"デモアセンブリを生成: {len(parts)} パーツ")

    # バウンディングボックス包含パーツを統合
    parts = me.merge_contained(parts)
    print(f"merge_contained 後: {len(parts)} パーツ")

    # --- 膨張 ---
    print(f"膨張中: d={args.d}, max_pieces={args.max_pieces}, "
          f"concavity_tol={args.concavity_tol} ...")
    opts = me.AssemblyExpanderOptions()
    opts.max_convex_pieces = args.max_pieces
    opts.concavity_tol     = args.concavity_tol
    expander = me.AssemblyExpander(opts)
    expanded = expander.expand(parts, args.d)
    print("完了")

    # --- 可視化 ---
    colors = palette(len(parts))
    geometries = []
    geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=max(args.d * 15, 0.015)))

    for i, (orig, exp) in enumerate(zip(parts, expanded)):
        if len(orig.faces) == 0:
            continue
        r, g, b = colors[i]
        print(f"  Part {i}: faces {len(orig.faces)} -> {len(exp.faces)}")

        # 元パーツ: 暗色の塗り + 黒ワイヤーフレーム
        orig_o3d = me_to_o3d(orig, color=[r * 0.35, g * 0.35, b * 0.35])
        orig_wf  = o3d.geometry.LineSet.create_from_triangle_mesh(orig_o3d)
        orig_wf.paint_uniform_color([0.0, 0.0, 0.0])

        # 膨張後: 明るい塗り + ワイヤーフレーム
        exp_o3d = me_to_o3d(exp, color=[r, g, b])
        exp_wf  = o3d.geometry.LineSet.create_from_triangle_mesh(exp_o3d)
        exp_wf.paint_uniform_color([r * 0.6, g * 0.6, b * 0.6])

        if args.side_by_side:
            bbox   = orig_o3d.get_axis_aligned_bounding_box()
            offset = (bbox.get_extent()[0] + args.d * 5) * 1.5
            orig_o3d.translate([-offset, 0, 0])
            orig_wf.translate([-offset, 0, 0])

        geometries += [orig_o3d, orig_wf, exp_o3d, exp_wf]

    print("\n操作: 左ドラッグ=回転 / 右ドラッグ=パン / スクロール=ズーム / Q=終了")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="MeshExpander Assembly — 暗色(同系色):元モデル  明色(同系色):膨張後",
        width=1280, height=720, mesh_show_back_face=True,
    )


if __name__ == "__main__":
    main()
