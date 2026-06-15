"""
visualize_compare.py — 膨張モデルの比較ビューア（再設計版）

1 画面で「単一凸 (K=1)」と「concavity 駆動分解 (K>1)」を左右に並べ、凹ポケットが
「埋まる / 保たれる」差を一目で見せる。元形状の輪郭を重ねて膨張量も把握できる。

  - 対話 (filament): 元=不透明グレー + 膨張=半透明ソリッド（殻と体積が見え、継ぎ目ノイズが消える）
  - 静止画 (legacy/オフスクリーン): 膨張=ソリッド配色 + 元形状=黒ワイヤー輪郭

使い方:
    python visualize_compare.py                         # L 字デモを対話表示
    python visualize_compare.py model.stl --d 2.0 --max-pieces 8
    python visualize_compare.py --screenshot out.png    # GUI を開かず PNG 保存

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
    sys.exit("meshexpander が見つかりません。PYTHONPATH に python/ を通すか pip install . してください。")


# ---------------------------------------------------------------------------
# デモ形状（凹コーナーを持つ L 字プリズム）
# ---------------------------------------------------------------------------
def make_lshape(R=10.0, H=20.0) -> me.Mesh:
    hz = H * 0.5
    v = np.array([
        [-R, -R, hz], [R, -R, hz], [R, 0, hz], [0, 0, hz], [0, R, hz], [-R, R, hz],
        [-R, -R, -hz], [R, -R, -hz], [R, 0, -hz], [0, 0, -hz], [0, R, -hz], [-R, R, -hz],
    ], dtype=np.float64)
    f = np.array([
        [0,1,2],[0,2,3],[0,3,4],[0,4,5],
        [6,8,7],[6,9,8],[6,10,9],[6,11,10],
        [0,6,7],[0,7,1],[1,7,8],[1,8,2],[2,8,9],[2,9,3],
        [3,9,10],[3,10,4],[4,10,11],[4,11,5],[5,11,6],[5,6,0],
    ], dtype=np.int32)
    return me.Mesh.from_arrays(v, f)


def to_o3d(mesh, translate) -> o3d.geometry.TriangleMesh:
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) + np.asarray(translate))
    m.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces))
    m.compute_vertex_normals()
    return m


def mesh_volume(mesh) -> float:
    V = np.asarray(mesh.vertices); F = np.asarray(mesh.faces)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    return abs(np.sum(np.einsum('ij,ij->i', a, np.cross(b, c)))) / 6.0


GREY  = (0.62, 0.64, 0.68)
RED   = (0.93, 0.30, 0.24)
GREEN = (0.22, 0.78, 0.42)


def build(mesh, d, max_pieces):
    """元形状・単一凸・分解を生成し、左右に配置した o3d メッシュ群を返す。"""
    V = np.asarray(mesh.vertices)
    ext = V.max(0) - V.min(0)
    gap = ext[0] * 1.9
    L = np.array([-gap / 2, 0, 0]); Rg = np.array([gap / 2, 0, 0])

    single = me.AssemblyExpander().expand([mesh], d)[0]
    opts = me.AssemblyExpanderOptions(); opts.max_convex_pieces = max_pieces
    multi = me.AssemblyExpander(opts).expand([mesh], d)[0]

    g = {
        "orig_L": to_o3d(mesh, L),  "exp_L": to_o3d(single, L),
        "orig_R": to_o3d(mesh, Rg), "exp_R": to_o3d(multi, Rg),
    }
    top = ext[2] * 0.5 + d + ext[1] * 0.30
    labels = [
        (L  + [0, 0, top], "K=1  convex hull  ->  pocket FILLED"),
        (Rg + [0, 0, top], "K=%d  decomposed  ->  pocket KEPT" % max_pieces),
    ]
    vin = mesh_volume(mesh)
    info = dict(d=d, max_pieces=max_pieces, vin=vin,
                single_vr=mesh_volume(single) / vin,
                single_faces=single.num_faces(), multi_faces=multi.num_faces())
    return g, labels, np.array([0, 0, 0.0]), float(gap), info


def print_summary(info):
    print("\n  ── 膨張比較サマリ (d=%.3g) ─────────────────────────" % info["d"])
    print("   単一凸  K=1 : VolRatio=%.3f   faces=%d" % (info["single_vr"], info["single_faces"]))
    print("   分解  K=%-3d : faces=%d  (凹ポケットを保持 → 和体積は単一凸より小)"
          % (info["max_pieces"], info["multi_faces"]))
    print("  ────────────────────────────────────────────────────\n")


def eye_of(center, radius):
    return center + np.array([radius * 0.15, -radius * 1.9, radius * 1.2])


# --- 静止画: 旧 OpenGL パイプライン（オフスクリーン可） -------------------
def render_png(g, center, radius, path, w=1600, h=900):
    def solid(m, color):
        m.paint_uniform_color(color); return m
    def wire(m):
        ls = o3d.geometry.LineSet.create_from_triangle_mesh(m)
        ls.paint_uniform_color([0.05, 0.05, 0.05]); return ls

    geoms = [
        solid(g["exp_L"], RED), wire(g["orig_L"]),
        solid(g["exp_R"], GREEN), wire(g["orig_R"]),
    ]
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=w, height=h)
    for x in geoms:
        vis.add_geometry(x)
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1]); opt.mesh_show_back_face = True
    opt.light_on = True
    vc = vis.get_view_control()
    vc.set_lookat(center); vc.set_front([-0.15, 0.9, -0.55]); vc.set_up([0, 0, 1]); vc.set_zoom(0.62)
    vis.poll_events(); vis.update_renderer()
    vis.capture_screen_image(path, do_render=True)
    vis.destroy_window()
    print("screenshot ->", path)


# --- 対話: filament（半透明ソリッド + 3D ラベル） -------------------------
def show_gui(g, labels, center, radius, title):
    import open3d.visualization.rendering as rendering
    import open3d.visualization.gui as gui

    def opaque(c):
        m = rendering.MaterialRecord(); m.shader = "defaultLit"; m.base_color = [*c, 1.0]; return m
    def glass(c, a=0.45):
        m = rendering.MaterialRecord(); m.shader = "defaultLitTransparency"; m.base_color = [*c, a]; return m

    app = gui.Application.instance; app.initialize()
    vis = o3d.visualization.O3DVisualizer(title, 1600, 900)
    vis.show_settings = True
    vis.set_background([1, 1, 1, 1], None)
    vis.add_geometry("orig (K=1)", g["orig_L"], opaque(GREY))
    vis.add_geometry("convex K=1", g["exp_L"], glass(RED))
    vis.add_geometry("orig (K=N)", g["orig_R"], opaque(GREY))
    vis.add_geometry("decomposed", g["exp_R"], glass(GREEN))
    for pos, txt in labels:
        vis.add_3d_label(pos, txt)
    vis.reset_camera_to_default()
    vis.setup_camera(50.0, center, eye_of(center, radius), [0, 0, 1])
    app.add_window(vis)
    app.run()


def main():
    ap = argparse.ArgumentParser(description="膨張モデル比較ビューア（単一凸 vs 分解）")
    ap.add_argument("input", nargs="?", help="入力 STL（省略時は L 字デモ）")
    ap.add_argument("--d", type=float, default=2.0, help="膨張距離（既定 2.0: 殻が見やすい）")
    ap.add_argument("--max-pieces", type=int, default=8, help="分解の最大ピース数（既定 8）")
    ap.add_argument("--screenshot", metavar="PNG", help="GUI を開かず PNG 保存")
    args = ap.parse_args()

    mesh = me.read_stl(args.input) if args.input else make_lshape()
    g, labels, center, radius, info = build(mesh, args.d, args.max_pieces)
    print_summary(info)

    if args.screenshot:
        render_png(g, center, radius, args.screenshot)
    else:
        print("操作: 左ドラッグ=回転 / 右ドラッグ=パン / スクロール=ズーム / 右上=設定パネル")
        show_gui(g, labels, center, radius,
                 "MeshExpander 膨張比較 — 赤:単一凸(ポケット充填)  緑:分解(ポケット保持)")


if __name__ == "__main__":
    main()
