#!/usr/bin/env python3
"""Generate animated GIF demonstrating the ConservativeExpander algorithm.

The key concept: start with a bounding box, then carve away corners one by one
using each face normal's half-space plane. The result is the expansion model.

4 phases per shape:
  Phase 1: Input mesh          -- polygon outline
  Phase 2: Bounding box        -- axis-aligned box that encloses input + d
  Phase 3: Carving             -- each face normal clips the box, removing a wedge
  Phase 4: Result              -- carved expansion model (green) overlaid on input (blue)

3 shapes cycle: pentagon / hexagon / diamond hexagon

Output: docs/images/carving.gif
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection

os.makedirs(os.path.join(os.path.dirname(__file__), "images"), exist_ok=True)
OUTPUT = os.path.join(os.path.dirname(__file__), "images", "carving.gif")

# ── colour palette ─────────────────────────────────────────────────────────────
BG     = "#0d1117"
FG     = "#e6edf3"
BLUE   = "#58a6ff"
GREEN  = "#3fb950"
ORANGE = "#f0883e"
YELLOW = "#e3b341"
PURPLE = "#bc8cff"
GREY   = "#484f58"
RED    = "#f85149"

# ── 2D polygon utilities ───────────────────────────────────────────────────────

def make_regular(n, R=4.0, offset_deg=90.0):
    a = np.radians(np.linspace(offset_deg, offset_deg + 360, n, endpoint=False))
    return np.column_stack([R * np.cos(a), R * np.sin(a)]).astype(float)

def make_diamond(R=4.0):
    a = np.radians([90, 30, 330, 270, 210, 150])
    r = np.array([R, R*0.72, R, R, R*0.72, R])
    return np.column_stack([r * np.cos(a), r * np.sin(a)]).astype(float)

def make_arrowhead(R=4.0):
    """非対称な凸多角形（矢じり形）"""
    pts = np.array([
        [0,  R],
        [R*0.7,  0],
        [R*0.3, -R*0.4],
        [-R*0.3, -R*0.4],
        [-R*0.7,  0],
    ], dtype=float)
    return pts

def edge_normals_and_offsets(poly, d):
    """各辺の (外向き単位法線, D_i = max(V·n) + d, 辺の中点) を返す"""
    n = len(poly)
    normals, offsets, midpoints = [], [], []
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        edge = p1 - p0
        normal = np.array([edge[1], -edge[0]], dtype=float)
        normal /= np.linalg.norm(normal)
        D = float((poly @ normal).max()) + d   # D_i = max(V·n) + d
        normals.append(normal)
        offsets.append(D)
        midpoints.append((p0 + p1) * 0.5)
    return np.array(normals), np.array(offsets), np.array(midpoints)

def initial_bbox(poly, d, margin=1.5):
    """入力 + d + margin を包む軸平行矩形の頂点（反時計回り）"""
    mn = poly.min(axis=0) - d - margin
    mx = poly.max(axis=0) + d + margin
    return np.array([
        [mn[0], mn[1]],
        [mx[0], mn[1]],
        [mx[0], mx[1]],
        [mn[0], mx[1]],
    ], dtype=float)

def clip_halfspace(polygon, normal, D):
    """Sutherland-Hodgman: polygon を {x: x·n <= D} でクリップする"""
    result = []
    n = len(polygon)
    for i in range(n):
        curr = polygon[i]
        nxt  = polygon[(i + 1) % n]
        dc = float(curr @ normal) - D
        dn = float(nxt  @ normal) - D
        if dc <= 0:
            result.append(curr)
        if (dc < 0) != (dn < 0):
            t = dc / (dc - dn)
            result.append(curr + t * (nxt - curr))
    return np.array(result, dtype=float) if result else np.empty((0, 2))

def compute_carving_sequence(poly, d):
    """
    バウンディングボックスから始めて各半空間でクリップする列を返す。
    戻り値: list of (polygon_after_clip, normal, cut_line_endpoints)
    """
    normals, offsets, midpoints = edge_normals_and_offsets(poly, d)
    current = initial_bbox(poly, d)
    steps = []
    for k in range(len(normals)):
        n = normals[k]
        D = offsets[k]
        clipped = clip_halfspace(current, n, D)
        steps.append(dict(
            before=current.copy(),
            after=clipped.copy(),
            normal=n,
            D=D,
            midpoint=midpoints[k],
        ))
        current = clipped
    return steps, current  # current = final expansion polygon

# ── shapes ─────────────────────────────────────────────────────────────────────
D_EXPAND = 0.9

SHAPES = []
for poly, label in [
    (make_regular(5),   "Pentagon"),
    (make_arrowhead(),  "Arrow"),
    (make_diamond(),    "Diamond"),
]:
    steps, expansion = compute_carving_sequence(poly, D_EXPAND)
    bbox = initial_bbox(poly, D_EXPAND)
    SHAPES.append(dict(
        poly=poly, label=label,
        steps=steps, expansion=expansion, bbox=bbox,
    ))

# ── layout ─────────────────────────────────────────────────────────────────────
FPS = 16

# フレーム数設定
F_P1      = 20   # Phase1: input mesh
F_P2      = 16   # Phase2: bbox appears
F_P2H     = 8
F_CARVE   = 10   # Phase3: 1 carve step duration
F_P3H     = 14   # Phase3: hold after all carves
F_P4      = 14   # Phase4: result fade-in
F_P4H     = 22   # Phase4: hold
F_XFADE   = 12

def frames_for(sh):
    return F_P1 + F_P2 + F_P2H + F_CARVE * len(sh["steps"]) + F_P3H + F_P4 + F_P4H + F_XFADE

PERIOD = max(frames_for(sh) for sh in SHAPES)
TOTAL_F = PERIOD * len(SHAPES)

# ── figure ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 5.0), facecolor=BG)

ax_title = fig.add_axes([0.0, 0.91, 1.0, 0.09], facecolor=BG)
ax_title.axis("off")
ax_title.text(
    0.5, 0.50,
    "ConservativeExpander: Bounding Box Carving by Face Normals",
    transform=ax_title.transAxes, ha="center", va="center",
    fontsize=12.5, fontweight="bold", color=FG,
    path_effects=[pe.withStroke(linewidth=3, foreground=BG)],
)

PANEL_W = 0.235
PANEL_X = [0.01 + k * (PANEL_W + 0.012) for k in range(4)]
axes    = [fig.add_axes([x, 0.05, PANEL_W, 0.84], facecolor=BG)
           for x in PANEL_X]

PANEL_LABELS = [
    ("① Input Mesh",     BLUE),
    ("② Bounding Box",   ORANGE),
    ("③ Carving",        RED),
    ("④ Expansion Model",GREEN),
]
PANEL_SUB = [
    "face normals extracted",
    "bbox encloses input + d",
    "clip by each face normal",
    "original inside expansion",
]

for ax in axes:
    ax.set_aspect("equal")
    ax.axis("off")

# ── helpers ────────────────────────────────────────────────────────────────────

def view_bounds(sh, margin=1.2):
    """全ステップを包む表示範囲"""
    pts = np.vstack([sh["bbox"], sh["poly"]])
    mn = pts.min(axis=0) - margin
    mx = pts.max(axis=0) + margin
    c  = (mn + mx) / 2
    r  = max(mx - mn) / 2
    return c[0] - r, c[0] + r, c[1] - r, c[1] + r

def set_lim(ax, sh):
    xlo, xhi, ylo, yhi = view_bounds(sh)
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)

def draw_poly_fill(ax, poly, color, fill_alpha, edge_alpha=1.0, lw=2.0):
    if len(poly) < 3:
        return
    closed = np.vstack([poly, poly[0]])
    if fill_alpha > 0:
        ax.fill(*poly.T, color=color, alpha=fill_alpha, zorder=1)
    ax.plot(*closed.T, color=color, lw=lw, alpha=edge_alpha,
            zorder=3, solid_capstyle="round", solid_joinstyle="round")

def draw_normals(ax, sh, alpha=1.0, length=1.0):
    poly = sh["poly"]
    n = len(poly)
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        mid = (p0 + p1) * 0.5
        edge = p1 - p0
        nrm = np.array([edge[1], -edge[0]])
        nrm = nrm / np.linalg.norm(nrm) * length
        ax.annotate(
            "", xy=(mid[0] + nrm[0], mid[1] + nrm[1]),
            xytext=(mid[0], mid[1]),
            arrowprops=dict(arrowstyle="-|>", color=ORANGE,
                            lw=1.6, mutation_scale=12),
            alpha=alpha, zorder=5,
        )

def draw_cut_line(ax, sh, step_idx, alpha=1.0):
    """クリップ平面を示す破線を引く"""
    step = sh["steps"][step_idx]
    n = step["normal"]
    D = step["D"]
    xlo, xhi, ylo, yhi = view_bounds(sh)
    span = max(xhi - xlo, yhi - ylo) * 1.6
    # 法線に垂直な接線方向
    tx, ty = -n[1], n[0]
    Px, Py = n * D
    ax.plot([Px - tx * span, Px + tx * span],
            [Py - ty * span, Py + ty * span],
            color=RED, lw=1.8, linestyle="--",
            alpha=alpha * 0.85, zorder=4)
    # 法線矢印（切断方向を示す）
    mp = step["midpoint"]
    ax.annotate(
        "", xy=(Px, Py),
        xytext=(mp[0], mp[1]),
        arrowprops=dict(arrowstyle="-|>", color=ORANGE,
                        lw=1.4, mutation_scale=11),
        alpha=alpha * 0.7, zorder=6,
    )

def draw_labels(active_phase, alpha=1.0):
    for k, ax in enumerate(axes):
        label, color = PANEL_LABELS[k]
        done   = k < active_phase
        active = k == active_phase
        pending= k > active_phase
        c  = color if not pending else GREY
        ta = alpha if active else (alpha * 0.65 if done else alpha * 0.25)
        ax.text(0.5, 1.02, label, transform=ax.transAxes,
                ha="center", va="bottom", fontsize=9.5, fontweight="bold",
                color=c, alpha=ta)
        ax.text(0.5, -0.01, PANEL_SUB[k], transform=ax.transAxes,
                ha="center", va="top", fontsize=7.5,
                color=FG if active else GREY, alpha=ta * 0.85)

# ── animate ────────────────────────────────────────────────────────────────────

def animate(frame):
    for ax in axes:
        ax.cla()
        ax.set_facecolor(BG)
        ax.axis("off")

    si  = min(frame // PERIOD, len(SHAPES) - 1)
    sh  = SHAPES[si]
    loc = frame - si * PERIOD

    nsteps    = len(sh["steps"])
    F_P3_TOTAL= F_CARVE * nsteps
    t_p1_end  = F_P1
    t_p2_end  = t_p1_end + F_P2 + F_P2H
    t_p3_end  = t_p2_end + F_P3_TOTAL + F_P3H
    t_p4_end  = t_p3_end + F_P4 + F_P4H
    t_xfade   = t_p4_end

    alpha = 1.0 if loc < t_xfade else max(0.0, 1.0 - (loc - t_xfade) / F_XFADE)

    # アクティブフェーズ判定
    if loc < t_p1_end:
        active_phase = 0
    elif loc < t_p2_end:
        active_phase = 1
    elif loc < t_p3_end:
        active_phase = 2
    else:
        active_phase = 3

    # ── Panel 0: Input Mesh ──────────────────────────────────────────────────
    ax = axes[0]
    set_lim(ax, sh)
    draw_poly_fill(ax, sh["poly"], BLUE, 0.12 * alpha, alpha)
    draw_normals(ax, sh, alpha=alpha * 0.75)
    ax.text(0.5, 0.94, sh["label"], transform=ax.transAxes,
            ha="center", va="top", fontsize=10, fontweight="bold",
            color=FG, alpha=alpha)

    # ── Panel 1: Bounding Box ────────────────────────────────────────────────
    if loc >= t_p1_end:
        ax = axes[1]
        set_lim(ax, sh)
        # bbox の出現フェード
        t_bb = np.clip((loc - t_p1_end) / max(F_P2, 1), 0.0, 1.0)
        draw_poly_fill(ax, sh["poly"], BLUE, 0.08 * alpha, 0.4 * alpha)
        draw_poly_fill(ax, sh["bbox"], ORANGE, 0.06 * t_bb * alpha,
                       t_bb * alpha, lw=1.8)
        # d の矢印（bbox 右辺 → poly 右辺）
        if t_bb > 0.5:
            arr_a = (t_bb - 0.5) * 2 * alpha
            rx_poly = sh["poly"][:, 0].max()
            rx_bbox = sh["bbox"][:, 0].max()
            cy = 0.0
            ax.annotate(
                "", xy=(rx_bbox, cy), xytext=(rx_poly, cy),
                arrowprops=dict(arrowstyle="<->", color=YELLOW,
                                lw=1.4, mutation_scale=11),
                alpha=arr_a, zorder=7,
            )
            ax.text((rx_poly + rx_bbox) / 2, cy + 0.3, "d",
                    color=YELLOW, fontsize=9, fontweight="bold",
                    ha="center", alpha=arr_a)

    # ── Panel 2: Carving ─────────────────────────────────────────────────────
    if loc >= t_p2_end:
        ax = axes[2]
        set_lim(ax, sh)
        elapsed = loc - t_p2_end
        # 何ステップ目まで完了しているか
        step_done = min(int(elapsed // F_CARVE), nsteps)
        step_frac = np.clip((elapsed % F_CARVE) / F_CARVE, 0.0, 1.0) \
                    if step_done < nsteps else 1.0

        # 現在の削られたポリゴン
        if step_done == 0:
            current_poly = sh["bbox"]
        else:
            current_poly = sh["steps"][step_done - 1]["after"]

        # 元形状（薄く）
        draw_poly_fill(ax, sh["poly"], BLUE, 0.06 * alpha, 0.35 * alpha)

        # 現在のポリゴン（削られていく）
        draw_poly_fill(ax, current_poly, ORANGE, 0.10 * alpha,
                       alpha, lw=2.0)

        # 現在削っている面の切断線とアニメーション
        if step_done < nsteps and elapsed >= 0:
            draw_cut_line(ax, sh, step_done, alpha=step_frac * alpha)
            # 切断後のプレビュー（半透明で次の形）
            if step_frac > 0.4:
                next_poly = sh["steps"][step_done]["after"]
                t_preview = (step_frac - 0.4) / 0.6
                draw_poly_fill(ax, next_poly, GREEN,
                               0.12 * t_preview * alpha,
                               t_preview * alpha * 0.6, lw=1.5)

        # 何ステップ目か表示
        if step_done > 0 and step_done <= nsteps:
            ax.text(0.5, 0.06,
                    f"cut {step_done}/{nsteps}",
                    transform=ax.transAxes, ha="center", va="bottom",
                    fontsize=8.5, color=RED, alpha=alpha)

    # ── Panel 3: Expansion Model ─────────────────────────────────────────────
    if loc >= t_p3_end:
        ax = axes[3]
        set_lim(ax, sh)
        t_fade = np.clip((loc - t_p3_end) / F_P4, 0.0, 1.0)
        # 膨張モデル（緑）
        draw_poly_fill(ax, sh["expansion"], GREEN,
                       0.18 * t_fade * alpha, t_fade * alpha, lw=2.2)
        # 元形状（青、内側に収まっている）
        draw_poly_fill(ax, sh["poly"], BLUE,
                       0.15 * t_fade * alpha, 0.85 * t_fade * alpha)
        # d の矢印
        if t_fade > 0.6:
            arr_a = (t_fade - 0.6) / 0.4 * alpha
            rx_p = sh["poly"][:, 0].max()
            rx_e = sh["expansion"][:, 0].max()
            cy = 0.0
            ax.annotate(
                "", xy=(rx_e, cy), xytext=(rx_p, cy),
                arrowprops=dict(arrowstyle="<->", color=YELLOW,
                                lw=1.4, mutation_scale=11),
                alpha=arr_a, zorder=7,
            )
            ax.text((rx_p + rx_e) / 2, cy + 0.3, "d",
                    color=YELLOW, fontsize=9, fontweight="bold",
                    ha="center", alpha=arr_a)

    draw_labels(active_phase, alpha=alpha)
    return []

# ── render ─────────────────────────────────────────────────────────────────────
anim = FuncAnimation(fig, animate, frames=TOTAL_F,
                     init_func=lambda: [], blit=False, repeat=True)

print(f"Rendering {TOTAL_F} frames ({len(SHAPES)} shapes) -> {OUTPUT}")
sys.stdout.flush()
anim.save(OUTPUT, writer="pillow", fps=FPS, dpi=90)
print(f"Done  {os.path.getsize(OUTPUT) / 1024:.0f} KB  -> {OUTPUT}")
plt.close()
