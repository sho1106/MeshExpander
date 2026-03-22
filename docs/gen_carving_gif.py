#!/usr/bin/env python3
"""Generate animated GIF demonstrating the ConservativeExpander algorithm.

4 phases per shape:
  Phase 1: Input mesh      — polygon outline
  Phase 2: Face normals    — outward normal vectors appear edge by edge
  Phase 3: Half-spaces     — boundary planes D_i = max(V·n_i) + d appear one by one
  Phase 4: Expansion model — intersection of all half-spaces (green)

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
from matplotlib.animation import FuncAnimation

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

# ── 2D polygon utilities ───────────────────────────────────────────────────────

def make_polygon(angles_deg, radii):
    """任意の角度・半径で凸多角形を生成する"""
    angles = np.radians(angles_deg)
    pts = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    return pts.astype(float)

def make_regular(n, R=4.0, offset_deg=90.0):
    a = np.linspace(offset_deg, offset_deg + 360, n, endpoint=False)
    return make_polygon(a, R)

def make_diamond(R=4.0):
    a = [90, 30, 330, 270, 210, 150]
    r = [R, R*0.75, R, R, R*0.75, R]
    return make_polygon(a, r)

def edge_normals(poly):
    """各辺の外向き単位法線と中点を返す (n 辺分)"""
    n = len(poly)
    normals, midpoints = [], []
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        edge = p1 - p0
        # 右回り法線 (外向き)
        normal = np.array([edge[1], -edge[0]])
        normal /= np.linalg.norm(normal)
        normals.append(normal)
        midpoints.append((p0 + p1) * 0.5)
    return np.array(normals), np.array(midpoints)

def halfspace_offset(poly, normal, d):
    """半空間境界: D = max(V·n) + d を返す"""
    projections = poly @ normal
    return projections.max() + d

def clip_polygon_halfspace(polygon, normal, D):
    """Sutherland-Hodgman で凸多角形を半空間 {x: x·n <= D} でクリップする"""
    result = []
    n = len(polygon)
    for i in range(n):
        curr = polygon[i]
        nxt  = polygon[(i + 1) % n]
        d_curr = curr @ normal - D
        d_nxt  = nxt  @ normal - D
        if d_curr <= 0:          # curr is inside
            result.append(curr)
        if (d_curr < 0) != (d_nxt < 0):  # edge crosses boundary
            t = d_curr / (d_curr - d_nxt)
            result.append(curr + t * (nxt - curr))
    return np.array(result) if result else np.empty((0, 2))

def compute_expansion(poly, normals, d, bound=20.0):
    """全半空間の交差として膨張モデルを計算する"""
    # 大きな初期矩形から出発
    result = np.array([[-bound, -bound], [ bound, -bound],
                       [ bound,  bound], [-bound,  bound]], dtype=float)
    for normal in normals:
        D = halfspace_offset(poly, normal, d)
        result = clip_polygon_halfspace(result, normal, D)
        if len(result) == 0:
            break
    return result

def poly_bounds(polys, margin=1.5):
    """複数のポリゴンを内包する軸範囲を返す"""
    all_pts = np.vstack(polys)
    mn = all_pts.min(axis=0) - margin
    mx = all_pts.max(axis=0) + margin
    cx = (mn[0] + mx[0]) / 2
    cy = (mn[1] + mx[1]) / 2
    r  = max(mx[0] - mn[0], mx[1] - mn[1]) / 2
    return cx - r, cx + r, cy - r, cy + r

# ── shapes ─────────────────────────────────────────────────────────────────────
D_EXPAND = 1.0   # 膨張量 d

SHAPES = []
for poly, label in [
    (make_regular(5),   "Pentagon"),
    (make_regular(6),   "Hexagon"),
    (make_diamond(),    "Diamond Hex"),
]:
    normals, midpoints = edge_normals(poly)
    expansion = compute_expansion(poly, normals, D_EXPAND)
    SHAPES.append(dict(
        poly=poly, normals=normals, midpoints=midpoints,
        expansion=expansion, label=label,
    ))

# ── animation parameters ───────────────────────────────────────────────────────
FPS      = 16
F_P1     = 18   # Phase1 hold (input mesh)
F_P2     = 24   # Phase2 duration (normals appear)
F_P2H    = 10   # Phase2 hold
F_P3     = 28   # Phase3 duration (half-spaces appear)
F_P3H    = 12   # Phase3 hold
F_P4     = 14   # Phase4 duration (expansion fills)
F_P4H    = 18   # Phase4 hold
F_XFADE  = 10   # cross-fade frames

PERIOD   = F_P1 + F_P2 + F_P2H + F_P3 + F_P3H + F_P4 + F_P4H + F_XFADE
TOTAL_F  = PERIOD * len(SHAPES)

# ── figure layout ──────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 4.8), facecolor=BG)

ax_title = fig.add_axes([0.0, 0.90, 1.0, 0.10], facecolor=BG)
ax_title.axis("off")
ax_title.text(
    0.5, 0.50,
    "MeshExpander — Conservative Expansion via Half-Space Intersection",
    transform=ax_title.transAxes, ha="center", va="center",
    fontsize=12.5, fontweight="bold", color=FG,
    path_effects=[pe.withStroke(linewidth=3, foreground=BG)],
)

# 4 パネル: 入力 / 面法線 / 半空間生成 / 膨張モデル
PANEL_W = 0.235
PANEL_X = [0.01 + k * (PANEL_W + 0.012) for k in range(4)]
axes    = [fig.add_axes([x, 0.05, PANEL_W, 0.82], facecolor=BG)
           for x in PANEL_X]

PANEL_LABELS = [
    ("① Input Mesh",      BLUE),
    ("② Face Normals",    ORANGE),
    ("③ Half-Spaces",     PURPLE),
    ("④ Expansion Model", GREEN),
]
PANEL_SUB = [
    "polygon boundary",
    "outward unit normals",
    "D\u1d62 = max(V\u00b7n\u1d62) + d",
    "intersection of all half-spaces",
]

for ax in axes:
    ax.set_aspect("equal")
    ax.axis("off")

def set_lim(ax, sh):
    polys = [sh["poly"], sh["expansion"]] if len(sh["expansion"]) else [sh["poly"]]
    xlo, xhi, ylo, yhi = poly_bounds(polys, margin=1.8)
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)

def draw_labels(active_phase, alpha=1.0):
    for k, ax in enumerate(axes):
        label, color = PANEL_LABELS[k]
        c     = color if k <= active_phase else GREY
        a_txt = alpha if k == active_phase else (alpha * 0.6 if k < active_phase else alpha * 0.25)
        ax.text(0.5, 1.03, label, transform=ax.transAxes,
                ha="center", va="bottom", fontsize=9.5, fontweight="bold",
                color=c, alpha=a_txt)
        ax.text(0.5, -0.02, PANEL_SUB[k], transform=ax.transAxes,
                ha="center", va="top", fontsize=7.5,
                color=FG if k == active_phase else GREY,
                alpha=a_txt * 0.85)

def draw_polygon(ax, poly, color, lw=2.0, alpha=1.0, fill_alpha=0.0):
    closed = np.vstack([poly, poly[0]])
    if fill_alpha > 0:
        ax.fill(*poly.T, color=color, alpha=fill_alpha, zorder=1)
    ax.plot(*closed.T, color=color, lw=lw, alpha=alpha, zorder=3,
            solid_capstyle="round", solid_joinstyle="round")

def draw_shape_name(ax, sh, alpha=1.0):
    ax.text(0.5, 0.93, sh["label"], transform=ax.transAxes,
            ha="center", va="top", fontsize=10.5, fontweight="bold",
            color=FG, alpha=alpha)

def draw_normals(ax, sh, count, alpha=1.0, arrow_len=1.2):
    mids = sh["midpoints"]
    norms = sh["normals"]
    n_total = len(mids)
    for k in range(min(count, n_total)):
        mx, my = mids[k]
        nx, ny = norms[k] * arrow_len
        ax.annotate(
            "", xy=(mx + nx, my + ny), xytext=(mx, my),
            arrowprops=dict(arrowstyle="-|>", color=ORANGE,
                            lw=1.8, mutation_scale=14),
            alpha=alpha, zorder=5,
        )

def draw_halfspaces(ax, sh, count, alpha=1.0):
    """半空間境界線を引く（D_i = max(V·n_i) + d の位置）"""
    poly   = sh["poly"]
    norms  = sh["normals"]
    xlo, xhi, ylo, yhi = poly_bounds([poly, sh["expansion"]], margin=2.5)

    for k in range(min(count, len(norms))):
        n = norms[k]
        D = halfspace_offset(poly, n, D_EXPAND)
        # 法線方向に垂直な平行線を描く
        # 線上の1点: P = n * D
        Px, Py = n * D
        # 接線方向
        tx, ty = -n[1], n[0]
        # 両端まで延ばす (パラメトリック)
        tmax = max(xhi - xlo, yhi - ylo) * 1.5
        x0, y0 = Px - tx * tmax, Py - ty * tmax
        x1, y1 = Px + tx * tmax, Py + ty * tmax
        ax.plot([x0, x1], [y0, y1],
                color=PURPLE, lw=1.2, alpha=alpha * 0.65,
                linestyle="--", zorder=2)
        # D の注釈（最初の数本だけ）
        if k < 3:
            ax.text(Px + n[0] * 0.25, Py + n[1] * 0.25,
                    f"D{k+1}", color=PURPLE,
                    fontsize=7, alpha=alpha * 0.85, ha="center", va="center",
                    zorder=6)

def draw_expansion(ax, sh, t, alpha=1.0):
    """膨張モデルを t (0→1) で徐々に塗りつぶす"""
    exp = sh["expansion"]
    if len(exp) < 3:
        return
    ax.fill(*exp.T, color=GREEN, alpha=min(t, 1.0) * 0.30 * alpha, zorder=1)
    draw_polygon(ax, exp, GREEN, lw=2.2, alpha=min(t, 1.0) * alpha)
    draw_polygon(ax, sh["poly"], BLUE, lw=1.8,
                 alpha=0.80 * min(t, 1.0) * alpha)

# ── main animate function ──────────────────────────────────────────────────────

def animate(frame):
    for ax in axes:
        ax.cla()
        ax.set_facecolor(BG)
        ax.axis("off")

    si  = min(frame // PERIOD, len(SHAPES) - 1)
    sh  = SHAPES[si]
    loc = frame - si * PERIOD

    # クロスフェード alpha
    xfade_start = PERIOD - F_XFADE
    alpha = 1.0 if loc < xfade_start else max(0.0, 1.0 - (loc - xfade_start) / F_XFADE)

    # 各フェーズの境界
    t1 = F_P1
    t2 = t1 + F_P2 + F_P2H
    t3 = t2 + F_P3 + F_P3H
    t4 = t3 + F_P4 + F_P4H

    # ── Phase 1: 入力メッシュ ──────────────────────────────────────────────────
    if loc <= t4:
        ax = axes[0]
        set_lim(ax, sh)
        draw_polygon(ax, sh["poly"], BLUE, alpha=alpha, fill_alpha=0.10 * alpha)
        draw_shape_name(ax, sh, alpha=alpha)

    # ── Phase 2: 面法線 ────────────────────────────────────────────────────────
    if loc >= t1:
        ax = axes[1]
        set_lim(ax, sh)
        draw_polygon(ax, sh["poly"], BLUE, alpha=alpha * 0.7, fill_alpha=0.07 * alpha)
        elapsed = loc - t1
        n_total = len(sh["normals"])
        n_show  = int(np.clip(elapsed / F_P2 * n_total, 0, n_total))
        draw_normals(ax, sh, n_show, alpha=alpha)
        # 全法線表示後は完全に
        if loc >= t1 + F_P2:
            draw_normals(ax, sh, n_total, alpha=alpha)

    # ── Phase 3: 半空間 ────────────────────────────────────────────────────────
    if loc >= t2:
        ax = axes[2]
        set_lim(ax, sh)
        draw_polygon(ax, sh["poly"], BLUE, alpha=alpha * 0.5, fill_alpha=0.05 * alpha)
        elapsed = loc - t2
        n_total = len(sh["normals"])
        n_show  = int(np.clip(elapsed / F_P3 * n_total, 0, n_total))
        draw_halfspaces(ax, sh, n_show, alpha=alpha)
        # 法線も薄く残す
        draw_normals(ax, sh, n_total, alpha=alpha * 0.35, arrow_len=0.7)
        if loc >= t2 + F_P3:
            draw_halfspaces(ax, sh, n_total, alpha=alpha)

    # ── Phase 4: 削り出し膨張 ──────────────────────────────────────────────────
    if loc >= t3:
        ax = axes[3]
        set_lim(ax, sh)
        # 半空間境界を薄く残す
        draw_halfspaces(ax, sh, len(sh["normals"]), alpha=alpha * 0.25)
        elapsed = loc - t3
        t_fill  = np.clip(elapsed / F_P4, 0.0, 1.0)
        draw_expansion(ax, sh, t_fill, alpha=alpha)
        # 膨張量 d の矢印
        if t_fill > 0.6 and len(sh["expansion"]) >= 3:
            exp = sh["expansion"]
            poly = sh["poly"]
            # 最も右の辺の外向き点で矢印
            k = int(np.argmax(exp[:, 0]))
            px = poly[np.argmax(poly[:, 0])]
            ex = exp[k]
            mid_y = (px[1] + ex[1]) / 2
            ax.annotate(
                "", xy=(ex[0], mid_y), xytext=(px[0], mid_y),
                arrowprops=dict(arrowstyle="<->", color=YELLOW,
                                lw=1.6, mutation_scale=12),
                alpha=t_fill * alpha, zorder=7,
            )
            ax.text((px[0] + ex[0]) / 2, mid_y + 0.35, "d",
                    color=YELLOW, fontsize=10, fontweight="bold",
                    ha="center", va="bottom",
                    alpha=t_fill * alpha, zorder=8)

    # アクティブフェーズを計算してラベルを描く
    if loc < t1:
        phase = 0
    elif loc < t2:
        phase = 1
    elif loc < t3:
        phase = 2
    else:
        phase = 3

    draw_labels(phase, alpha=alpha)
    return []


# ── render ─────────────────────────────────────────────────────────────────────

anim = FuncAnimation(fig, animate, frames=TOTAL_F,
                     init_func=lambda: [], blit=False, repeat=True)

print(f"Rendering {TOTAL_F} frames ({len(SHAPES)} shapes) → {OUTPUT}")
sys.stdout.flush()
anim.save(OUTPUT, writer="pillow", fps=FPS, dpi=90)
print(f"Done  {os.path.getsize(OUTPUT) / 1024:.0f} KB  → {OUTPUT}")
plt.close()
