#!/usr/bin/env python3
'''Generate animated GIF demonstrating the MeshExpander algorithm.

Output: docs/images/demo.gif

Layout: 3 columns (Input / Voxelization / Expansion) + top title bar
Shapes: star prism, hollow cylinder, C-channel cycling through
Animation: 3 phases, smooth transitions
'''

import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation

os.makedirs(os.path.join(os.path.dirname(__file__), "images"), exist_ok=True)
OUTPUT = os.path.join(os.path.dirname(__file__), "images", "demo.gif")

# colour palette
BG     = "#0d1117"
FG     = "#e6edf3"
BLUE   = "#58a6ff"
ORANGE = "#f0883e"
GREEN  = "#3fb950"
YELLOW = "#e3b341"
GREY   = "#484f58"

GRID_N  = 16
D_CELLS = 1.6

def make_star(n=5, Ro=5.0, Ri=2.1):
    pts = []
    for i in range(n * 2):
        a = np.pi / 2 + i * np.pi / n
        r = Ro if i % 2 == 0 else Ri
        pts.append([r * np.cos(a), r * np.sin(a)])
    return np.array(pts)

def make_hollow_circle(R_out=5.2, R_in=2.5, N=60):
    t = np.linspace(0, 2 * np.pi, N, endpoint=False)
    outer = np.c_[R_out * np.cos(t), R_out * np.sin(t)]
    inner = np.c_[R_in  * np.cos(t[::-1]), R_in * np.sin(t[::-1])]
    return outer, inner

def make_c_shape(R=5.0, r=2.2, gap_deg=70, N=48):
    gap = np.radians(gap_deg)
    t = np.linspace(gap / 2, 2 * np.pi - gap / 2, N)
    outer = np.c_[R * np.cos(t), R * np.sin(t)]
    inner = np.c_[r * np.cos(t[::-1]), r * np.sin(t[::-1])]
    poly = np.vstack([outer, inner])
    return poly

def voxelize(poly_outer, poly_inner=None, n=GRID_N):
    all_pts = poly_outer if poly_inner is None else np.vstack([poly_outer, poly_inner])
    pad = 1.4
    xmin = all_pts[:, 0].min() - pad;  xmax = all_pts[:, 0].max() + pad
    ymin = all_pts[:, 1].min() - pad;  ymax = all_pts[:, 1].max() + pad
    dx = (xmax - xmin) / n;  dy = (ymax - ymin) / n

    cx = xmin + (np.arange(n) + .5) * dx
    cy = ymin + (np.arange(n) + .5) * dy
    CX, CY = np.meshgrid(cx, cy)
    pts = np.c_[CX.ravel(), CY.ravel()]

    in_out = Path(np.vstack([poly_outer, poly_outer[0]])).contains_points(pts).reshape(n, n)
    if poly_inner is not None:
        in_in = Path(np.vstack([poly_inner, poly_inner[0]])).contains_points(pts).reshape(n, n)
        solid = in_out & ~in_in
    else:
        solid = in_out

    cells = np.zeros((n, n), dtype=int)
    cells[solid] = 1
    return cells, xmin, ymin, dx, dy

def expand_cells(cells, d=D_CELLS):
    n = cells.shape[0]
    pad = int(np.ceil(d))
    exp = np.zeros_like(cells)
    for i in range(n):
        for j in range(n):
            if cells[i, j]:
                for di in range(-pad, pad + 1):
                    for dj in range(-pad, pad + 1):
                        if di * di + dj * dj <= d * d:
                            ni, nj = i + di, j + dj
                            if 0 <= ni < n and 0 <= nj < n:
                                exp[ni, nj] = 1
    return exp

# build shapes
star_poly = make_star()
hc_outer, hc_inner = make_hollow_circle()
c_poly = make_c_shape()

SHAPES = [
    dict(name="Star Prism",      outer=star_poly, inner=None,     color=BLUE),
    dict(name="Hollow Cylinder", outer=hc_outer,  inner=hc_inner, color=GREEN),
    dict(name="C-Channel",       outer=c_poly,    inner=None,     color=YELLOW),
]

for sh in SHAPES:
    sh["cells"], sh["xmin"], sh["ymin"], sh["dx"], sh["dy"] = voxelize(sh["outer"], sh["inner"])
    sh["exp"] = expand_cells(sh["cells"])
    rng = np.random.default_rng(42)
    solid = [(i, j) for i in range(GRID_N) for j in range(GRID_N) if sh["cells"][i, j]]
    exp_zone = [(i, j) for i in range(GRID_N) for j in range(GRID_N)
                if sh["exp"][i, j] and not sh["cells"][i, j]]
    rng.shuffle(solid)
    rng.shuffle(exp_zone)
    sh["solid_ij"] = solid
    sh["exp_ij"]   = exp_zone

FPS     = 18
F_PHASE = 28
F_HOLD  = 10
F_XFADE = 8
PHASES  = 3
PERIOD  = PHASES * (F_PHASE + F_HOLD) + F_XFADE
TOTAL_F = len(SHAPES) * PERIOD

fig = plt.figure(figsize=(12, 4.4), facecolor=BG)

ax_title = fig.add_axes([0.0, 0.88, 1.0, 0.12], facecolor=BG)
ax_title.axis("off")
ax_title.text(
    0.5, 0.45,
    "MeshExpander — Conservative 3D Mesh Expansion",
    transform=ax_title.transAxes, ha="center", va="center",
    fontsize=13, fontweight="bold", color=FG,
    path_effects=[pe.withStroke(linewidth=3, foreground=BG)],
)

axes = [fig.add_axes([0.01 + k * 0.334, 0.04, 0.31, 0.82], facecolor=BG)
        for k in range(3)]

PANEL_LABELS = [
    ("① Input Mesh",                 BLUE),
    ("② Solid Voxelization",         ORANGE),
    ("③ Conservative Expansion ≥ d", GREEN),
]
PANEL_SUB = [
    "concave / any topology",
    "BFS flood-fill · top-down boxes",
    "every surface point covered",
]

for ax in axes:
    ax.set_aspect("equal")
    ax.axis("off")

def _lim(sh):
    n = GRID_N
    xs = sh["xmin"];  ys = sh["ymin"]
    dx = sh["dx"];    dy = sh["dy"]
    pad = dx * 0.8
    return (xs - pad, xs + n * dx + pad), (ys - pad, ys + n * dy + pad)

def draw_shape(ax, sh, alpha=1.0):
    outer = sh["outer"];  inner = sh.get("inner");  color = sh["color"]
    ax.fill(*outer.T, color=color, alpha=0.12 * alpha, zorder=1)
    ax.plot(*np.vstack([outer, outer[0]]).T,
            color=color, lw=2, alpha=alpha, zorder=3, solid_capstyle="round")
    if inner is not None:
        ax.fill(*inner.T, color=BG, alpha=1.0, zorder=2)
        ax.plot(*np.vstack([inner, inner[0]]).T,
                color=color, lw=2, alpha=alpha, zorder=4, solid_capstyle="round")

def draw_grid(ax, sh, alpha=0.14):
    xs = sh["xmin"];  ys = sh["ymin"]
    dx = sh["dx"];    dy = sh["dy"]
    for i in range(GRID_N + 1):
        ax.axhline(ys + i * dy, color=FG, lw=0.25, alpha=alpha, zorder=0)
        ax.axvline(xs + i * dx, color=FG, lw=0.25, alpha=alpha, zorder=0)

def add_rect(ax, sh, i, j, color, alpha=0.88):
    x = sh["xmin"] + j * sh["dx"]
    y = sh["ymin"] + i * sh["dy"]
    r = mpatches.Rectangle(
        (x, y), sh["dx"] * 0.94, sh["dy"] * 0.94,
        linewidth=0.35, edgecolor="#ffffff1a", facecolor=color, alpha=alpha)
    ax.add_patch(r)

def set_axis_limits(ax, sh):
    xl, yl = _lim(sh)
    ax.set_xlim(*xl);  ax.set_ylim(*yl)
    ax.set_aspect("equal")

def draw_panel_labels(phase, alpha=1.0):
    for k, ax in enumerate(axes):
        label, color = PANEL_LABELS[k]
        c = color if k <= phase else GREY
        ax.text(0.5, 1.04, label, transform=ax.transAxes,
                ha="center", va="bottom", fontsize=10, fontweight="bold",
                color=c, alpha=alpha)
        ax.text(0.5, -0.03, PANEL_SUB[k], transform=ax.transAxes,
                ha="center", va="top", fontsize=7.5,
                color=FG if k == phase else GREY, alpha=alpha * 0.8)

def clear_all():
    for ax in axes:
        ax.cla()
        ax.set_facecolor(BG)
        ax.axis("off")

def animate(frame):
    clear_all()
    shape_idx = min(frame // PERIOD, len(SHAPES) - 1)
    sh = SHAPES[shape_idx]
    local = frame - shape_idx * PERIOD

    xfade_start = PHASES * (F_PHASE + F_HOLD)
    if local >= xfade_start:
        alpha = 1.0 - (local - xfade_start) / F_XFADE
    else:
        alpha = 1.0

    period_p = F_PHASE + F_HOLD
    phase = min(local // period_p, PHASES - 1)
    lp = local - phase * period_p
    t = min(lp / F_PHASE, 1.0)

    # panel 0: input mesh
    ax0 = axes[0]
    set_axis_limits(ax0, sh)
    draw_grid(ax0, sh, alpha=0.10 * alpha)
    mesh_a = 1.0
    draw_shape(ax0, sh, alpha=mesh_a * alpha)
    # show shape name in top of panel 0 throughout phase 0
    if phase == 0:
        ax0.text(0.5, 0.95, sh["name"],
                 transform=ax0.transAxes, ha="center", va="top",
                 fontsize=11, fontweight="bold", color=sh["color"],
                 alpha=alpha)

    # panel 1: voxelization
    ax1 = axes[1]
    set_axis_limits(ax1, sh)
    if phase >= 1:
        draw_grid(ax1, sh, alpha=0.16 * alpha)
        n_show = (len(sh["solid_ij"]) if phase > 1
                  else int(t * len(sh["solid_ij"])))
        for (i, j) in sh["solid_ij"][:n_show]:
            add_rect(ax1, sh, i, j, ORANGE, alpha=0.88 * alpha)

    # panel 2: expansion
    ax2 = axes[2]
    set_axis_limits(ax2, sh)
    if phase >= 2:
        for (i, j) in sh["solid_ij"]:
            add_rect(ax2, sh, i, j, ORANGE, alpha=0.88 * alpha)
        n_show = int(t * len(sh["exp_ij"]))
        for (i, j) in sh["exp_ij"][:n_show]:
            add_rect(ax2, sh, i, j, GREEN, alpha=0.55 * alpha)
        draw_shape(ax2, sh, alpha=0.75 * alpha)
        if t > 0.35:
            arr_a = min(1.0, (t - 0.35) / 0.25) * alpha
            rim = max(sh["solid_ij"], key=lambda ij: ij[1])
            x0 = sh["xmin"] + rim[1] * sh["dx"] + sh["dx"] * 0.5
            y0 = sh["ymin"] + rim[0] * sh["dy"] + sh["dy"] * 0.5
            x1 = x0 + D_CELLS * sh["dx"]
            ax2.annotate(
                "", xy=(x1, y0), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="->", color=YELLOW,
                                lw=1.8, mutation_scale=16),
                alpha=arr_a,
            )
            ax2.text((x0 + x1) / 2, y0 + sh["dy"] * 0.85, "d",
                     color=YELLOW, fontsize=10, ha="center",
                     alpha=arr_a, fontweight="bold")

    draw_panel_labels(phase, alpha=alpha)
    return []


anim = FuncAnimation(fig, animate, frames=TOTAL_F,
                     init_func=lambda: [], blit=False, repeat=True)

print(f"Rendering {TOTAL_F} frames ({len(SHAPES)} shapes x {PHASES} phases) -> {OUTPUT}")
sys.stdout.flush()
anim.save(OUTPUT, writer="pillow", fps=FPS, dpi=90)
print(f"Done  {os.path.getsize(OUTPUT) / 1024:.0f} KB  -> {OUTPUT}")
plt.close()
