#!/usr/bin/env python3
"""Generate animated GIF demonstrating the MeshExpander algorithm.

Output: docs/images/demo.gif
Frames: 3 phases × 30 frames each = 90 frames total, 15 fps → 6 s loop

Phase 1 — Original mesh (hollow cylinder cross-section)
Phase 2 — Solid voxelization (BFS flood-fill animation)
Phase 3 — Conservative expansion (d-margin growing outward)
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation

os.makedirs(os.path.join(os.path.dirname(__file__), "images"), exist_ok=True)
OUTPUT = os.path.join(os.path.dirname(__file__), "images", "demo.gif")

# ── colour palette (dark theme) ──────────────────────────────────────────────
BG      = "#0d1117"
FG      = "#c9d1d9"
BLUE    = "#58a6ff"
ORANGE  = "#f0883e"
GREEN   = "#3fb950"
RED     = "#f85149"
YELLOW  = "#e3b341"
GREY    = "#30363d"

# ── geometry ─────────────────────────────────────────────────────────────────
GRID_N  = 18   # voxel grid resolution
D_CELLS = 1.5  # expansion radius in grid cells


def make_hollow_circle(R_out=5.5, R_in=2.6, N=64):
    t = np.linspace(0, 2 * np.pi, N, endpoint=False)
    outer = np.c_[R_out * np.cos(t), R_out * np.sin(t)]
    inner = np.c_[R_in  * np.cos(t[::-1]), R_in * np.sin(t[::-1])]
    return outer, inner


def make_star(n=5, Ro=5.2, Ri=2.3):
    pts = []
    for i in range(n * 2):
        angle = np.pi / 2 + i * np.pi / n
        r = Ro if i % 2 == 0 else Ri
        pts.append([r * np.cos(angle), r * np.sin(angle)])
    return np.array(pts)


# ── voxelization ─────────────────────────────────────────────────────────────

def voxelize(outer, inner=None, n=GRID_N):
    """Return cells array (0=ext, 1=surface/solid, 2=hole interior)
    and grid origin/step."""
    all_pts = outer if inner is None else np.vstack([outer, inner])
    pad = 1.2
    xmin, ymin = all_pts.min(0) - pad
    xmax, ymax = all_pts.max(0) + pad
    dx = (xmax - xmin) / n
    dy = (ymax - ymin) / n

    outer_p = Path(np.vstack([outer, outer[0]]))
    inner_p = Path(np.vstack([inner, inner[0]])) if inner is not None else None

    cx = xmin + (np.arange(n) + 0.5) * dx
    cy = ymin + (np.arange(n) + 0.5) * dy
    CX, CY = np.meshgrid(cx, cy)
    pts = np.c_[CX.ravel(), CY.ravel()]

    in_out = outer_p.contains_points(pts).reshape(n, n)
    if inner_p is not None:
        in_in = inner_p.contains_points(pts).reshape(n, n)
        solid = in_out & ~in_in
        hole  = ~in_out & in_in  # (never happens for simple annulus)
    else:
        solid = in_out

    cells = np.zeros((n, n), dtype=int)
    cells[solid] = 1

    return cells, xmin, ymin, dx, dy


def bfs_order(cells):
    """BFS from exterior → returns list of (i,j) in fill order for interior."""
    n = cells.shape[0]
    visited = np.zeros((n, n), dtype=bool)
    # seed: all border cells that are exterior
    from collections import deque
    q = deque()
    for i in range(n):
        for j in [0, n-1]:
            if not cells[i, j] and not visited[i, j]:
                visited[i, j] = True; q.append((i, j))
    for j in range(n):
        for i in [0, n-1]:
            if not cells[i, j] and not visited[i, j]:
                visited[i, j] = True; q.append((i, j))

    ext_set = set()
    while q:
        i, j = q.popleft()
        ext_set.add((i, j))
        for di, dj in [(-1,0),(1,0),(0,-1),(0,1)]:
            ni, nj = i+di, j+dj
            if 0 <= ni < n and 0 <= nj < n and not visited[ni, nj] and not cells[ni, nj]:
                visited[ni, nj] = True; q.append((ni, nj))

    # interior empty cells (hole) in BFS-from-solid order
    interior = [(i,j) for i in range(n) for j in range(n)
                if not cells[i,j] and (i,j) not in ext_set]
    return interior


def expand_cells(cells, d=D_CELLS):
    n = cells.shape[0]
    pad = int(np.ceil(d))
    exp = np.zeros_like(cells)
    for i in range(n):
        for j in range(n):
            if cells[i, j]:
                for di in range(-pad, pad+1):
                    for dj in range(-pad, pad+1):
                        if di*di + dj*dj <= d*d:
                            ni, nj = i+di, j+dj
                            if 0 <= ni < n and 0 <= nj < n:
                                exp[ni, nj] = 1
    return exp


# ── pre-compute geometry ──────────────────────────────────────────────────────
outer, inner = make_hollow_circle()
cells, xmin, ymin, dx, dy = voxelize(outer, inner)
exp_cells = expand_cells(cells)

# Voxel cells in a fixed spiral-ish order for animation
solid_ij = [(i, j) for i in range(GRID_N) for j in range(GRID_N) if cells[i, j]]
np.random.seed(42)
np.random.shuffle(solid_ij)

# Expansion zone cells (not in original)
exp_ij = [(i, j) for i in range(GRID_N) for j in range(GRID_N)
          if exp_cells[i, j] and not cells[i, j]]
np.random.seed(7)
np.random.shuffle(exp_ij)

# ── figure setup ─────────────────────────────────────────────────────────────
PHASES   = 3
F_PHASE  = 30          # frames per phase
HOLD     = 8           # hold frames at end of each phase
TOTAL_F  = PHASES * (F_PHASE + HOLD)
FPS      = 15

fig, axes = plt.subplots(1, 3, figsize=(12, 4.2), facecolor=BG)
fig.subplots_adjust(left=0.02, right=0.98, top=0.88, bottom=0.05, wspace=0.06)

XLIM = (xmin - 0.3, xmin + GRID_N * dx + 0.3)
YLIM = (ymin - 0.3, ymin + GRID_N * dy + 0.3)

for ax in axes:
    ax.set_facecolor(BG)
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect("equal")
    ax.axis("off")


def cell_xy(i, j):
    return xmin + j * dx, ymin + i * dy


def add_rect(ax, i, j, color, alpha=0.85, edge=True):
    x, y = cell_xy(i, j)
    ec = "#ffffff18" if edge else "none"
    r = mpatches.Rectangle((x, y), dx * 0.96, dy * 0.96,
                            linewidth=0.4, edgecolor=ec,
                            facecolor=color, alpha=alpha)
    ax.add_patch(r)
    return r


def draw_shape(ax, alpha=1.0):
    """Draw hollow-cylinder outline."""
    ax.fill(*outer.T, color=BLUE, alpha=0.12 * alpha, zorder=1)
    ax.plot(*np.vstack([outer, outer[0]]).T,
            color=BLUE, lw=2, alpha=alpha, zorder=3)
    ax.fill(*inner.T, color=BG, alpha=1.0, zorder=2)
    ax.plot(*np.vstack([inner, inner[0]]).T,
            color=BLUE, lw=2, alpha=alpha, zorder=4)


def draw_grid(ax, alpha=0.12):
    for i in range(GRID_N + 1):
        ax.axhline(ymin + i * dy, color=FG, lw=0.3, alpha=alpha, zorder=0)
        ax.axvline(xmin + i * dx, color=FG, lw=0.3, alpha=alpha, zorder=0)


PHASE_TITLES = [
    ("① Input Mesh",         BLUE),
    ("② Solid Voxelization", ORANGE),
    ("③ Conservative Expansion ≥ d", GREEN),
]

SUBTITLE = [
    "any concave topology",
    "BFS flood-fill · top-down partitioner",
    "every input point covered by margin d",
]


def set_titles(phase_idx, active):
    for k, ax in enumerate(axes):
        title, color = PHASE_TITLES[k]
        c = color if k <= active else GREY
        ax.set_title(title, color=c, fontsize=10.5,
                     fontweight="bold", pad=6)
        sub_c = FG if k == active else GREY
        ax.text(0.5, -0.03, SUBTITLE[k], transform=ax.transAxes,
                ha="center", va="top", fontsize=7.5, color=sub_c)


def clear_axes():
    for ax in axes:
        ax.cla()
        ax.set_facecolor(BG)
        ax.set_xlim(*XLIM)
        ax.set_ylim(*YLIM)
        ax.set_aspect("equal")
        ax.axis("off")


# ── animation ─────────────────────────────────────────────────────────────────

def animate(frame):
    clear_axes()

    # global frame → phase + local t ∈ [0,1]
    period = F_PHASE + HOLD
    phase  = min(frame // period, PHASES - 1)
    local  = frame - phase * period
    t      = min(local / F_PHASE, 1.0)   # 0→1, clamped at 1 during hold

    # ── panel 0: always show shape (dim before phase 0) ──────────────────────
    mesh_alpha = min(1.0, t * 3) if phase == 0 else 1.0
    draw_shape(axes[0], alpha=mesh_alpha)
    draw_grid(axes[0], alpha=0.10 if phase == 0 else 0.06)

    # ── panel 1: voxelization ─────────────────────────────────────────────────
    if phase >= 1:
        draw_grid(axes[1], alpha=0.15)
        n_show = len(solid_ij) if phase > 1 else int(t * len(solid_ij))
        for (i, j) in solid_ij[:n_show]:
            add_rect(axes[1], i, j, ORANGE, alpha=0.85)

    # ── panel 2: expansion ────────────────────────────────────────────────────
    if phase >= 2:
        # base: original voxels
        for (i, j) in solid_ij:
            add_rect(axes[2], i, j, ORANGE, alpha=0.85)
        # expansion zone growing
        n_show = int(t * len(exp_ij))
        for (i, j) in exp_ij[:n_show]:
            add_rect(axes[2], i, j, GREEN, alpha=0.55)
        # overlay original shape
        draw_shape(axes[2], alpha=0.7)
        # draw "d" arrow (once expansion reaches ~40%)
        if t > 0.4:
            arrow_alpha = min(1.0, (t - 0.4) / 0.2)
            # find a rightmost solid cell
            rim_i = max(solid_ij, key=lambda ij: ij[1])
            x0, y0 = cell_xy(*rim_i)
            x0 += dx * 0.5;  y0 += dy * 0.5
            x1 = xmin + (rim_i[1] + D_CELLS) * dx + dx * 0.5
            axes[2].annotate(
                "", xy=(x1, y0), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="->", color=YELLOW,
                                lw=1.6, mutation_scale=14),
                alpha=arrow_alpha
            )
            axes[2].text((x0 + x1) / 2, y0 + dy * 0.6, "d",
                         color=YELLOW, fontsize=9, ha="center",
                         alpha=arrow_alpha, fontweight="bold")

    set_titles(frame // (F_PHASE + HOLD), phase)

    return []


anim = FuncAnimation(fig, animate, frames=TOTAL_F,
                     init_func=lambda: [], blit=False, repeat=True)

print(f"Saving {TOTAL_F} frames to {OUTPUT} …")
anim.save(OUTPUT, writer="pillow", fps=FPS, dpi=90)
print(f"Done → {OUTPUT}")
plt.close()
