#!/usr/bin/env python3
"""Generate rotating 3D animation GIF for MeshExpander README.

Shows original mesh (blue semi-transparent) + expanded result (green semi-transparent).
Three shapes side by side: HollowCylinder / GearTooth / StarPrism — all rotating.

Requirements: matplotlib, numpy, pillow
Output: docs/images/rotate3d.gif
"""

import os
import sys
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation

OUT_DIR = os.path.join(os.path.dirname(__file__), "images")
STL_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__),
          "../build/stl_output"))
OUTPUT  = os.path.join(OUT_DIR, "rotate3d.gif")
os.makedirs(OUT_DIR, exist_ok=True)

# ── STL reader ────────────────────────────────────────────────────────────────

def read_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        tris = []
        for _ in range(n):
            f.read(12)
            v = [struct.unpack("<fff", f.read(12)) for _ in range(3)]
            tris.append(v)
            f.read(2)
    tris = np.array(tris, dtype=np.float32)  # (M, 3, 3)
    return tris

# ── procedural mesh generators ────────────────────────────────────────────────

def make_hollow_cylinder(Ro=0.060, Ri=0.040, H=0.100, N=32):
    verts, faces = [], []
    def ring(r, z):
        b = len(verts)
        for k in range(N):
            a = 2*np.pi*k/N
            verts.append([r*np.cos(a), r*np.sin(a), z])
        return b
    ot=ring(Ro,H/2); ob=ring(Ro,-H/2); it_=ring(Ri,H/2); ib=ring(Ri,-H/2)
    for k in range(N):
        n=(k+1)%N
        faces += [[ot+k,ob+k,ob+n],[ot+k,ob+n,ot+n]]   # outer
        faces += [[it_+k,ib+n,ib+k],[it_+k,it_+n,ib+n]] # inner (rev)
        faces += [[ot+k,it_+n,it_+k],[ot+k,ot+n,it_+n]] # top
        faces += [[ob+k,ib+k,ib+n],[ob+k,ib+n,ob+n]]    # bot
    v = np.array(verts,np.float32); f = np.array(faces,np.int32)
    return v[f]   # (M,3,3)


def make_gear(Rb=0.060, nt=12, th=0.020, H=0.040):
    verts = []
    N2 = nt*2
    for k in range(N2):
        a = 2*np.pi*k/N2
        r = Rb+th if k%2==0 else Rb
        verts += [[r*np.cos(a), r*np.sin(a),  H/2],
                  [r*np.cos(a), r*np.sin(a), -H/2]]
    faces = []
    for k in range(N2):
        n=(k+1)%N2
        a,b,c,d = 2*k, 2*k+1, 2*n, 2*n+1
        faces += [[a,b,d],[a,d,c]]
    # top/bot fan
    ct=len(verts); verts.append([0,0, H/2])
    cb=len(verts); verts.append([0,0,-H/2])
    for k in range(N2):
        n=(k+1)%N2
        faces += [[ct,2*n,2*k],[cb,2*k+1,2*n+1]]
    v=np.array(verts,np.float32); f=np.array(faces,np.int32)
    return v[f]


def make_star_prism(npt=5, Ro=0.060, Ri=0.025, H=0.050):
    pts2d=[]
    for i in range(npt*2):
        a=np.pi/2+i*np.pi/npt; r=Ro if i%2==0 else Ri
        pts2d.append([r*np.cos(a), r*np.sin(a)])
    pts2d=np.array(pts2d); N2=len(pts2d)
    verts=[[p[0],p[1], H/2] for p in pts2d]+[[p[0],p[1],-H/2] for p in pts2d]
    faces=[]
    for k in range(N2):
        n=(k+1)%N2
        faces+=[[k,N2+k,N2+n],[k,N2+n,n]]
    ct=len(verts); verts.append([0,0, H/2])
    cb=len(verts); verts.append([0,0,-H/2])
    for k in range(N2):
        n=(k+1)%N2
        faces+=[[ct,n,k],[cb,k+N2,n+N2]]
    v=np.array(verts,np.float32); f=np.array(faces,np.int32)
    return v[f]

# ── lighting (Phong-ish, per triangle) ───────────────────────────────────────

LIGHT = np.array([0.6, -0.3, 1.0], np.float32)
LIGHT /= np.linalg.norm(LIGHT)

def shade(tris, base_rgb, ambient=0.30):
    """Return per-triangle RGBA array with simple diffuse shading."""
    e1 = tris[:,1] - tris[:,0]
    e2 = tris[:,2] - tris[:,0]
    n  = np.cross(e1, e2)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln = np.where(ln < 1e-12, 1.0, ln)
    n  = n / ln
    diff = np.abs(n.dot(LIGHT))          # abs: lit from both sides
    intensity = ambient + (1-ambient)*diff
    r,g,b = base_rgb
    return np.clip(np.stack([intensity*r, intensity*g, intensity*b], axis=1), 0, 1)

# ── shape records ─────────────────────────────────────────────────────────────

ORIG_RGB = (0.34, 0.64, 1.00)   # blue — all input meshes
EXP_RGB  = (0.25, 0.73, 0.31)   # green — all expanded results

SHAPES = [
    dict(name="Hollow Cylinder",  tris=make_hollow_cylinder(),
         exp_stl="cad_Hollow_cylinder_robust.stl",
         orig_rgb=ORIG_RGB, exp_rgb=EXP_RGB),
    dict(name="Gear (12 teeth)",  tris=make_gear(),
         exp_stl="cad_Gear_12-tooth_robust.stl",
         orig_rgb=ORIG_RGB, exp_rgb=EXP_RGB),
    dict(name="Star Prism",       tris=make_star_prism(),
         exp_stl="cad_Star_prism_5pt_robust.stl",
         orig_rgb=ORIG_RGB, exp_rgb=EXP_RGB),
]

for sh in SHAPES:
    sh["exp_tris"] = read_stl(os.path.join(STL_DIR, sh["exp_stl"]))
    sh["orig_colors"] = shade(sh["tris"],    sh["orig_rgb"])
    sh["exp_colors"]  = shade(sh["exp_tris"], sh["exp_rgb"])

# ── figure ────────────────────────────────────────────────────────────────────

BG     = "#0d1117"
FPS    = 20
FRAMES = 72   # 360° / 5° per frame
ELEV   = 32.0   # higher angle → hollow interior visible

fig = plt.figure(figsize=(12, 4.5), facecolor=BG)
fig.subplots_adjust(left=0.01, right=0.99, top=0.91, bottom=0.01,
                    wspace=0.02)

axes = [fig.add_subplot(1, 3, k+1, projection="3d") for k in range(3)]

# static top title
fig.text(0.5, 0.96,
         "MeshExpander — Conservative 3D Mesh Expansion",
         ha="center", va="top", color="#e6edf3",
         fontsize=12, fontweight="bold",
         path_effects=[pe.withStroke(linewidth=3, foreground=BG)])

LABEL_COLORS = ["#58a6ff", "#f0883e", "#e3b341"]

def setup_ax(ax, sh, color):
    ax.set_facecolor(BG)
    ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("none")
    ax.yaxis.pane.set_edgecolor("none")
    ax.zaxis.pane.set_edgecolor("none")
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    ax.set_xlabel(""); ax.set_ylabel(""); ax.set_zlabel("")
    ax.grid(False)
    # set axis limits
    all_v = np.vstack([sh["tris"].reshape(-1,3), sh["exp_tris"].reshape(-1,3)])
    mn = all_v.min(axis=0); mx = all_v.max(axis=0)
    cen = (mn+mx)/2; r = (mx-mn).max()/2 * 0.60
    ax.set_xlim(cen[0]-r, cen[0]+r)
    ax.set_ylim(cen[1]-r, cen[1]+r)
    ax.set_zlim(cen[2]-r, cen[2]+r)
    ax.set_box_aspect([1, 1, 1])   # equal scale in all axes
    ax.set_title(sh["name"], color=color, fontsize=10.5,
                 fontweight="bold", pad=4)


def render_shape(ax, sh, azim):
    orig = sh["tris"]
    expt = sh["exp_tris"]
    oc   = sh["orig_colors"]
    ec   = sh["exp_colors"]

    # depth-sort expanded (painter's algorithm, back→front)
    cen_exp = expt.mean(axis=1)   # (M,3)
    azr = np.radians(azim); elvr = np.radians(ELEV)
    eye = np.array([np.cos(elvr)*np.cos(azr),
                    np.cos(elvr)*np.sin(azr),
                    np.sin(elvr)], np.float32)
    depth_exp = cen_exp.dot(eye)
    idx_exp   = np.argsort(depth_exp)  # back→front

    depth_orig = orig.mean(axis=1).dot(eye)
    idx_orig   = np.argsort(depth_orig)

    # expanded (green, semi-transparent)
    p_exp = Poly3DCollection(expt[idx_exp],
                             facecolors=np.hstack([ec[idx_exp],
                                                   np.full((len(idx_exp),1),0.28)]),
                             edgecolors="none",
                             zsort="average")
    ax.add_collection3d(p_exp)

    # original (blue, semi-transparent)
    p_orig = Poly3DCollection(orig[idx_orig],
                              facecolors=np.hstack([oc[idx_orig],
                                                    np.full((len(idx_orig),1),0.55)]),
                              edgecolors=np.array(sh["orig_rgb"]+(0.25,)).reshape(1,4),
                              linewidths=0.2,
                              zsort="average")
    ax.add_collection3d(p_orig)


def animate(frame):
    azim = frame * 360.0 / FRAMES
    for ax, sh, lc in zip(axes, SHAPES, LABEL_COLORS):
        ax.cla()
        setup_ax(ax, sh, lc)
        ax.view_init(elev=ELEV, azim=azim)
        render_shape(ax, sh, azim)
    return []


print(f"Rendering {FRAMES} frames …")
sys.stdout.flush()

anim = FuncAnimation(fig, animate, frames=FRAMES,
                     init_func=lambda: [], blit=False, repeat=True)
anim.save(OUTPUT, writer="pillow", fps=FPS, dpi=90)
kb = os.path.getsize(OUTPUT) / 1024
print(f"Done  {kb:.0f} KB  → {OUTPUT}")
plt.close()
