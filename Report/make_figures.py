#!/usr/bin/env python3
"""
Generate every figure for the IEEE report as vector PDF.

    python make_figures.py            write figures/*.pdf
    python make_figures.py --preview  also write preview/*.png for eyeballing

Column widths follow IEEEtran: single = 3.5 in, double = 7.16 in.
Fonts are embedded as Type 42 (TrueType); IEEE rejects Type 3.
"""

import os
import re
import sys
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle, Wedge

plt.rcParams.update({
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "DejaVu Serif"],
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.6,
    "ytick.major.width": 0.6,
    "lines.linewidth": 0.9,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

COL1, COL2 = 3.5, 7.16
HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, "figures")
PREV = os.path.join(HERE, "preview")
LOGDIR = os.path.normpath(os.path.join(
    HERE, "..", "Code", "Rotor_Rate_Controller_Package", "logs"))
os.makedirs(FIGS, exist_ok=True)

PREVIEW = "--preview" in sys.argv
if PREVIEW:
    os.makedirs(PREV, exist_ok=True)

INK = "#1a1a1a"
GREY = "#777777"
PALE = "#cccccc"
FILL = "#eceff1"
FILL2 = "#d8e3e4"
ACC = "#14595c"
RED = "#a8382b"
GRN = "#2f6b45"
BLU = "#2d4f8f"


def save(fig, name):
    fig.savefig(os.path.join(FIGS, name + ".pdf"))
    if PREVIEW:
        fig.savefig(os.path.join(PREV, name + ".png"), dpi=190)
    plt.close(fig)
    print("wrote figures/%s.pdf" % name)


def box(ax, x, y, w, h, label, sub=None, fc=FILL, ec=INK, fs=7.0, lw=0.7,
        subfs=6.2, radius=0.012, gap=0.004, spacing=1.45):
    """Draw a rounded box with a title and optional multi-line sub-caption.

    The title and the sub-caption are laid out as one block that is centred
    vertically in the box, so the text can never spill past the border no
    matter how many lines `sub` has. Line heights are computed from the
    figure height, since these axes are in 0..1 figure-relative units.
    """
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0,rounding_size=%g" % radius,
        fc=fc, ec=ec, lw=lw, zorder=2))
    cx, cy = x + w / 2, y + h / 2

    if not sub:
        ax.text(cx, cy, label, ha="center", va="center", fontsize=fs,
                zorder=3, color=INK)
        return

    fig_h_in = ax.figure.get_size_inches()[1]
    lh_label = fs / 72.0 / fig_h_in * spacing
    lh_sub = subfs / 72.0 / fig_h_in * spacing
    n_sub = sub.count("\n") + 1

    total = lh_label + gap + n_sub * lh_sub
    top = cy + total / 2.0

    ax.text(cx, top, label, ha="center", va="top", fontsize=fs, zorder=3,
            color=INK)
    ax.text(cx, top - lh_label - gap, sub, ha="center", va="top",
            fontsize=subfs, color=GREY, zorder=3, linespacing=spacing)


def arrow(ax, p0, p1, color=INK, ls="-", lw=0.7):
    ax.add_patch(FancyArrowPatch(
        p0, p1, arrowstyle="-|>", mutation_scale=6, lw=lw, color=color,
        linestyle=ls, shrinkA=0, shrinkB=0, zorder=1))


def vlabel(ax, x, y0, y1, text, fs=6.0, color=GREY, side=0.008):
    """Label for a vertical arrow, placed to its right, vertically centred."""
    ax.text(x + side, (y0 + y1) / 2, text, fontsize=fs, ha="left",
            va="center", color=color, zorder=4)


def blank(w, h):
    fig = plt.figure(figsize=(w, h))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    return fig, ax


# ==========================================================================
# Fig. 1  -- wiring / interconnect
# ==========================================================================

def fig_wiring():
    fig, ax = blank(COL2, 3.35)

    # ---- power -----------------------------------------------------------
    box(ax, 0.02, 0.845, 0.155, 0.10, "4S LiPo", "14.8 V", fc=FILL2, gap=0.004)
    box(ax, 0.02, 0.665, 0.155, 0.10, "BEC 5 V", "from ESC", fc=FILL2,
        gap=0.004)
    arrow(ax, (0.0975, 0.845), (0.0975, 0.770))

    # ---- ESC and motors --------------------------------------------------
    box(ax, 0.295, 0.775, 0.225, 0.17, "4-in-1 ESC, 45 A",
        "BLHeli-S / Bluejay\nbidirectional DShot300", fc=FILL2, subfs=6.0)
    arrow(ax, (0.175, 0.895), (0.290, 0.895))
    ax.text(0.198, 0.903, "14.8 V", fontsize=6.0, color=GREY)

    motors = [("M1", "front-left", "CCW"), ("M2", "front-right", "CW"),
              ("M3", "rear-right", "CCW"), ("M4", "rear-left", "CW")]
    for i, (m, place, spin) in enumerate(motors):
        y = 0.920 - i * 0.043
        ax.plot([0.520, 0.552], [y, y], color=INK, lw=0.6)
        ax.add_patch(Circle((0.560, y), 0.0085, fc=FILL, ec=INK, lw=0.6))
        ax.text(0.576, y, "%s  %-11s %s" % (m, place, spin), fontsize=6.0,
                family="monospace", va="center", color=INK)
    ax.text(0.576, 0.735, "3-phase BLDC, 14 poles", fontsize=6.0, color=GREY)

    # ---- MCU -------------------------------------------------------------
    box(ax, 0.295, 0.115, 0.225, 0.545, "", None, fc="#ffffff")
    ax.text(0.4075, 0.615, "ESP32-WROOM-32", fontsize=7.4, ha="center")
    ax.text(0.4075, 0.585, "240 MHz dual core", fontsize=6.0, ha="center",
            color=GREY)
    ax.text(0.4075, 0.563, "control loop on core 1", fontsize=6.0,
            ha="center", color=GREY)
    ax.text(0.4075, 0.541, "flow task on core 0", fontsize=6.0, ha="center",
            color=GREY)

    ax.plot([0.310, 0.505], [0.512, 0.512], color=PALE, lw=0.5)
    for i, (pin, m) in enumerate([("GPIO26", "M1"), ("GPIO14", "M2"),
                                  ("GPIO27", "M3"), ("GPIO25", "M4")]):
        ax.text(0.312, 0.480 - i * 0.030, "%s  %s" % (pin, m), fontsize=5.8,
                family="monospace", color=ACC, va="center")
    ax.text(0.312, 0.352, "GPIO22  SDA", fontsize=5.8, family="monospace",
            color=INK, va="center")
    ax.text(0.312, 0.322, "GPIO21  SCL", fontsize=5.8, family="monospace",
            color=INK, va="center")
    ax.text(0.312, 0.270, "GPIO18/19/23", fontsize=5.8, family="monospace",
            color=GREY, va="center")
    ax.text(0.312, 0.240, "GPIO13/33", fontsize=5.8, family="monospace",
            color=GREY, va="center")

    arrow(ax, (0.175, 0.715), (0.290, 0.560))
    ax.text(0.192, 0.660, "5 V", fontsize=6.0, color=GREY)

    # DShot bundle up to the ESC
    arrow(ax, (0.4075, 0.660), (0.4075, 0.770), color=ACC, lw=1.0)
    ax.text(0.416, 0.715, "4 x DShot300 + eRPM return", fontsize=6.0,
            color=ACC, va="center")

    # ---- I2C bus ---------------------------------------------------------
    bx0, bx1 = 0.600, 0.622
    ax.plot([bx0, bx0], [0.270, 0.600], color=INK, lw=1.0)
    ax.plot([bx1, bx1], [0.270, 0.600], color=INK, lw=1.0)
    arrow(ax, (0.520, 0.352), (bx0 - 0.004, 0.352))
    arrow(ax, (0.520, 0.322), (bx1 - 0.004, 0.322))
    ax.text(0.611, 0.244, "I$^2$C  400 kHz", fontsize=6.2, ha="center",
            color=GREY)

    i2c = [("MPU-6050", "0x68   6-axis IMU\nmounted inverted", 0.475),
           ("VL53L0X", "0x29   ToF rangefinder\nfacing down", 0.345),
           ("QMC5883P", "0x2C   magnetometer\nmounted reversed", 0.215)]
    for nm, sub, y in i2c:
        box(ax, 0.680, y, 0.300, 0.125, nm, sub, subfs=5.8, gap=0.004)
        ax.plot([bx1, 0.680], [y + 0.0625, y + 0.0625], color=INK, lw=0.7)

    # ---- SPI, routed clear of the I2C bus --------------------------------
    box(ax, 0.680, 0.050, 0.300, 0.125, "PMW3901",
        "optical flow, SPI 2 MHz\ndedicated FreeRTOS task", subfs=5.8,
        gap=0.004)
    ax.plot([0.520, 0.680], [0.1125, 0.1125], color=INK, lw=0.7)
    ax.text(0.528, 0.100, "SPI 2 MHz\nSCK18 MISO19 MOSI23\nCS13  RST33",
            fontsize=5.6, family="monospace", color=GREY, va="top",
            linespacing=1.4)

    # ---- radio -----------------------------------------------------------
    box(ax, 0.02, 0.135, 0.175, 0.225, "Transmitter",
        "ESP32 + 2 gimbals\n3 analogue axes\n+ yaw pushbuttons\nNVS calibration",
        fc=FILL2, subfs=5.8, gap=0.004)
    arrow(ax, (0.195, 0.248), (0.290, 0.248), color=ACC)
    ax.text(0.200, 0.258, "ESP-NOW, 50 Hz", fontsize=6.0, color=ACC)

    ax.text(0.02, 0.020, "Ground and the 3.3 V peripheral rail are common to "
                         "every device and are not drawn.", fontsize=6.0,
            color=GREY)
    save(fig, "fig_wiring")


# ==========================================================================
# Fig. 2  -- physical body / rotor layout and control effectiveness
# ==========================================================================

def fig_body():
    fig = plt.figure(figsize=(COL2, 2.75))
    axL = fig.add_axes([0.015, 0.04, 0.43, 0.93])
    axR = fig.add_axes([0.520, 0.04, 0.47, 0.93])

    ax = axL
    ax.set_xlim(-0.150, 0.150)
    ax.set_ylim(-0.140, 0.150)
    ax.set_aspect("equal")
    ax.axis("off")

    ax_arm, ay_arm = 0.0815, 0.08225
    pos = [(-ax_arm, +ay_arm), (+ax_arm, +ay_arm),
           (+ax_arm, -ay_arm), (-ax_arm, -ay_arm)]
    spin = [+1, -1, +1, -1]
    lbl = ["M1", "M2", "M3", "M4"]

    ax.plot([pos[0][0], pos[2][0]], [pos[0][1], pos[2][1]], color=INK, lw=1.4)
    ax.plot([pos[1][0], pos[3][0]], [pos[1][1], pos[3][1]], color=INK, lw=1.4)

    for (px, py), s, nm in zip(pos, spin, lbl):
        ax.add_patch(Circle((px, py), 0.0355, fc=FILL, ec=INK, lw=0.8,
                            zorder=3))
        ax.text(px, py + 0.007, nm, ha="center", va="center", fontsize=7.5,
                zorder=4)
        ax.text(px, py - 0.011, "CCW" if s > 0 else "CW", ha="center",
                va="center", fontsize=6.0, color=GREY, zorder=4)
        th = np.linspace(0.40, 2.0 * math.pi - 0.40, 60)[::s]
        r = 0.0250
        ax.plot(px + r * np.cos(th), py + r * np.sin(th), color=ACC, lw=0.7,
                zorder=4)
        ax.add_patch(FancyArrowPatch(
            (px + r * np.cos(th[-3]), py + r * np.sin(th[-3])),
            (px + r * np.cos(th[-1]), py + r * np.sin(th[-1])),
            arrowstyle="-|>", mutation_scale=5, color=ACC, lw=0.7, zorder=5))

    ax.add_patch(FancyArrowPatch((0, 0), (0.052, 0), arrowstyle="-|>",
                                 mutation_scale=7, color=RED, lw=1.1, zorder=6))
    ax.text(0.056, 0.000, "+X right", fontsize=6.8, color=RED, va="center")
    ax.add_patch(FancyArrowPatch((0, 0), (0, 0.046), arrowstyle="-|>",
                                 mutation_scale=7, color=GRN, lw=1.1, zorder=6))
    ax.text(0.004, 0.050, "+Y fwd", fontsize=6.8, color=GRN)
    ax.add_patch(Circle((0, 0), 0.0072, fc="white", ec=BLU, lw=1.0, zorder=6))
    ax.add_patch(Circle((0, 0), 0.0019, fc=BLU, ec=BLU, zorder=7))
    ax.text(-0.013, -0.010, "+Z up", fontsize=6.8, color=BLU, ha="right",
            va="center")

    ax.annotate("", xy=(-ax_arm, -0.122), xytext=(ax_arm, -0.122),
                arrowprops=dict(arrowstyle="<->", lw=0.6, color=GREY))
    ax.text(0, -0.119, "0.1630 m", fontsize=6.0, ha="center", color=GREY)
    ax.annotate("", xy=(0.126, -ay_arm), xytext=(0.126, ay_arm),
                arrowprops=dict(arrowstyle="<->", lw=0.6, color=GREY))
    ax.text(0.131, 0, "0.1645 m", fontsize=6.0, va="center", color=GREY,
            rotation=90)
    ax.text(0, 0.138, "FRONT", fontsize=6.5, ha="center", color=GREY)
    ax.text(0, -0.138, "REAR", fontsize=6.5, ha="center", color=GREY)
    ax.text(-0.148, 0.140, "(a)", fontsize=8, weight="bold")

    # ---- (b) control effectiveness ---------------------------------------
    ax = axR
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.0, 0.955, "(b)", fontsize=8, weight="bold")

    rows = [
        ("Thrust  $T$", [+1, +1, +1, +1], r"$k_F\,\Sigma u_i$"),
        ("Pitch  $\\tau_x$", [+1, +1, -1, -1],
         r"$k_F a_y(u_1{+}u_2{-}u_3{-}u_4)$"),
        ("Roll  $\\tau_y$", [+1, -1, -1, +1],
         r"$k_F a_x(u_1{-}u_2{-}u_3{+}u_4)$"),
        ("Yaw  $\\tau_z$", [-1, +1, -1, +1],
         r"$-k_M(u_1{-}u_2{+}u_3{-}u_4)$"),
    ]
    x0, dx = 0.265, 0.082
    ax.text(0.0, 0.865, "axis", fontsize=6.5, color=GREY)
    for j, nm in enumerate(lbl):
        ax.text(x0 + j * dx, 0.865, nm, fontsize=6.5, ha="center", color=GREY)
    ax.text(0.615, 0.865, "produced by", fontsize=6.5, color=GREY)
    ax.plot([0, 1.0], [0.845, 0.845], color=INK, lw=0.8)

    for i, (nm, pat, expr) in enumerate(rows):
        y = 0.775 - i * 0.105
        ax.text(0.0, y, nm, fontsize=7.0, va="center")
        for j, s in enumerate(pat):
            c = ACC if s > 0 else "#ffffff"
            ax.add_patch(Circle((x0 + j * dx, y), 0.027, fc=c, ec=INK,
                                lw=0.6, zorder=3))
            ax.text(x0 + j * dx, y, "+" if s > 0 else "−", ha="center",
                    va="center", fontsize=6.5, zorder=4,
                    color="white" if s > 0 else INK)
        ax.text(0.615, y, expr, fontsize=6.3, va="center")
        ax.plot([0, 1.0], [y - 0.052, y - 0.052], color="#e2e2e2", lw=0.4)

    ax.plot([0, 1.0], [0.318, 0.318], color=INK, lw=0.6)
    ax.text(0.0, 0.255,
            "Filled = that rotor speeds up.  $u_i=\\omega_i^2$;  "
            "$a_x=0.0815$ m, $a_y=0.08225$ m.\n"
            "A CCW rotor is driven against aerodynamic drag, so the airframe "
            "receives\nthe opposite reaction. Yaw is therefore the only row "
            "set by $k_M$ instead of\n$k_F$, and $k_M/k_F\\approx0.01$ m -- an "
            "eightfold authority deficit.\n\n"
            "Sensor lever arms on the $+Y$ centreline: PMW3901 at $+0.040$ m, "
            "VL53L0X\nat $+0.070$ m from the airframe centre.",
            fontsize=6.3, va="top", color=INK, linespacing=1.55)

    save(fig, "fig_body")


# ==========================================================================
# Fig. 3  -- control cascade
# ==========================================================================

def fig_cascade():
    fig, ax = blank(COL2, 3.5)

    XL, XM, XR = 0.045, 0.385, 0.725          # column lefts
    CL, CM, CR = 0.1775, 0.5175, 0.8575       # column centres
    W = 0.265
    H = 0.105                                  # box height, single-line sub

    y_est, h_est = 0.938, 0.055
    y_out = 0.783
    y_att = 0.603
    y_rat = 0.423
    y_wre, h_wre = 0.308, 0.055
    y_all, h_all = 0.183, 0.080
    y_rot, h_rot = 0.063, 0.080

    box(ax, 0.045, y_est, 0.945, h_est,
        "STATE ESTIMATE      roll, pitch, yaw   |   $v_{body}$, $v_z$   |   "
        "$x$, $y$ (nav), altitude", fc=FILL2, fs=6.9)

    for x in (CL, CM, CR):
        arrow(ax, (x, y_est), (x, y_out + H))

    box(ax, XL, y_out, W, H, "XY HOLD",
        "$p\\rightarrow v\\rightarrow a\\rightarrow$ tilt;  $K_p$ 1.0 then 2.0",
        fs=6.9, subfs=6.0, gap=0.004)
    box(ax, XM, y_out, W, H, "ALTITUDE",
        "$h\\rightarrow v_z\\rightarrow a\\rightarrow T$;  $K_p$ 1.5 then 4.0",
        fs=6.9, subfs=6.0, gap=0.004)
    box(ax, XR, y_out, W, H, "HEADING",
        "$\\psi_e\\rightarrow\\omega_z^{cmd}$;  $K_p$ 3.0, $\\leq$35 $^\\circ$/s",
        fs=6.9, subfs=6.0, gap=0.004)

    arrow(ax, (CL, y_out), (CL, y_att + H))
    vlabel(ax, CL, y_out, y_att + H, "tilt cmd, $\\leq$12$^\\circ$")

    arrow(ax, (CR, y_out), (CR, y_rat + H))
    vlabel(ax, CR, y_out, y_rat + H, "rate target")

    arrow(ax, (CM, y_out), (CM, y_wre + h_wre), color=ACC, lw=0.9)
    vlabel(ax, CM, y_out, y_wre + h_wre, "$T$  [N]", color=ACC, fs=6.4)

    box(ax, XL, y_att, W, H, "ATTITUDE",
        "$\\omega^{cmd}=10\\,(\\phi_d-\\phi)$, clamped $\\pm$200 $^\\circ$/s",
        fs=6.9, subfs=6.0, gap=0.004)
    arrow(ax, (CL, y_att), (CL, y_rat + H))
    vlabel(ax, CL, y_att, y_rat + H, "rate target")

    box(ax, XL, y_rat, W, H, "RATE PID   roll, pitch",
        "35 / 8 / 0.40 $\\rightarrow\\alpha$;  "
        "$\\tau_y{=}I_{yy}\\alpha$, $\\tau_x{=}I_{xx}\\alpha$",
        fs=6.8, subfs=5.9, gap=0.004)
    box(ax, XR, y_rat, W, H, "RATE PID   yaw",
        "15 / 4 / 0.20 $\\rightarrow\\alpha$;  "
        "$\\tau_z{=}I_{zz}\\alpha$, then clamped",
        fs=6.8, subfs=5.9, gap=0.004)

    arrow(ax, (CL, y_rat), (CL, y_wre + h_wre))
    vlabel(ax, CL, y_rat, y_wre + h_wre, "$\\tau_x,\\tau_y$", fs=6.4)
    arrow(ax, (CR, y_rat), (CR, y_wre + h_wre))
    vlabel(ax, CR, y_rat, y_wre + h_wre, "$\\tau_z$", fs=6.4)

    box(ax, 0.045, y_wre, 0.945, h_wre,
        "BODY WRENCH      $T$,   $\\tau_x$ nose-up,   "
        "$\\tau_y$ right-wing-down,   $\\tau_z$ counter-clockwise",
        fc=FILL2, fs=6.9)
    arrow(ax, (CM, y_wre), (CM, y_all + h_all))

    box(ax, 0.235, y_all, 0.565, h_all, "ROTOR ALLOCATOR",
        "geometry inverse; priority roll/pitch $\\rightarrow$ thrust "
        "$\\rightarrow$ yaw", fs=6.9, subfs=6.0, gap=0.004)
    arrow(ax, (CM, y_all), (CM, y_rot + h_rot))
    ax.text(CM + 0.008, (y_all + y_rot + h_rot) / 2, "$\\omega_{1..4}$",
            fontsize=6.4, color=GREY, va="center")

    box(ax, 0.235, y_rot, 0.565, h_rot,
        "ROTOR SPEED LOOP  $\\rightarrow$  ESC",
        "feedforward + PI closed on measured RPM, DShot 48-750", fs=6.9,
        subfs=6.0, gap=0.004)

    yfb = y_rot + h_rot / 2
    for p0, p1 in (((0.800, yfb), (0.845, yfb)),
                   ((0.845, yfb), (0.845, y_rot + h_rot))):
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-", lw=0.6,
                                     color=GREY, linestyle="--"))
    ax.text(0.852, yfb, "measured RPM", fontsize=6.0, color=GREY, va="center")

    ytop = y_est + h_est / 2
    for p0, p1 in (((0.235, yfb), (0.020, yfb)), ((0.020, yfb), (0.020, ytop))):
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-", lw=0.6,
                                     color=GREY, linestyle="--"))
    ax.add_patch(FancyArrowPatch((0.020, ytop), (0.040, ytop),
                                 arrowstyle="-|>", mutation_scale=6, lw=0.6,
                                 color=GREY, linestyle="--"))
    ax.text(0.030, 0.50, "airframe $\\rightarrow$ sensors $\\rightarrow$ "
                         "estimator", fontsize=6.0, color=GREY, rotation=90,
            va="center", ha="center")

    save(fig, "fig_cascade")


# ==========================================================================
# Fig. 4  -- allocation saturation priority
# ==========================================================================

def fig_alloc():
    fig = plt.figure(figsize=(COL1, 2.25))
    ax = fig.add_axes([0.155, 0.155, 0.80, 0.80])

    uMin, uMax = 62500.0, 2560000.0
    hover = 950096.0
    ax.set_ylim(-40000, uMax + 120000)
    ax.set_xlim(-0.62, 2.72)
    ax.set_yticks([uMin, hover, uMax])
    ax.set_yticklabels(["$u_{min}$\n250 rad/s", "hover\n975 rad/s",
                        "$u_{max}$\n1600 rad/s"], fontsize=6.0)
    ax.set_ylabel("rotor $u=\\omega^2$", fontsize=7)
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(["1. slide the\ncollective", "2. scale\nroll+pitch",
                        "3. fit yaw in\nwhat is left"], fontsize=6.4)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.axhline(uMin, color=GREY, lw=0.6, ls=":")
    ax.axhline(uMax, color=GREY, lw=0.6, ls=":")

    w = 0.42
    ch = 70000.0

    def draw(xi, coll, rp, yaw=0.0):
        ax.add_patch(Rectangle((xi - w / 2, coll + ch), w, rp, fc=ACC, ec=INK,
                               lw=0.6, alpha=0.5, zorder=3))
        ax.add_patch(Rectangle((xi - w / 2, coll - ch - rp), w, rp, fc=ACC,
                               ec=INK, lw=0.6, alpha=0.5, zorder=3))
        ax.add_patch(Rectangle((xi - w / 2, coll - ch), w, 2 * ch, fc="white",
                               ec=INK, lw=0.8, zorder=4))
        if yaw:
            ax.add_patch(Rectangle((xi - w / 2, coll + ch + rp), w, yaw,
                                   fc="none", ec=RED, lw=0.8, ls="--",
                                   zorder=5))
            ax.add_patch(Rectangle((xi - w / 2, coll - ch - rp - yaw), w, yaw,
                                   fc="none", ec=RED, lw=0.8, ls="--",
                                   zorder=5))

    # step 1: requested collective too high, slides down so the spread fits
    requested = hover + 330000
    ax.add_patch(Rectangle((-w / 2, requested - ch), w, 2 * ch, fc="none",
                           ec=PALE, lw=0.7, ls="--", zorder=2))
    ax.annotate("", xy=(0, hover + ch + 40000),
                xytext=(0, requested - ch - 40000),
                arrowprops=dict(arrowstyle="-|>", lw=0.7, color=INK))
    draw(0, hover, 810000)
    ax.text(-0.60, requested + 130000, "requested", fontsize=5.8, color=GREY)

    draw(1, hover, 540000)
    draw(2, hover, 540000, 230000)

    ax.text(0.245, hover + 500000, "roll/pitch\nspread", fontsize=6.0,
            color=INK, va="center", ha="left")
    ax.text(2.245, hover + 830000, "yaw", fontsize=6.0, color=RED,
            va="center")
    ax.text(-0.60, hover - 195000, "collective", fontsize=6.0, color=GREY)

    save(fig, "fig_alloc")


# ==========================================================================
# Fig. 5  -- estimator observability
# ==========================================================================

def fig_estimator():
    fig = plt.figure(figsize=(COL1, 2.25))
    ax = fig.add_axes([0.30, 0.235, 0.66, 0.60])

    # 0 = not observed, 1 = observed, 2 = coupling that vanishes at hover
    meas = ["VL53L0X altitude", "PMW3901 flow", "zero-velocity",
            "gravity direction", "magnetometer yaw"]
    hit = [
        [0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 2],
        [0, 0, 0, 0, 0, 0, 0, 2, 1],
    ]
    rate = ["100 Hz", "100 Hz", "20 Hz, on ground", "100 Hz", "100 Hz"]

    ax.set_xlim(-0.5, 8.5)
    ax.set_ylim(-0.6, 4.6)
    ax.invert_yaxis()
    ax.set_xticks(range(9))
    ax.set_xticklabels(["x", "y", "z"] * 3, fontsize=6.2)
    ax.set_yticks(range(5))
    ax.set_yticklabels(meas, fontsize=6.5)
    ax.tick_params(length=0)
    for s in ax.spines.values():
        s.set_visible(False)

    for i, row in enumerate(hit):
        for j, v in enumerate(row):
            if v == 1:
                ax.add_patch(Circle((j, i), 0.30, fc=ACC, ec=INK, lw=0.5,
                                    zorder=3))
            elif v == 2:
                ax.add_patch(Wedge((j, i), 0.30, 90, 270, fc=ACC, ec=INK,
                                   lw=0.5, zorder=3))
                ax.add_patch(Circle((j, i), 0.30, fc="none", ec=INK, lw=0.5,
                                    zorder=4))
            else:
                ax.add_patch(Circle((j, i), 0.30, fc="none", ec="#cfcfcf",
                                    lw=0.5, zorder=3))
    for j in (2.5, 5.5):
        ax.axvline(j, color=GREY, lw=0.5, ls=":")
    for i, r in enumerate(rate):
        ax.text(8.85, i, r, fontsize=5.8, color=GREY, va="center")

    ax.text(1.0, -1.12, "$\\delta p$", fontsize=7.5, ha="center")
    ax.text(4.0, -1.12, "$\\delta v$", fontsize=7.5, ha="center")
    ax.text(7.0, -1.12, "$\\delta\\theta$", fontsize=7.5, ha="center")

    fig.text(0.03, 0.055,
             "filled = observed;  half = coupling that vanishes at level, so "
             "gravity never\nsees heading and the compass sees only heading; "
             "empty = no Jacobian entry.",
             fontsize=5.9, color=GREY, linespacing=1.5)
    save(fig, "fig_estimator")


# ==========================================================================
# flight-log parsing
# ==========================================================================

PAT = re.compile(
    r"ST=(\d+) RC=([-+][\d.]+),([-+][\d.]+),([-+][\d.]+),([-+][\d.]+) "
    r"ARM=(\d) LNK=(\d) \| ALT=([\d.]+)/([\d.]+) VZ=([-+][\d.]+) \| "
    r"X=([-+][\d.]+)/([-+][\d.]+) Y=([-+][\d.]+)/([-+][\d.]+) "
    r"VB=([-+][\d.]+),([-+][\d.]+) \| RP=([-+][\d.]+)/([-+][\d.]+) "
    r"PT=([-+][\d.]+)/([-+][\d.]+) YAW=([-+][\d.]+)/([-+][\d.]+) "
    r"YR=([-+][\d.]+) \| T=([\d.]+)N TAU=([-+][\d.]+),([-+][\d.]+),([-+][\d.]+) \| "
    r"OMG=([\d.]+),([\d.]+),([\d.]+),([\d.]+) DS=(\d+),(\d+),(\d+),(\d+) "
    r"M=([0-9A-F]+) \| HOLD A=(\w+) X=(\w+) H=(\w+) YP=(\d)")

FLIGHT = "hover05_20260815_152920.log"


def load(fname):
    rows = []
    with open(os.path.join(LOGDIR, fname), "r", errors="replace") as fh:
        for idx, line in enumerate(fh):
            m = PAT.search(line)
            if not m:
                continue
            g = m.groups()
            rows.append(dict(
                idx=idx, st=int(g[0]), alt=float(g[7]), alt_t=float(g[8]),
                vz=float(g[9]), x=float(g[10]), xt=float(g[11]),
                y=float(g[12]), yt=float(g[13]),
                rp=float(g[16]), pt=float(g[18]),
                yaw=float(g[20]), yaw_t=float(g[21]), yr=float(g[22]),
                T=float(g[23]), tx=float(g[24]), ty=float(g[25]),
                tz=float(g[26]),
                omg=[float(g[27]), float(g[28]), float(g[29]), float(g[30])],
                ha=g[36], hx=g[37], hh=g[38], yp=int(g[39])))
    return rows


def longest_airborne(rows):
    """Longest run of consecutive airborne telemetry lines, so the synthesised
    5 Hz time axis never spans a disarm."""
    best, cur = [], []
    for r in rows:
        if r["st"] in (3, 4) and (not cur or r["idx"] - cur[-1]["idx"] <= 3):
            cur.append(r)
        else:
            if len(cur) > len(best):
                best = cur
            cur = [r] if r["st"] in (3, 4) else []
    return best if len(best) >= len(cur) else cur


# ==========================================================================
# Fig. 6  -- hover results
# ==========================================================================

def fig_hover():
    seg = longest_airborne(load(FLIGHT))
    if not seg:
        print("!! no airborne data")
        return
    t = np.arange(len(seg)) * 0.2
    print("   hover segment: %d samples, %.0f s" % (len(seg), t[-1]))

    fig, axs = plt.subplots(3, 1, figsize=(COL1, 3.4), sharex=True)
    fig.subplots_adjust(left=0.165, right=0.975, top=0.98, bottom=0.115,
                        hspace=0.14)

    axs[0].plot(t, [r["alt"] for r in seg], color=ACC, lw=0.8,
                label="measured")
    axs[0].plot(t, [r["alt_t"] for r in seg], color=RED, lw=0.8, ls="--",
                label="target")
    axs[0].set_ylabel("altitude [m]")
    axs[0].set_ylim(0.25, 0.62)
    axs[0].legend(frameon=False, loc="lower right", handlelength=1.6,
                  fontsize=6.2)

    axs[1].plot(t, [r["T"] for r in seg], color=ACC, lw=0.8)
    axs[1].axhline(0.45359237 * 9.80665, color=RED, lw=0.8, ls="--")
    axs[1].annotate("weight 4.448 N", xy=(t[-1] * 0.72, 4.448),
                    xytext=(t[-1] * 0.50, 4.28), fontsize=5.9, color=RED,
                    arrowprops=dict(arrowstyle="-", lw=0.5, color=RED))
    axs[1].set_ylabel("thrust cmd [N]")
    axs[1].set_ylim(4.15, 5.05)

    om = np.array([np.mean(r["omg"]) for r in seg])
    om[om < 400] = np.nan
    axs[2].plot(t, om, color=ACC, lw=0.8)
    axs[2].axhline(974.7, color=RED, lw=0.8, ls="--")
    axs[2].annotate("model hover 975 rad/s", xy=(t[-1] * 0.72, 974.7),
                    xytext=(t[-1] * 0.42, 940), fontsize=5.9, color=RED,
                    arrowprops=dict(arrowstyle="-", lw=0.5, color=RED))
    axs[2].set_ylabel("mean $\\omega$ [rad/s]")
    axs[2].set_ylim(925, 1035)
    axs[2].set_xlabel("time [s]   (5 Hz telemetry)")

    for a in axs:
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
        a.grid(True, lw=0.3, color="#e4e4e4")
        a.set_axisbelow(True)
        a.set_xlim(0, t[-1])

    save(fig, "fig_hover")


# ==========================================================================
# Fig. 7  -- attitude regulation, torque utilisation, XY hold
# ==========================================================================

def fig_results():
    rows = load(FLIGHT)
    seg = longest_airborne(rows)
    air = [r for r in rows if r["st"] in (3, 4)]
    t = np.arange(len(seg)) * 0.2

    fig = plt.figure(figsize=(COL2, 2.1))
    a1 = fig.add_axes([0.055, 0.185, 0.40, 0.775])
    a2 = fig.add_axes([0.560, 0.185, 0.175, 0.775])
    a3 = fig.add_axes([0.830, 0.185, 0.160, 0.775])

    # (a) attitude
    a1.plot(t, [r["rp"] for r in seg], color=ACC, lw=0.55, label="roll")
    a1.plot(t, [r["pt"] for r in seg], color=RED, lw=0.55, label="pitch")
    a1.set_ylabel("attitude [deg]")
    a1.set_xlabel("time [s]   (5 Hz telemetry)")
    a1.set_ylim(-5.5, 6.5)
    a1.set_xlim(0, t[-1])
    a1.legend(frameon=False, ncol=2, loc="upper right", handlelength=1.6,
              fontsize=6.4)
    a1.text(0.012, 0.035, "(a)", transform=a1.transAxes, fontsize=8,
            weight="bold")

    # (b) torque utilisation
    used = [max(abs(r["tx"]) for r in air), max(abs(r["ty"]) for r in air),
            max(abs(r["tz"]) for r in air)]
    avail = [0.342, 0.339, 0.0416]
    xs = np.arange(3)
    a2.bar(xs - 0.185, avail, 0.35, color=FILL2, ec=INK, lw=0.6)
    a2.bar(xs + 0.185, used, 0.35, color=ACC, ec=INK, lw=0.6)
    a2.set_yscale("log")
    a2.set_ylim(0.004, 1.15)
    a2.set_xticks(xs)
    a2.set_xticklabels(["$\\tau_x$", "$\\tau_y$", "$\\tau_z$"])
    a2.set_ylabel("torque [N$\\cdot$m]")
    for xi, (a, u) in enumerate(zip(avail, used)):
        a2.text(xi, 0.62, "%.0f%%" % (100 * u / a), ha="center", fontsize=6.0,
                color=INK)
    a2.text(0.05, 0.035, "(b)", transform=a2.transAxes, fontsize=8,
            weight="bold")

    # (c) XY hold error
    xy = [r for r in air if r["hx"] == "HELD"]
    if xy:
        ex = [r["x"] - r["xt"] for r in xy]
        ey = [r["y"] - r["yt"] for r in xy]
        a3.scatter(ex, ey, s=1.1, color=ACC, alpha=0.40, linewidths=0)
        for rad in (0.25, 0.50):
            a3.add_patch(Circle((0, 0), rad, fc="none", ec=GREY, lw=0.5,
                                ls=":"))
        a3.plot(0, 0, marker="+", color=RED, ms=6, mew=1.0)
        a3.set_xlim(-0.8, 0.8)
        a3.set_ylim(-0.8, 0.8)
        a3.set_xticks([-0.5, 0, 0.5])
        a3.set_yticks([-0.5, 0, 0.5])
        a3.set_aspect("equal")
        a3.set_xlabel("nav $x$ error [m]", fontsize=6.5)
        a3.set_ylabel("nav $y$ error [m]", fontsize=6.5)
        a3.text(0.05, 0.035, "(c)", transform=a3.transAxes, fontsize=8,
                weight="bold")

    for a in (a1, a2, a3):
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
        a.grid(True, lw=0.3, color="#e4e4e4")
        a.set_axisbelow(True)

    save(fig, "fig_results")


# ==========================================================================
# Fig. 8  -- flight state machine and hold engagement
# ==========================================================================

def fig_states():
    fig, ax = blank(COL2, 2.35)

    # ================= (a) flight state machine, left half ================
    ax.text(0.015, 0.975, "(a)", fontsize=8, weight="bold", va="top")
    ax.text(0.055, 0.975, "flight state machine", fontsize=6.6, color=INK,
            va="top")

    BX, BW, BH = 0.055, 0.185, 0.070
    states = [("DISARMED", 0.845), ("ARM RAMP", 0.685), ("GROUND IDLE", 0.525),
              ("CLIMB", 0.365), ("HOVER", 0.205)]
    for i, (nm, y) in enumerate(states):
        box(ax, BX, y, BW, BH, nm, fc=FILL2 if nm == "HOVER" else FILL, fs=6.8)
        if i:
            arrow(ax, (BX + BW / 2, states[i - 1][1]), (BX + BW / 2, y + BH))

    conds = ["arm requested,\nthrottle near centre",
             "2 s ramp complete",
             "throttle $>$ 0.10",
             "target reached, or\npilot throttle input"]
    for txt, (_, y) in zip(conds, states[1:]):
        ax.text(BX + BW + 0.018, y + BH + 0.045, txt, fontsize=5.7,
                color=GREY, va="center", linespacing=1.45)

    box(ax, BX, 0.045, BW, BH, "DISARM RAMP", fc=FILL, fs=6.8)
    for p0, p1 in (((BX, 0.240), (0.022, 0.240)), ((0.022, 0.240), (0.022, 0.080))):
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-", lw=0.6, color=INK))
    arrow(ax, (0.022, 0.080), (BX - 0.004, 0.080))
    arrow(ax, (BX + BW, 0.080), (BX + BW + 0.030, 0.080))
    ax.text(BX + BW + 0.038, 0.080, "1.5 s ramp,\nthen DISARMED",
            fontsize=5.7, color=GREY, va="center", linespacing=1.45)

    ax.text(0.015, -0.030,
            "released by: signal lost $\\cdot$ kill $\\cdot$ safety stop "
            "$\\cdot$ serial stop $\\cdot$ switch off",
            fontsize=5.6, color=GREY, va="bottom")

    # divider
    ax.plot([0.475, 0.475], [0.06, 0.95], color=PALE, lw=0.6, ls=":")

    # ============= (b) per-axis hold engagement, right half ===============
    ax.text(0.505, 0.975, "(b)", fontsize=8, weight="bold", va="top")
    ax.text(0.545, 0.975, "per-axis hold engagement", fontsize=6.6, color=INK,
            va="top")

    hb = [("FREE", 0.505), ("SETTLING", 0.680), ("HELD", 0.855)]
    for nm, x in hb:
        box(ax, x, 0.790, 0.135, 0.070, nm,
            fc=FILL2 if nm == "HELD" else FILL, fs=6.8)
    arrow(ax, (0.640, 0.825), (0.676, 0.825))
    arrow(ax, (0.815, 0.825), (0.851, 0.825))
    ax.text(0.658, 0.870, "release", fontsize=5.6, color=GREY, ha="center")
    ax.text(0.833, 0.870, "settle", fontsize=5.6, color=GREY, ha="center")

    # return path
    for p0, p1 in (((0.922, 0.790), (0.922, 0.735)),
                   ((0.922, 0.735), (0.572, 0.735))):
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-", lw=0.6,
                                     color=GREY, linestyle="--"))
    arrow(ax, (0.572, 0.735), (0.572, 0.786), color=GREY, ls="--")
    ax.text(0.747, 0.700, "stick input, or any precondition lost",
            fontsize=5.6, color=GREY, ha="center")

    ax.text(0.505, 0.640,
            "capture requires the axis speed to stay below\n"
            "its threshold for the whole dwell, so the target\n"
            "is a point the vehicle is actually at:",
            fontsize=5.9, color=INK, va="top", linespacing=1.55)

    ax.text(0.520, 0.470,
            "altitude   0.12 m/s   400 ms\n"
            "position   0.15 m/s   500 ms\n"
            "heading    8 deg/s    300 ms",
            fontsize=5.9, color=INK, va="top", family="monospace",
            linespacing=1.7)

    ax.text(0.505, 0.265,
            "Position hold additionally requires proof, within\n"
            "this flight, that the yaw loop is behaving: rate\n"
            "within 10 deg/s and heading error within 8 deg,\n"
            "continuously for 3 s while airborne. Disarming\n"
            "clears the proof.",
            fontsize=5.9, color=GREY, va="top", linespacing=1.55)

    save(fig, "fig_states")


if __name__ == "__main__":
    for f in (fig_wiring, fig_body, fig_cascade, fig_alloc, fig_estimator,
              fig_states, fig_hover, fig_results):
        f()
    print("\nfigures in", FIGS)
