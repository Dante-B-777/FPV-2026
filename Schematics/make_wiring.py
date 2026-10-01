#!/usr/bin/env python3
"""
Generate the FPV 2026 wiring diagrams.

    python make_wiring.py            -> wiring_receiver.pdf, wiring_transmitter.pdf
    python make_wiring.py --preview  -> also PNG at 190 dpi

Every pin assignment here is taken from the firmware:
    Receiver_RotorRate/Receiver_RotorRate.ino
    Transmitter_RotorRate/Transmitter_RotorRate.ino
If a pin moves in firmware it must move here too.

Wire colours encode PURPOSE, not net name:
    red      pack voltage, 14.8 V nominal
    orange   +5 V logic supply
    magenta  +3.3 V sensor supply
    black    ground
    blue     I2C
    green    SPI
    purple   DShot motor signal, bidirectional
    grey     3-phase motor output
    gold     analogue / ADC
    teal     digital input, active low
"""

import os
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, Circle

plt.rcParams.update({
    "pdf.fonttype": 42,
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.06,
})

HERE = os.path.dirname(os.path.abspath(__file__))
PREVIEW = "--preview" in sys.argv

INK = "#1a1a1a"
BOXFILL = "#ffffff"
CHIPFILL = "#eef3f2"
FAINT = "#9a9a9a"

C_VBAT = "#C62828"
C_5V = "#EF6C00"
C_3V3 = "#C2185B"
C_GND = "#212121"
C_I2C = "#1565C0"
C_SPI = "#2E7D32"
C_SIG = "#6A1B9A"
C_PHASE = "#5D5D5D"
C_ADC = "#9A7B0A"
C_DIN = "#00838F"


# ---------------------------------------------------------------- primitives

def sheet(w_in, h_in, vw, vh, title, subtitle):
    fig = plt.figure(figsize=(w_in, h_in))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, vw)
    ax.set_ylim(0, vh)
    ax.axis("off")
    ax.add_patch(Rectangle((2, 2), vw - 4, vh - 4, fc="none", ec=INK, lw=1.0))
    ax.text(16, vh - 22, title, fontsize=13, weight="bold", va="top")
    ax.text(16, vh - 42, subtitle, fontsize=7.5, color=FAINT, va="top")
    return fig, ax


def chip(ax, x, y, w, h, name, sub=None, left=None, right=None,
         top=None, bottom=None, fill=CHIPFILL, namefs=8.5, pinfs=6.6,
         stub=13):
    """Box with pin stubs. Each pin is (key, note) or (key, note, colour).

    A key may carry a display override as "KEY#suffix" -- the part before the
    "#" is what gets printed, the whole string is the dictionary key. That is
    how a chip with two pins both labelled GND stays addressable: without it
    the second one silently overwrites the first in the returned dict and one
    of them ends up drawn but unwired.

    Pin notes are printed INSIDE the body. Outside, they sit exactly where the
    wires run.

    Returns {key: (x, y)} of every stub end, for wiring.
    """
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0,rounding_size=3",
                                fc=fill, ec=INK, lw=1.0, zorder=3))
    ax.text(x + w / 2, y + h - 11, name, ha="center", va="top",
            fontsize=namefs, weight="bold", zorder=4)
    if sub:
        ax.text(x + w / 2, y + h - 11 - namefs * 1.55, sub, ha="center",
                va="top", fontsize=pinfs, color=FAINT, zorder=4,
                linespacing=1.4)

    pts = {}

    def run(pins, side):
        if not pins:
            return
        n = len(pins)
        span = h if side in ("l", "r") else w
        base = y if side in ("l", "r") else x
        step = span / (n + 1)
        for i, p in enumerate(pins):
            key, note_txt = p[0], p[1]
            disp = key.split("#", 1)[0]
            col = p[2] if len(p) > 2 else INK
            if side in ("l", "r"):
                off = base + span - (i + 1) * step
            else:
                off = base + (i + 1) * step
            if side == "l":
                ax.plot([x - stub, x], [off, off], color=col, lw=1.1, zorder=2)
                ax.text(x + 4, off, disp, ha="left", va="center",
                        fontsize=pinfs, zorder=4)
                if note_txt:
                    ax.text(x + 34, off, note_txt, ha="left", va="center",
                            fontsize=pinfs - 0.6, color=FAINT, zorder=4)
                pts[key] = (x - stub, off)
            elif side == "r":
                ax.plot([x + w, x + w + stub], [off, off], color=col, lw=1.1,
                        zorder=2)
                ax.text(x + w - 4, off, disp, ha="right", va="center",
                        fontsize=pinfs, zorder=4)
                if note_txt:
                    ax.text(x + w - 34, off, note_txt, ha="right",
                            va="center", fontsize=pinfs - 0.6, color=FAINT,
                            zorder=4)
                pts[key] = (x + w + stub, off)
            elif side == "t":
                ax.plot([off, off], [y + h, y + h + stub], color=col, lw=1.1,
                        zorder=2)
                pts[key] = (off, y + h + stub)
            else:
                ax.plot([off, off], [y - stub, y], color=col, lw=1.1, zorder=2)
                pts[key] = (off, y - stub)

    run(left, "l")
    run(right, "r")
    run(top, "t")
    run(bottom, "b")
    return pts


def wire(ax, pts, color, lw=1.1):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, lw=lw,
            zorder=2, solid_capstyle="round")


def junction(ax, x, y, color=INK):
    ax.add_patch(Circle((x, y), 2.6, fc=color, ec=color, zorder=5))


def gnd(ax, x, y, color=C_GND, scale=1.0):
    s = 7.0 * scale
    ax.plot([x, x], [y, y - s], color=color, lw=1.1, zorder=2)
    for i, wdt in enumerate((s * 1.5, s * 0.95, s * 0.45)):
        yy = y - s - i * (s * 0.42)
        ax.plot([x - wdt / 2, x + wdt / 2], [yy, yy], color=color, lw=1.1,
                zorder=2)


def netflag(ax, x, y, text, color, side="r"):
    d = 5.0
    sgn = 1 if side == "r" else -1
    ax.plot([x, x + sgn * d, x + sgn * (d + 6), x + sgn * d, x],
            [y, y + d, y, y - d, y], color=color, lw=1.0, zorder=4)
    ax.text(x + sgn * (d + 10), y, text,
            ha="left" if side == "r" else "right",
            va="center", fontsize=6.8, color=color, weight="bold", zorder=4)


def legend(ax, x, y, entries, title="WIRE COLOUR = PURPOSE", w=190):
    h = 20 + len(entries) * 13
    ax.add_patch(Rectangle((x, y - h), w, h, fc="#fbfbfb", ec=FAINT, lw=0.7,
                           zorder=3))
    ax.text(x + 8, y - 12, title, fontsize=6.4, weight="bold", va="center",
            zorder=4)
    for i, (col, label) in enumerate(entries):
        yy = y - 26 - i * 13
        ax.plot([x + 8, x + 30], [yy, yy], color=col, lw=2.0, zorder=4)
        ax.text(x + 36, yy, label, fontsize=6.2, va="center", zorder=4)


def note(ax, x, y, lines, w=230, kind="note"):
    h = 12 + len(lines) * 10.5
    edge = "#B08000" if kind == "warn" else FAINT
    face = "#FFF8E1" if kind == "warn" else "#f6f8f8"
    ax.add_patch(Rectangle((x, y - h), w, h, fc=face, ec=edge, lw=0.8,
                           zorder=3))
    for i, ln in enumerate(lines):
        ax.text(x + 7, y - 11 - i * 10.5, ln, fontsize=6.1, va="center",
                zorder=4, color=INK, weight="bold" if i == 0 else "normal")


def titleblock(ax, x, y, w, h, cells):
    ax.add_patch(Rectangle((x, y), w, h, fc="#fbfbfb", ec=FAINT, lw=0.8,
                           zorder=3))
    cw = w / len(cells)
    for i, (k, v) in enumerate(cells):
        cx = x + i * cw
        if i:
            ax.plot([cx, cx], [y, y + h], color=FAINT, lw=0.6, zorder=4)
        ax.text(cx + 10, y + h - 16, k, fontsize=5.8, color=FAINT,
                weight="bold", zorder=4)
        ax.text(cx + 10, y + h - 36, v, fontsize=6.8, zorder=4)


def save(fig, name):
    fig.savefig(os.path.join(HERE, name + ".pdf"))
    if PREVIEW:
        fig.savefig(os.path.join(HERE, name + ".png"), dpi=190)
    plt.close(fig)
    print("wrote " + name + ".pdf")


# ------------------------------------------------------------------ receiver

def receiver():
    VW, VH = 1200, 880
    fig, ax = sheet(12.0, 8.8, VW, VH,
                    "FPV 2026  -  RECEIVER / FLIGHT CONTROLLER",
                    "ESP32-WROOM-32 devkit   |   pins per "
                    "Receiver_RotorRate.ino   |   all grounds common")

    # ---------------- power band, across the top ---------------------------
    ax.text(40, 806, "POWER", fontsize=7.2, weight="bold", color=FAINT)

    ax.add_patch(Rectangle((40, 694), 100, 82, fc=BOXFILL, ec=INK, lw=1.0,
                           zorder=3))
    ax.text(90, 758, "4S LiPo", ha="center", fontsize=7.8, weight="bold",
            zorder=4)
    ax.text(90, 744, "14.8 V nominal", ha="center", fontsize=6.2,
            color=FAINT, zorder=4)
    ax.text(90, 732, "16.8 V full", ha="center", fontsize=6.2, color=FAINT,
            zorder=4)
    ax.text(90, 720, "XT60", ha="center", fontsize=6.0, color=FAINT, zorder=4)

    wire(ax, [(140, 756), (172, 756)], C_VBAT, 2.4)
    wire(ax, [(140, 708), (172, 708)], C_GND, 1.4)
    netflag(ax, 172, 756, "VBAT", C_VBAT)
    gnd(ax, 172, 708)

    buck = chip(ax, 256, 700, 130, 66, "BUCK", "15 V -> 5 V",
                left=[("IN", "", C_VBAT), ("GND#L", "", C_GND)],
                right=[("OUT", "", C_5V), ("GND#R", "", C_GND)])

    TRUNK = 800
    wire(ax, [(172, 756), (208, 756), (208, TRUNK)], C_VBAT, 2.4)
    wire(ax, [(208, TRUNK), (232, TRUNK), (232, buck["IN"][1]), buck["IN"]],
         C_VBAT, 2.4)
    junction(ax, 208, TRUNK, C_VBAT)
    gnd(ax, buck["GND#R"][0] + 8, buck["GND#R"][1])
    gnd(ax, buck["GND#L"][0] - 8, buck["GND#L"][1])

    # ---------------- ESP32 -------------------------------------------------
    EX, EY, EW, EH = 470, 100, 200, 540
    esp = chip(ax, EX, EY, EW, EH, "ESP32-WROOM-32", "DEVKIT V1",
               left=[("3V3", "3.3 V out", C_3V3),
                     ("GND#L", "", C_GND),
                     ("22", "SDA", C_I2C),
                     ("21", "SCL", C_I2C),
                     ("18", "SCK", C_SPI),
                     ("19", "MISO", C_SPI),
                     ("23", "MOSI", C_SPI),
                     ("13", "CS", C_SPI),
                     ("33", "RST", C_SPI)],
               right=[("VIN", "5 V in", C_5V),
                      ("GND#R", "", C_GND),
                      ("26", "M1", C_SIG),
                      ("14", "M2", C_SIG),
                      ("27", "M3", C_SIG),
                      ("25", "M4", C_SIG)],
               namefs=9.0, pinfs=7.0)

    wire(ax, [buck["OUT"], (764, buck["OUT"][1]),
              (764, esp["VIN"][1]), esp["VIN"]], C_5V, 2.0)
    ax.text(575, buck["OUT"][1] + 9, "+5 V", fontsize=6.6, color=C_5V,
            weight="bold", ha="center")
    gnd(ax, esp["GND#R"][0] + 14, esp["GND#R"][1])

    # ---------------- 3V3 and GND rails, far left --------------------------
    RAIL_X, GRAIL_X = 34, 52
    wire(ax, [esp["3V3"], (430, esp["3V3"][1]), (430, 664),
              (RAIL_X, 664), (RAIL_X, 118)], C_3V3, 1.8)
    netflag(ax, 430, esp["3V3"][1], "+3V3", C_3V3, side="l")
    wire(ax, [esp["GND#L"], (448, esp["GND#L"][1]), (448, 650),
              (GRAIL_X, 650), (GRAIL_X, 118)], C_GND, 1.6)
    gnd(ax, GRAIL_X, 118)

    # ---------------- sensor column ----------------------------------------
    SDA_X, SCL_X = 340, 356
    ax.text(348, 628, "I2C  400 kHz", fontsize=6.6, color=C_I2C,
            weight="bold", ha="center")

    for nm, sub, yy in (("MPU-6050", "IMU  0x68\nmounted inverted", 536),
                        ("VL53L0X", "rangefinder  0x29\nfacing down", 416),
                        ("QMC5883P", "magnetometer  0x2C\nmounted reversed",
                         296)):
        s = chip(ax, 90, yy, 160, 80, nm, sub,
                 right=[("SDA", "", C_I2C), ("SCL", "", C_I2C)],
                 left=[("VCC", "", C_3V3), ("GND", "", C_GND)],
                 namefs=7.8, pinfs=6.4, stub=11)
        wire(ax, [s["VCC"], (RAIL_X, s["VCC"][1])], C_3V3, 1.2)
        junction(ax, RAIL_X, s["VCC"][1], C_3V3)
        wire(ax, [s["GND"], (GRAIL_X, s["GND"][1])], C_GND, 1.2)
        junction(ax, GRAIL_X, s["GND"][1], C_GND)
        wire(ax, [s["SDA"], (SDA_X, s["SDA"][1])], C_I2C, 1.2)
        junction(ax, SDA_X, s["SDA"][1], C_I2C)
        wire(ax, [s["SCL"], (SCL_X, s["SCL"][1])], C_I2C, 1.2)
        junction(ax, SCL_X, s["SCL"][1], C_I2C)

    wire(ax, [esp["22"], (SDA_X, esp["22"][1])], C_I2C, 1.6)
    wire(ax, [esp["21"], (SCL_X, esp["21"][1])], C_I2C, 1.6)
    wire(ax, [(SDA_X, 349), (SDA_X, 609)], C_I2C, 1.6)
    wire(ax, [(SCL_X, 323), (SCL_X, 583)], C_I2C, 1.6)

    f = chip(ax, 90, 126, 160, 120, "PMW3901", "optical flow\nSPI  2 MHz",
             right=[("SCK", "", C_SPI), ("MISO", "", C_SPI),
                    ("MOSI", "", C_SPI), ("CS", "", C_SPI),
                    ("RST", "", C_SPI)],
             left=[("VCC", "", C_3V3), ("GND", "", C_GND)],
             namefs=7.8, pinfs=6.4, stub=11)
    wire(ax, [f["VCC"], (RAIL_X, f["VCC"][1])], C_3V3, 1.2)
    junction(ax, RAIL_X, f["VCC"][1], C_3V3)
    wire(ax, [f["GND"], (GRAIL_X, f["GND"][1])], C_GND, 1.2)
    junction(ax, GRAIL_X, f["GND"][1], C_GND)

    for pin, gpio, lane in (("SCK", "18", 270), ("MISO", "19", 282),
                            ("MOSI", "23", 294), ("CS", "13", 306),
                            ("RST", "33", 318)):
        wire(ax, [f[pin], (lane, f[pin][1]), (lane, esp[gpio][1]), esp[gpio]],
             C_SPI, 1.2)
    ax.text(294, 260, "SPI", fontsize=6.6, color=C_SPI, weight="bold",
            ha="center")

    # ---------------- ESC and motors ---------------------------------------
    esc = chip(ax, 800, 250, 170, 310, "4-IN-1 ESC   45 A",
               "BLHeli-S / Bluejay\nbidirectional DShot300",
               left=[("S1", "", C_SIG), ("S2", "", C_SIG),
                     ("S3", "", C_SIG), ("S4", "", C_SIG),
                     ("GND", "", C_GND), ("+5V", "", C_5V)],
               right=[("M1", "", C_PHASE), ("M2", "", C_PHASE),
                      ("M3", "", C_PHASE), ("M4", "", C_PHASE)],
               top=[("PWR", "", C_VBAT)],
               namefs=8.4, pinfs=6.8)
    ax.text(885, 588, "6-pin signal header", fontsize=6.3, color=FAINT,
            ha="center")

    wire(ax, [(208, TRUNK), (885, TRUNK), esc["PWR"]], C_VBAT, 2.6)
    ax.text(560, TRUNK + 9, "14.8 V direct to the ESC power pads",
            fontsize=6.4, color=C_VBAT, ha="center")

    wire(ax, [esc["GND"], (768, esc["GND"][1])], C_GND, 1.4)
    gnd(ax, 768, esc["GND"][1])
    cx, cy = esc["+5V"][0] - 14, esc["+5V"][1]
    ax.plot([cx, cx + 9], [cy + 4.5, cy - 4.5], color=FAINT, lw=1.1, zorder=4)
    ax.plot([cx, cx + 9], [cy - 4.5, cy + 4.5], color=FAINT, lw=1.1, zorder=4)
    ax.text(cx - 6, cy, "n/c", ha="right", va="center", fontsize=6.0,
            color=FAINT)

    for gpio, escpin, lane in (("26", "S1", 700), ("14", "S2", 712),
                               ("27", "S3", 724), ("25", "S4", 736)):
        wire(ax, [esp[gpio], (lane, esp[gpio][1]), (lane, esc[escpin][1]),
                  esc[escpin]], C_SIG, 1.3)
    ax.text(692, 618, "DShot300, bidirectional\n(command out, eRPM back)",
            fontsize=6.3, color=C_SIG, ha="left", linespacing=1.4)

    motors = (("M1", "front-left", "CCW"), ("M2", "front-right", "CW"),
              ("M3", "rear-right", "CCW"), ("M4", "rear-left", "CW"))
    for (mk, place, spin), key in zip(motors, ("M1", "M2", "M3", "M4")):
        px, py = esc[key]
        mx, mh = 1012, 52
        ax.add_patch(Rectangle((mx, py - mh / 2), 148, mh, fc=BOXFILL,
                               ec=INK, lw=0.9, zorder=3))
        ax.text(mx + 74, py + 13, mk + "   " + place, ha="center",
                fontsize=7.0, weight="bold", zorder=4)
        ax.text(mx + 74, py + 1, "brushless, 14 poles", ha="center",
                fontsize=5.9, color=FAINT, zorder=4)
        ax.text(mx + 74, py - 12, spin, ha="center", fontsize=6.3,
                color=FAINT, zorder=4)
        for k in (-1, 0, 1):
            wire(ax, [(px, py + k * 5), (mx, py + k * 5)], C_PHASE, 1.0)
        ax.text((px + mx) / 2, py + 17, "3-phase", ha="center", fontsize=5.7,
                color=FAINT)

    # ---------------- legend, notes, title block ---------------------------
    legend(ax, 700, 232, [
        (C_VBAT, "pack, 14.8 V"),
        (C_5V, "+5 V logic"),
        (C_3V3, "+3.3 V sensors"),
        (C_GND, "ground"),
        (C_I2C, "I2C"),
        (C_SPI, "SPI"),
        (C_SIG, "DShot signal"),
        (C_PHASE, "motor phases"),
    ], w=172)

    note(ax, 892, 232, [
        "I2C IS SWAPPED FROM THE ESP32 DEFAULT",
        "SDA = GPIO22,  SCL = GPIO21.  Every generic ESP32",
        "pinout says the opposite. Firmware calls",
        "Wire.begin(22, 21); wiring 21/22 gives a dead bus.",
    ], w=272, kind="warn")

    note(ax, 892, 152, [
        "BUILD NOTES",
        "- ESC +5V pin left unconnected: the ESP32 is fed from",
        "  the separate buck, not the ESC BEC.",
        "- All four sensors share the devkit 3V3 regulator.",
        "- GPIO 6-11 are the internal SPI flash. Never connect.",
        "- Free pins: 4, 16, 17, 32, 34, 35, 36, 39.",
    ], w=272)

    titleblock(ax, 40, 26, 600, 60,
               (("PROJECT", "FPV 2026 quadrotor"),
                ("SHEET", "1 of 2  -  receiver"),
                ("SOURCE", "Receiver_RotorRate.ino"),
                ("DRAWN BY", "make_wiring.py")))

    save(fig, "wiring_receiver")


# --------------------------------------------------------------- transmitter

def transmitter():
    VW, VH = 1120, 700
    fig, ax = sheet(11.2, 7.0, VW, VH,
                    "FPV 2026  -  TRANSMITTER",
                    "ESP32-WROOM-32 devkit   |   pins per "
                    "Transmitter_RotorRate.ino   |   ESP-NOW peer "
                    "5C:01:3B:BA:CA:EC")

    EX, EY, EW, EH = 400, 132, 200, 452
    esp = chip(ax, EX, EY, EW, EH, "ESP32-WROOM-32", "DEVKIT V1",
               left=[("GND#L", "", C_GND),
                     ("3V3", "3.3 V out", C_3V3),
                     ("35", "throttle", C_ADC),
                     ("34", "roll", C_ADC),
                     ("32", "pitch", C_ADC),
                     ("33", "yaw pot", C_ADC),
                     ("27", "yaw CCW", C_DIN),
                     ("26", "yaw CW", C_DIN),
                     ("13", "kill", C_DIN),
                     ("14", "on / off", C_DIN)],
               right=[("VIN", "5-6 V in", C_5V),
                      ("GND#R", "", C_GND),
                      ("22", "ON LED", C_DIN),
                      ("21", "OFF LED", C_DIN)],
               namefs=9.0, pinfs=7.0)

    # ---------------- battery, top right ------------------------------------
    ax.add_patch(Rectangle((850, 560), 106, 66, fc=BOXFILL, ec=INK, lw=1.0,
                           zorder=3))
    ax.text(903, 608, "Battery pack", ha="center", fontsize=7.6,
            weight="bold", zorder=4)
    ax.text(903, 594, "+6 V", ha="center", fontsize=6.4, color=FAINT,
            zorder=4)
    ax.text(903, 581, "4 x AA or 2S", ha="center", fontsize=6.0, color=FAINT,
            zorder=4)

    wire(ax, [(850, 604), (700, 604), (700, esp["VIN"][1]), esp["VIN"]],
         C_5V, 1.8)
    wire(ax, [(850, 578), (724, 578), (724, esp["GND#R"][1]), esp["GND#R"]],
         C_GND, 1.4)
    gnd(ax, 668, esp["GND#R"][1])

    # ---------------- 3V3 and GND rails, far left ---------------------------
    RAIL_X, GRAIL_X = 34, 52
    wire(ax, [esp["3V3"], (352, esp["3V3"][1]), (352, 610),
              (RAIL_X, 610), (RAIL_X, 366)], C_3V3, 1.6)
    netflag(ax, 352, esp["3V3"][1], "+3V3", C_3V3, side="l")
    wire(ax, [esp["GND#L"], (364, esp["GND#L"][1]), (364, 626),
              (GRAIL_X, 626), (GRAIL_X, 162)], C_GND, 1.4)
    gnd(ax, GRAIL_X, 162)

    # ---------------- gimbals ------------------------------------------------
    g1 = chip(ax, 72, 446, 188, 132, "GIMBAL 1", "throttle / yaw",
              left=[("VCC", "", C_3V3), ("GND", "", C_GND)],
              right=[("VRY", "throttle", C_ADC), ("VRX", "yaw", C_ADC),
                     ("SW", "yaw CCW", C_DIN)],
              namefs=7.8, pinfs=6.4, stub=11)
    g2 = chip(ax, 72, 286, 188, 132, "GIMBAL 2", "roll / pitch",
              left=[("VCC", "", C_3V3), ("GND", "", C_GND)],
              right=[("VRX", "roll", C_ADC), ("VRY", "pitch", C_ADC),
                     ("SW", "yaw CW", C_DIN)],
              namefs=7.8, pinfs=6.4, stub=11)

    for g in (g1, g2):
        wire(ax, [g["VCC"], (RAIL_X, g["VCC"][1])], C_3V3, 1.2)
        junction(ax, RAIL_X, g["VCC"][1], C_3V3)
        wire(ax, [g["GND"], (GRAIL_X, g["GND"][1])], C_GND, 1.2)
        junction(ax, GRAIL_X, g["GND"][1], C_GND)

    # One dedicated lane per signal, so no two routes can share a column.
    for src, dst, col, lane in ((g1["VRY"], esp["35"], C_ADC, 290),
                                (g1["VRX"], esp["33"], C_ADC, 300),
                                (g1["SW"], esp["27"], C_DIN, 310),
                                (g2["VRX"], esp["34"], C_ADC, 320),
                                (g2["VRY"], esp["32"], C_ADC, 330),
                                (g2["SW"], esp["26"], C_DIN, 340)):
        wire(ax, [src, (lane, src[1]), (lane, dst[1]), dst], col, 1.2)

    # ---------------- buttons, aligned to their pins so runs stay straight ---
    for label, gpio, bh in (("KILL", "13", 32), ("ON / OFF", "14", 32)):
        py = esp[gpio][1]
        ax.add_patch(Rectangle((150, py - bh / 2), 84, bh, fc=BOXFILL,
                               ec=INK, lw=0.9, zorder=3))
        ax.text(192, py + 6, label, ha="center", fontsize=6.9, weight="bold",
                zorder=4)
        ax.text(192, py - 6, "momentary", ha="center", fontsize=5.9,
                color=FAINT, zorder=4)
        wire(ax, [(234, py), esp[gpio]], C_DIN, 1.2)
        wire(ax, [(150, py), (GRAIL_X, py)], C_GND, 1.2)
        junction(ax, GRAIL_X, py, C_GND)

    ax.text(150, 142, "active LOW, INPUT_PULLUP  -  no external resistor",
            fontsize=6.0, color=FAINT)

    # ---------------- LEDs ---------------------------------------------------
    for label, gpio, col in (("ON LED", "22", "#2E7D32"),
                             ("OFF LED", "21", "#C62828")):
        px, py = esp[gpio]
        ax.add_patch(Rectangle((px + 70, py - 7), 30, 14, fc="none", ec=INK,
                               lw=0.8, zorder=3))
        ax.text(px + 85, py, "330", ha="center", va="center", fontsize=5.5,
                zorder=4)
        ax.add_patch(Circle((px + 150, py), 8.5, fc=col, ec=INK, lw=0.8,
                            zorder=3, alpha=0.85))
        ax.text(px + 166, py, label, va="center", fontsize=6.5, zorder=4)
        wire(ax, [(px, py), (px + 70, py)], C_DIN, 1.2)
        wire(ax, [(px + 100, py), (px + 141, py)], C_DIN, 1.2)
        gnd(ax, px + 150, py - 9)

    # ---------------- legend, notes, title block ----------------------------
    legend(ax, 830, 318, [
        (C_5V, "+5 / 6 V supply"),
        (C_3V3, "+3.3 V"),
        (C_GND, "ground"),
        (C_ADC, "analogue, ADC1"),
        (C_DIN, "digital, active low"),
    ], w=166)

    note(ax, 830, 186, [
        "GIMBAL POTS MUST RUN FROM 3.3 V",
        "GPIO34/35 are 3.6 V absolute max. Feeding the pots",
        "from 5 V or 6 V puts the wiper above the ADC input",
        "range and over the pin spec. Verify before power-on.",
    ], w=262, kind="warn")

    note(ax, 40, 108, [
        "NOTES",
        "- All four axes use ADC1. ADC2 (0,2,4,12-15,25-27) cannot be read",
        "  while WiFi is up, and ESP-NOW keeps it up.",
        "- GPIO33 yaw pot NOT FITTED: firmware default YAW_MODE 0 takes yaw",
        "  from the two gimbal pushbuttons at +/-45 deg/s.",
        "- GPIO34/35 are input-only; INPUT_PULLUP is unavailable there.",
        "- Stick calibration lives in NVS, not in source.",
    ], w=380)

    titleblock(ax, 470, 30, 470, 58,
               (("SHEET", "2 of 2  -  transmitter"),
                ("SOURCE", "Transmitter_RotorRate.ino"),
                ("LINK", "ESP-NOW, 50 Hz")))

    save(fig, "wiring_transmitter")


if __name__ == "__main__":
    receiver()
    transmitter()
