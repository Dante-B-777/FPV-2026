#!/usr/bin/env python3
"""
Analyse a flight track log captured with `tracklog on`.

    python analyze_track.py logs/circle_20260816_143000.log
    python analyze_track.py logs/circle.log --out figs --span 40 110

Reads the "#TRKCOLS" header out of the capture, so column order is never
hard-coded here. Validates the log before plotting anything: a sequence gap
or a non-uniform sample interval invalidates every time-domain result, and
you want to know that before you put a figure in a report.

Produces (PDF by default, PNG with --png for the web):
    track_overview   the full command-to-response chain against time
    track_ground     ground track, with RC stick command overlaid
    track_axes       per-axis commanded vs achieved
    track_chain      one axis, all five cascade stages stacked
"""

import argparse
import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

plt.rcParams.update({
    "pdf.fonttype": 42,
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8,
    "axes.labelsize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "axes.linewidth": 0.6,
    "lines.linewidth": 0.9,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

COL1, COL2 = 3.5, 7.16

# CONTROL_PERIOD_US in Receiver_RotorRate.ino. The control loop is gated on
# this, so dt_us in the log is a period measured against it, never below it.
CONTROL_PERIOD_US = 2500      # 400 Hz

"""
Output format for the figures.

PDF is the default because these go in a LaTeX report. PNG exists because they
also go on a web page, and a browser will not render a PDF inline. Set by
--png; the figures themselves are identical either way.
"""
FIGURE_FORMAT = "pdf"
FIGURE_DPI = 200


def savefig(fig, out, stem):
    path = os.path.join(out, "%s.%s" % (stem, FIGURE_FORMAT))
    fig.savefig(path, dpi=FIGURE_DPI)
    return path


MEAS = "#14595c"      # measured / achieved
CMD = "#a8382b"       # commanded / target
RC = "#8a6d1f"        # raw pilot input
GREY = "#777777"
PALE = "#dddddd"


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def load(path):
    """Return (dict of column -> ndarray, metadata dict)."""
    cols = None
    rows = []
    meta = {}

    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()

            if line.startswith("#TRKCOLS,"):
                cols = line.split(",")[1:]
                continue

            if line.startswith("#TRK "):
                # Only key=value tokens with an identifier-shaped key. The
                # header also carries prose lines ("0=free 1=settling") that
                # would otherwise land in here as junk entries.
                for token in line[5:].split():
                    if "=" not in token:
                        continue
                    k, v = token.split("=", 1)
                    if k and (k[0].isalpha() or k[0] == "_") and \
                            all(c.isalnum() or c == "_" for c in k):
                        meta[k] = v
                if line.startswith("#TRK v"):
                    meta["version"] = line.split()[1]
                continue

            if line.startswith("TRK,"):
                parts = line.split(",")[1:]
                rows.append(parts)

    if cols is None:
        sys.exit("No #TRKCOLS header found. Was the capture started before "
                 "`tracklog on` was issued? The header is printed once, at "
                 "the moment logging is enabled.")
    if not rows:
        sys.exit("Header found but no TRK data rows.")

    width = len(cols)
    good = [r for r in rows if len(r) == width]
    if len(good) != len(rows):
        print("  note: discarded %d truncated rows (serial corruption)"
              % (len(rows) - len(good)))

    table = np.array(good, dtype=object)
    data = {}
    for i, name in enumerate(cols):
        try:
            data[name] = table[:, i].astype(float)
        except ValueError:
            data[name] = np.array(
                [float(int(v, 16)) for v in table[:, i]])

    return data, meta


def validate(d):
    """Check the log is fit to plot. Returns seconds-since-start."""
    n = len(d["t_ms"])
    seq = d["seq"].astype(np.int64)
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0

    print("  rows            : %d" % n)
    print("  duration        : %.1f s" % (t[-1] - t[0]))

    gaps = np.diff(seq)

    """
    A NEGATIVE gap is a sequence reset, not a loss.

    `tracklog` zeroes the counter every time logging starts, so a capture that
    had logging stopped and restarted holds two runs back to back. Summing raw
    gaps across that boundary produced a nonsense count - a file with 3024
    good rows followed by a 4-row restart reported "-3024 samples missing".
    Sessions are therefore identified first, and continuity is checked WITHIN
    each one.
    """
    restarts = np.where(gaps < 0)[0]
    if len(restarts):
        bounds = [0] + [int(i) + 1 for i in restarts] + [n]
        print("  *** this capture holds %d separate logging sessions"
              % (len(bounds) - 1))
        for k in range(len(bounds) - 1):
            a, b = bounds[k], bounds[k + 1]
            print("        session %d: %d rows, t = %.1f to %.1f s"
                  % (k + 1, b - a, t[a], t[b - 1]))
        print("      They are concatenated here, which is meaningless for any")
        print("      time-domain result. Select one with --span, e.g.")
        print("      --span %.1f %.1f for session 1."
              % (t[bounds[0]], t[bounds[1] - 1]))

    within = gaps[gaps >= 0]
    missing = int(np.sum(within - 1)[()]) if len(within) else 0
    if missing:
        """
        Severity has to be proportionate, or the warning gets ignored.

        A single isolated sample lost to line noise is not the same failure as
        a burst dropout, and telling someone to re-fly over one row in 2318
        (0.04%, measured on the 2026-08-18 props-on bench run) trains them to
        skip the message that matters. What actually invalidates a time-domain
        result is either a large total loss or a CONTIGUOUS gap, which leaves
        a hole no interpolation can honestly cover.
        """
        idx = np.where(gaps > 1)[0]
        biggest = int(gaps[idx].max() - 1)
        rate = 100.0 * missing / (missing + n)

        print("  sequence        : %d samples missing across %d breaks "
              "(%.3f%%, biggest gap %d)" % (missing, len(idx), rate, biggest))
        print("                    first at t=%.2f s" % t[idx[0]])

        if rate > 0.5 or biggest > 3:
            print("      *** that is enough to invalidate a time-domain "
                  "result.")
            print("      Lower the tracklog rate and re-fly before using this "
                  "for a report.")
        else:
            print("                    isolated losses at this level are line "
                  "noise, not a")
            print("                    sampling failure - the figures are "
                  "still usable.")
    elif not len(restarts):
        print("  sequence        : continuous, no samples lost")
    else:
        print("  sequence        : continuous within each session")

    dt = np.diff(t)
    if len(dt):
        print("  sample interval : mean %.1f ms, sd %.2f ms, max %.1f ms"
              % (dt.mean() * 1e3, dt.std() * 1e3, dt.max() * 1e3))
        if dt.std() * 1e3 > 2.0:
            print("      *** jittery. Check `timing` for dropped samples.")

    if "dt_us" in d:
        """
        dt_us is the control PERIOD, not the duration of the step's work.

        The firmware computes it as `nowUs - previousControlUs` and then
        returns early if it is below CONTROL_PERIOD_US, so it can never read
        below 2500 us. Labelling it "control step ... (budget 2500)" made
        every log look pinned at 100% of budget, which is alarming and wrong.
        What it actually measures is scheduling jitter: how late each cycle
        ran. For the work duration, use the `timing` serial command.
        """
        period = d["dt_us"]
        late = period - CONTROL_PERIOD_US
        print("  control period  : mean %.0f us, max %.0f us  "
              "(target %d, so +%.0f us mean jitter)"
              % (period.mean(), period.max(), CONTROL_PERIOD_US, late.mean()))

        # A cycle that runs a whole period late means one was missed outright.
        overrun = int((period >= 2 * CONTROL_PERIOD_US).sum())
        if overrun:
            print("      *** %d cycles ran a full period late - the control "
                  "loop missed them. Check `timing`." % overrun)

    for name, label in (("sat_yaw_demand", "yaw demand clipped"),
                        ("sat_yaw_alloc", "yaw allocation clipped"),
                        ("sat_rollpitch", "roll/pitch scaled")):
        if name in d:
            k = int(d[name].sum())
            if k:
                print("  *** %s on %d of %d rows" % (label, k, n))

    if "flow_ok" in d:
        print("  flow healthy    : %.0f%% of rows"
              % (100.0 * d["flow_ok"].mean()))
    if "flow_squal" in d:
        print("  flow squal      : mean %.0f, min %.0f"
              % (d["flow_squal"].mean(), d["flow_squal"].min()))

    return t


def airborne_mask(d):
    """Rows where the aircraft is in Climb or Hover."""
    return (d["state"] == 3) | (d["state"] == 4)


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def tidy(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(True, lw=0.3, color="#e4e4e4")
    ax.set_axisbelow(True)


def fig_overview(d, t, out):
    fig, axs = plt.subplots(6, 1, figsize=(COL2, 8.2), sharex=True)
    fig.subplots_adjust(left=0.085, right=0.985, top=0.99, bottom=0.055,
                        hspace=0.16)

    axs[0].plot(t, d["rc_roll"], color=RC, lw=0.7, label="roll stick")
    axs[0].plot(t, d["rc_pitch"], color=MEAS, lw=0.7, label="pitch stick")
    axs[0].plot(t, d["rc_thr"], color=CMD, lw=0.7, label="throttle")
    axs[0].set_ylabel("RC input\n[-1, 1]")
    axs[0].legend(frameon=False, ncol=3, loc="upper right")

    axs[1].plot(t, d["vbx_t"], color=CMD, lw=0.8, ls="--", label="target")
    axs[1].plot(t, d["vbx"], color=MEAS, lw=0.8, label="measured")
    axs[1].set_ylabel("body $v_x$\n[m/s]")
    axs[1].legend(frameon=False, ncol=2, loc="upper right")

    axs[2].plot(t, d["vby_t"], color=CMD, lw=0.8, ls="--")
    axs[2].plot(t, d["vby"], color=MEAS, lw=0.8)
    axs[2].set_ylabel("body $v_y$\n[m/s]")

    axs[3].plot(t, d["roll_d"], color=CMD, lw=0.8, ls="--")
    axs[3].plot(t, d["roll"], color=MEAS, lw=0.8)
    axs[3].set_ylabel("roll\n[deg]")

    axs[4].plot(t, d["pitch_d"], color=CMD, lw=0.8, ls="--")
    axs[4].plot(t, d["pitch"], color=MEAS, lw=0.8)
    axs[4].set_ylabel("pitch\n[deg]")

    axs[5].plot(t, d["alt_t"], color=CMD, lw=0.8, ls="--")
    axs[5].plot(t, d["alt"], color=MEAS, lw=0.8)
    axs[5].set_ylabel("altitude\n[m]")
    axs[5].set_xlabel("time [s]")

    for a in axs:
        tidy(a)
        a.set_xlim(t[0], t[-1])

    savefig(fig, out, "track_overview")
    plt.close(fig)
    print("  wrote track_overview.%s" % FIGURE_FORMAT)


def fig_ground(d, t, out):
    m = airborne_mask(d)
    if not m.any():
        print("  skipped track_ground.pdf: no airborne rows")
        return

    px, py = d["px"][m], d["py"][m]
    held = d["hold_xy"][m] == 2

    fig = plt.figure(figsize=(COL2, 3.1))
    a1 = fig.add_axes([0.06, 0.135, 0.40, 0.83])
    a2 = fig.add_axes([0.57, 0.135, 0.40, 0.83])

    # --- ground track ----------------------------------------------------
    a1.plot(px, py, color=MEAS, lw=0.8, label="flown path")
    if held.any():
        a1.plot(d["px_t"][m][held], d["py_t"][m][held], color=CMD, lw=0.8,
                ls="--", label="hold target")
    a1.plot(px[0], py[0], marker="o", ms=4, color=MEAS, mfc="white",
            label="start")
    a1.plot(px[-1], py[-1], marker="s", ms=4, color=MEAS, label="end")
    a1.set_xlabel("nav $x$ [m]  (east at reset)")
    a1.set_ylabel("nav $y$ [m]  (north at reset)")
    a1.set_aspect("equal")
    a1.legend(loc="center", framealpha=0.88, edgecolor="none", fontsize=6.4)
    tidy(a1)

    # fitted circle, if the path looks like one
    cx, cy, r, resid = fit_circle(px, py)
    if r is not None and 0.15 < r < 5.0:
        a1.add_patch(Circle((cx, cy), r, fc="none", ec=GREY, lw=0.6, ls=":"))
        a1.plot(cx, cy, marker="+", color=GREY, ms=6)
        a1.set_title("fitted circle r=%.3f m, rms resid %.3f m"
                     % (r, resid), fontsize=7)

    # --- stick command vs measured body velocity -------------------------
    a2.plot(t[m], d["rc_roll"][m], color=RC, lw=0.7,
            label="roll stick (right)")
    a2.plot(t[m], d["rc_pitch"][m], color=GREY, lw=0.7,
            label="pitch stick (fwd)")
    a2b = a2.twinx()
    a2b.plot(t[m], d["vbx"][m], color=MEAS, lw=0.8, label="body $v_x$")
    a2b.plot(t[m], d["vby"][m], color=CMD, lw=0.8, label="body $v_y$")
    a2b.set_ylabel("body velocity [m/s]")
    a2.set_xlabel("time [s]")
    a2.set_ylabel("stick [-1, 1]")
    # Opaque frames: these sit over dense traces and are unreadable without.
    a2.legend(loc="upper left", framealpha=0.88, edgecolor="none",
              fontsize=6.4)
    a2b.legend(loc="upper right", framealpha=0.88, edgecolor="none",
               fontsize=6.4)
    tidy(a2)
    for s in ("top",):
        a2b.spines[s].set_visible(False)

    savefig(fig, out, "track_ground")
    plt.close(fig)
    print("  wrote track_ground.%s" % FIGURE_FORMAT)


def fit_circle(x, y):
    """Algebraic least-squares circle fit. Returns (cx, cy, r, rms)."""
    if len(x) < 20:
        return None, None, None, None
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None, None, None, None
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    resid = np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2))
    return cx, cy, r, resid


def fig_axes(d, t, out):
    """Commanded vs achieved for all six degrees of freedom."""
    fig, axs = plt.subplots(3, 2, figsize=(COL2, 4.6), sharex=True)
    fig.subplots_adjust(left=0.075, right=0.985, top=0.965, bottom=0.085,
                        hspace=0.18, wspace=0.19)

    panels = [
        (axs[0][0], "px_t", "px", "nav $x$ [m]"),
        (axs[1][0], "py_t", "py", "nav $y$ [m]"),
        (axs[2][0], "alt_t", "alt", "altitude [m]"),
        (axs[0][1], "roll_d", "roll", "roll [deg]"),
        (axs[1][1], "pitch_d", "pitch", "pitch [deg]"),
        (axs[2][1], "yaw_t", "yaw", "heading [deg]"),
    ]
    for ax, cmd, meas, label in panels:
        ax.plot(t, d[cmd], color=CMD, lw=0.8, ls="--", label="commanded")
        ax.plot(t, d[meas], color=MEAS, lw=0.8, label="achieved")
        ax.set_ylabel(label)
        tidy(ax)
        ax.set_xlim(t[0], t[-1])
        err = d[meas] - d[cmd]
        ax.set_title("rms err %.3f" % np.sqrt(np.mean(err ** 2)),
                     fontsize=6.5, loc="right", color=GREY, pad=1.5)

    axs[0][0].legend(frameon=False, ncol=2, loc="upper left")
    axs[2][0].set_xlabel("time [s]")
    axs[2][1].set_xlabel("time [s]")

    savefig(fig, out, "track_axes")
    plt.close(fig)
    print("  wrote track_axes.%s" % FIGURE_FORMAT)


def fig_chain(d, t, out, axis="roll"):
    """
    One axis, every stage of the cascade stacked. This is the figure that
    shows where a tracking error is introduced: each panel's command is the
    panel above it doing its job.
    """
    if axis == "roll":
        stick, vel_t, vel = "rc_roll", "vbx_t", "vbx"
        ang_t, ang = "roll_d", "roll"
        rate_t, rate = "wy_t", "wy"
        torque = "tau_y"
        name = "ROLL  (about +Y)"
    else:
        stick, vel_t, vel = "rc_pitch", "vby_t", "vby"
        ang_t, ang = "pitch_d", "pitch"
        rate_t, rate = "wx_t", "wx"
        torque = "tau_x"
        name = "PITCH  (about +X)"

    fig, axs = plt.subplots(5, 1, figsize=(COL1, 6.0), sharex=True)
    fig.subplots_adjust(left=0.175, right=0.975, top=0.955, bottom=0.075,
                        hspace=0.15)
    fig.text(0.5, 0.985, name, ha="center", fontsize=8)

    axs[0].plot(t, d[stick], color=RC, lw=0.8)
    axs[0].set_ylabel("stick\n[-1, 1]")

    axs[1].plot(t, d[vel_t], color=CMD, lw=0.8, ls="--", label="target")
    axs[1].plot(t, d[vel], color=MEAS, lw=0.8, label="measured")
    axs[1].set_ylabel("body vel\n[m/s]")
    axs[1].legend(frameon=False, ncol=2, loc="upper right")

    axs[2].plot(t, d[ang_t], color=CMD, lw=0.8, ls="--")
    axs[2].plot(t, d[ang], color=MEAS, lw=0.8)
    axs[2].set_ylabel("angle\n[deg]")

    axs[3].plot(t, d[rate_t], color=CMD, lw=0.8, ls="--")
    axs[3].plot(t, d[rate], color=MEAS, lw=0.8)
    axs[3].set_ylabel("body rate\n[deg/s]")

    axs[4].plot(t, d[torque], color=MEAS, lw=0.8)
    axs[4].set_ylabel("torque\n[N$\\cdot$m]")
    axs[4].set_xlabel("time [s]")

    for a in axs:
        tidy(a)
        a.set_xlim(t[0], t[-1])

    savefig(fig, out, "track_chain_%s" % axis)
    plt.close(fig)
    print("  wrote track_chain_%s.%s" % (axis, FIGURE_FORMAT))


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--out", default=None,
                    help="output directory (default: alongside the log)")
    ap.add_argument("--span", nargs=2, type=float, metavar=("T0", "T1"),
                    help="restrict to this time window, seconds from start")
    ap.add_argument("--airborne", action="store_true",
                    help="restrict to Climb and Hover rows")
    ap.add_argument("--png", action="store_true",
                    help="write PNG instead of PDF, for the web")
    ap.add_argument("--dpi", type=int, default=200,
                    help="raster resolution when --png is used")
    args = ap.parse_args()

    global FIGURE_FORMAT, FIGURE_DPI
    if args.png:
        FIGURE_FORMAT = "png"
    FIGURE_DPI = args.dpi

    out = args.out or os.path.dirname(os.path.abspath(args.logfile))
    os.makedirs(out, exist_ok=True)

    print("reading", args.logfile)
    d, meta = load(args.logfile)
    if meta:
        print("  header meta     :",
              " ".join("%s=%s" % kv for kv in sorted(meta.items())))
    """
    Select BEFORE validating.

    Validation is what tells you whether the data is fit to plot, so it has to
    run on the rows that will actually be plotted. Validating the whole file
    and then trimming meant `--span` could not be used to answer the very
    complaint validation had just raised - a capture holding two logging
    sessions reported jitter no matter which window you asked for.
    """
    t_all = (d["t_ms"] - d["t_ms"][0]) / 1000.0

    keep = np.ones(len(t_all), dtype=bool)
    if args.airborne:
        keep &= airborne_mask(d)
    if args.span:
        keep &= (t_all >= args.span[0]) & (t_all <= args.span[1])
    if keep.sum() < 10:
        sys.exit("Selection left %d rows; nothing to plot." % keep.sum())
    if not keep.all():
        print("  selected        : %d of %d rows" % (keep.sum(), len(t_all)))
        d = {k: v[keep] for k, v in d.items()}

    t = validate(d)

    print("writing figures to", out)
    fig_overview(d, t, out)
    fig_ground(d, t, out)
    fig_axes(d, t, out)
    fig_chain(d, t, out, "roll")
    fig_chain(d, t, out, "pitch")


if __name__ == "__main__":
    main()
