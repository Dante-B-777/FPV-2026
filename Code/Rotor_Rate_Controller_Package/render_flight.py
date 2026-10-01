#!/usr/bin/env python3
"""
Render an animated telemetry panel from a flight track log, for video.

    # plots only, as a standalone movie
    python render_flight.py logs/circle_20260816_143000.log

    # composited beside the flight footage
    python render_flight.py logs/circle.log --video dji_0042.mp4 --sync 12.4

    # transparent PNG sequence to drop on a timeline in an editor
    python render_flight.py logs/circle.log --overlay

Reads the log through analyze_track.load(), so the "#TRKCOLS" header decides
column order here exactly as it does in the analysis. A log captured over the
ESP-NOW downlink (`tracklog net`) and one captured over USB (`tracklog on`)
are the same file format and both work.

PANELS
    ground track      where the aircraft went, with the circle it was flying
    sticks            the two gimbals, drawn as the pilot moved them
    position          x, y, altitude against the target each was holding
    attitude          roll, pitch, yaw against the demand
    rotor speed       per-rotor commanded omega vs the ESC's measured omega
    estimator         ESKF state vs the raw sensor it was fusing

The rotor panel is the one that answers "does the model predict the machine".
w*_sp is the allocator's model-based demand - the speed kF and CT say each
rotor needs for the commanded wrench - and w* is what Bluejay actually
reports. The estimator panel is the different question: ESKF altitude against
the rangefinder, ESKF heading against the magnetometer.

SYNCHRONISATION
    --sync T   log time, in seconds from the first row, that corresponds to
               the FIRST FRAME of --video.

    Get T from a marker visible in both streams. The cleanest is the arm: the
    log records the state transition and the video shows the props spin up.
    Find the video frame where they start moving, note the log time of the
    first row with armed=1, and the difference is T. A wrong T shows up
    immediately as stick motion that leads or lags the aircraft.

DEPENDENCIES
    numpy, matplotlib, and - only for --video and mp4 output - opencv-python.
"""

import argparse
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

# The loader, the validator and the airborne mask are already written and
# already agreed with the firmware about what a column means. Importing them
# keeps exactly one definition of each.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_track import load, validate, airborne_mask, fit_circle


# ---------------------------------------------------------------------------
# appearance
#
# Two palettes. On video the panel is seen for a few seconds at a time from
# across a room, so it uses heavier lines and a dark ground; the print palette
# in analyze_track.py is tuned for a page instead. The role colours are the
# same in both: measured teal, commanded red, pilot input gold.
# ---------------------------------------------------------------------------

THEMES = {
    "dark": dict(
        bg="#12161a", panel="#1a2026", grid="#2c353d",
        text="#e6edf3", dim="#8b98a5",
        meas="#3fd0c9", cmd="#ff6b5a", rc="#ffc857",
        accent="#6ea8fe", warn="#ff4d4d",
    ),
    "light": dict(
        bg="#ffffff", panel="#f4f6f8", grid="#d9dee3",
        text="#1b2733", dim="#66727e",
        meas="#14595c", cmd="#a8382b", rc="#8a6d1f",
        accent="#2f5fa8", warn="#c62828",
    ),
}

ROTOR_NAMES = ["M1 back-right", "M2 back-left", "M3 front-left", "M4 front-right"]


def style_axes(ax, T, label=None, ylabel=None):
    ax.set_facecolor(T["panel"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(T["grid"])
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=T["dim"], labelsize=8, length=3, width=0.8)
    ax.grid(True, color=T["grid"], linewidth=0.5, alpha=0.7)
    ax.set_axisbelow(True)
    if ylabel:
        ax.set_ylabel(ylabel, color=T["dim"], fontsize=8)
    if label:
        ax.set_title(label, color=T["text"], fontsize=9.5,
                     loc="left", pad=5, fontweight="bold")


def wrap180(a):
    """
    Fold a heading into +/-180.

    The firmware already wraps what it logs, so on a real flight this is the
    identity. It is applied anyway because an unwrapped heading - from a
    hand-built log, or from a future build that stops wrapping - would silently
    stretch the attitude panel's y-axis by a factor of five and hide the tilt
    axes, which is exactly the failure this panel exists to make visible.
    """
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def nice_limits(*series, pad=0.12, symmetric=False, floor=None,
                robust=True, pct=0.6):
    """
    A y-range that holds every series with a little air, and never zero.

    Robust by default: the range comes from a percentile band, not from the
    extremes. A single-sample spike - a rate-loop transient, a flow dropout,
    the discontinuity at a mode change - is real data worth keeping in the
    trace, but letting it set the axis compresses the other 99.9% of the
    flight into a flat line and destroys the panel. The spike still draws; it
    just draws off the top.
    """
    if robust:
        lo = min(float(np.nanpercentile(s, pct)) for s in series if len(s))
        hi = max(float(np.nanpercentile(s, 100.0 - pct))
                 for s in series if len(s))
    else:
        lo = min(float(np.nanmin(s)) for s in series if len(s))
        hi = max(float(np.nanmax(s)) for s in series if len(s))
    if not np.isfinite(lo) or not np.isfinite(hi):
        return (-1.0, 1.0)
    if symmetric:
        m = max(abs(lo), abs(hi))
        lo, hi = -m, m
    span = hi - lo
    if span < 1e-9:
        span = max(abs(hi), 1.0)
    if floor is not None and span < floor:
        mid = 0.5 * (lo + hi)
        lo, hi = mid - floor / 2, mid + floor / 2
        span = floor
    return lo - pad * span, hi + pad * span


# ---------------------------------------------------------------------------
# the panel
# ---------------------------------------------------------------------------

class FlightPanel:
    """
    Builds the figure once, then advances a playhead.

    Everything that does not change between frames - the full traces, the
    axes, the labels - is drawn once into a cached background. Each frame only
    redraws the handful of artists that move. Without that, a 60-second render
    at 30 fps is 1800 full figure draws and takes the better part of an hour.
    """

    def __init__(self, data, t, meta, width_px, height_px, theme="dark",
                 dpi=100, trail_s=6.0, title=None, transparent=False):
        self.d = data
        self.t = t
        self.meta = meta
        self.T = THEMES[theme]
        self.trail_s = trail_s

        # Decimation for the live traces.
        #
        # A panel is under a thousand pixels wide, so drawing all 3500 samples
        # of a 70 s log puts three or four samples on every pixel column. The
        # background trace keeps the full resolution; the live overlay is
        # strided down to roughly one sample per pixel, which is invisible in
        # the output and is most of the per-frame cost. The current sample is
        # always appended so the line still ends exactly at the playhead.
        self.stride = max(1, len(t) // 1200)

        self.fig = plt.figure(
            figsize=(width_px / dpi, height_px / dpi), dpi=dpi)
        self.fig.patch.set_facecolor(self.T["bg"])

        # Transparency is decided ONCE, here, not per frame. Changing it
        # later would invalidate the cached background and silently drop the
        # render back to a full redraw per frame - which is the difference
        # between a two-minute overlay and an hour-long one.
        self.transparent = transparent

        # Left column carries the two spatial panels, which need to be square
        # and want to be big; the right column carries the time series.
        gs = self.fig.add_gridspec(
            4, 2, width_ratios=[1.0, 1.35], height_ratios=[1.0, 1.0, 1.0, 1.0],
            left=0.055, right=0.985, top=0.925, bottom=0.055,
            wspace=0.20, hspace=0.55)

        self.ax_ground = self.fig.add_subplot(gs[0:2, 0])
        self.ax_sticks = self.fig.add_subplot(gs[2:4, 0])
        self.ax_pos = self.fig.add_subplot(gs[0, 1])
        self.ax_att = self.fig.add_subplot(gs[1, 1])
        self.ax_rpm = self.fig.add_subplot(gs[2, 1])
        self.ax_est = self.fig.add_subplot(gs[3, 1])

        self.dynamic = []      # artists redrawn every frame
        self._build_ground()
        self._build_sticks()
        self._build_position()
        self._build_attitude()
        self._build_rotors()
        self._build_estimator()
        self._build_readout(title)

        if self.transparent:
            self.fig.patch.set_alpha(0.0)
            for ax in (self.ax_ground, self.ax_sticks, self.ax_pos,
                       self.ax_att, self.ax_rpm, self.ax_est):
                # Not fully clear: the traces need something to sit on or the
                # footage underneath shows through them and both become
                # unreadable. 0.72 keeps the footage legible behind the panel.
                ax.patch.set_alpha(0.72)

        self.fig.canvas.draw()
        self.background = None   # captured on the first render()

    # -- spatial panels ----------------------------------------------------

    def _build_ground(self):
        ax, T, d = self.ax_ground, self.T, self.d
        style_axes(ax, T, "GROUND TRACK", None)

        air = airborne_mask(d)
        px, py = d["px"], d["py"]

        # The whole flight, faint, so the shape is visible before the aircraft
        # gets there. Kept well below the live trail: at 0.45 it competed with
        # the trail and the track read as two overlapping paths.
        ax.plot(px, py, color=T["dim"], lw=0.9, alpha=0.22, zorder=2)

        # The circle the pilot was flying, fitted to the airborne samples
        # only: taxiing and the pre-takeoff sit would drag the centre.
        if air.sum() > 40:
            """
            Only draw the fit if the track is actually a circle.

            A least-squares circle fit always returns something. On a hover it
            returns a small circle through a blob of drift - the 2026-08-18
            hover produced "r = 0.14 m, rms 0.041 m", a 29% residual drawn as
            confidently as a real orbit would be. In a video that is a claim
            about flight path accuracy that the data does not support.

            A residual under 15% of the radius is a recognisable circle; above
            that, say nothing rather than something false.
            """
            cx, cy, r, rms = fit_circle(px[air], py[air])
            circular = (r is not None and np.isfinite(r) and r > 1e-6
                        and rms / r < 0.15)
            if circular and 0.05 < r < 20.0:
                ax.add_patch(Circle((cx, cy), r, fill=False,
                                    ec=T["cmd"], lw=1.0, ls="--",
                                    alpha=0.65, zorder=3))
                ax.text(0.03, 0.03,
                        "fitted circle  r = %.2f m   rms %.3f m" % (r, rms),
                        transform=ax.transAxes, color=T["cmd"],
                        fontsize=8, zorder=6)

        self.g_trail, = ax.plot([], [], color=T["meas"], lw=2.2, zorder=4)
        self.g_dot, = ax.plot([], [], "o", ms=9, mfc=T["meas"],
                              mec=T["bg"], mew=1.5, zorder=5)
        # Nose direction, so yaw is visible on the ground track too.
        self.g_nose, = ax.plot([], [], color=T["text"], lw=2.0, zorder=5)
        self.dynamic += [self.g_trail, self.g_dot, self.g_nose]

        m = 0.25
        ax.set_xlim(px.min() - m, px.max() + m)
        ax.set_ylim(py.min() - m, py.max() + m)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x  [m]  +right", color=T["dim"], fontsize=8)
        ax.set_ylabel("y  [m]  +forward", color=T["dim"], fontsize=8)

    def _build_sticks(self):
        """
        The two gimbals as the pilot sees them.

        Mode 2 layout, which is what the transmitter sketch reads: left stick
        is throttle (vertical) and yaw (horizontal), right stick is pitch
        (vertical) and roll (horizontal). Values are the RAW pre-deadband
        commands from the log, with the deadband drawn as a pale box so a
        stick sitting inside it does not look like a dead radio.
        """
        ax, T = self.ax_sticks, self.T
        style_axes(ax, T, "PILOT INPUT")
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlim(-1.15, 3.35)
        ax.set_ylim(-1.45, 1.35)
        ax.set_aspect("equal", adjustable="box")

        db_rp = float(self.meta.get("deadband_rp", 0.0) or 0.0)
        db_thr = float(self.meta.get("deadband_thr", 0.0) or 0.0)
        db_yaw = float(self.meta.get("deadband_yaw", 0.0) or 0.0)

        self.stick_dots = []
        for i, (cx, name, dbx, dby) in enumerate([
            (0.0, "throttle / yaw", db_yaw, db_thr),
            (2.2, "pitch / roll", db_rp, db_rp),
        ]):
            ax.add_patch(Rectangle((cx - 1.0, -1.0), 2.0, 2.0, fill=False,
                                   ec=T["grid"], lw=1.2))
            if dbx > 0 or dby > 0:
                ax.add_patch(Rectangle(
                    (cx - dbx, -dby), 2 * dbx, 2 * dby,
                    facecolor=T["grid"], alpha=0.5, ec="none"))
            ax.plot([cx - 1.0, cx + 1.0], [0, 0], color=T["grid"], lw=0.7)
            ax.plot([cx, cx], [-1.0, 1.0], color=T["grid"], lw=0.7)
            ax.text(cx, -1.22, name, color=T["dim"], fontsize=8, ha="center")

            dot, = ax.plot([], [], "o", ms=13, mfc=T["rc"],
                           mec=T["bg"], mew=1.5, zorder=5)
            self.stick_dots.append((cx, dot))
            self.dynamic.append(dot)

        self.arm_text = ax.text(
            1.1, 1.16, "", color=T["dim"], fontsize=9,
            ha="center", fontweight="bold")
        self.dynamic.append(self.arm_text)

    # -- time series -------------------------------------------------------

    def _twin(self, ax, ylabel):
        """A right-hand axis sharing x, styled to stay quiet behind the left."""
        T = self.T
        axr = ax.twinx()
        axr.set_facecolor("none")
        axr.grid(False)
        for side in ("top", "left", "bottom"):
            axr.spines[side].set_visible(False)
        axr.spines["right"].set_color(T["grid"])
        axr.spines["right"].set_linewidth(0.8)
        axr.tick_params(colors=T["dim"], labelsize=7.5, length=3, width=0.8)
        axr.set_ylabel(ylabel, color=T["dim"], fontsize=8)
        axr.set_xlim(self.t[0], self.t[-1])
        return axr

    def _legend(self, ax, entries, ncol=4):
        """
        Legend built from proxy lines rather than from the live artists.

        The live artists start empty, and an empty Line2D gives a legend entry
        with no visible swatch until the playhead reaches it - which is the one
        moment the viewer most needs the key.
        """
        from matplotlib.lines import Line2D
        T = self.T
        handles = [Line2D([], [], color=c, ls=ls, lw=1.8) for c, ls, _ in entries]
        labels = [lab for _, _, lab in entries]

        # Above the axes, beside the panel title, rather than inside it. In
        # the data area the key sits on top of exactly the region a hover log
        # spends most of its time in, and no corner is reliably empty across
        # six panels and a whole flight.
        leg = ax.legend(handles, labels, loc="lower right",
                        bbox_to_anchor=(1.0, 1.005), ncol=ncol,
                        frameon=False, fontsize=7.5, handlelength=1.4,
                        columnspacing=1.0, borderaxespad=0.0,
                        labelspacing=0.25)
        for txt in leg.get_texts():
            txt.set_color(T["dim"])
        leg.set_zorder(8)
        return leg

    def _timeseries(self, ax, series, ylabel, title, symmetric=False,
                    floor=None, legend=True):
        """
        One time-series panel.

        The whole flight is drawn faintly so the shape of the manoeuvre is
        visible before the playhead reaches it, and the portion already flown
        is redrawn opaque on top. That reads better on video than a scrolling
        window, which gives the viewer no idea what is coming.
        """
        T = self.T
        style_axes(ax, T, title, ylabel)

        live = []
        for y, color, label, ls in series:
            ax.plot(self.t, y, color=color, lw=0.9, alpha=0.22,
                    ls=ls, zorder=2)
            ln, = ax.plot([], [], color=color, lw=1.8, ls=ls,
                          label=label, zorder=4)
            live.append((ln, y))
            self.dynamic.append(ln)

        cursor = ax.axvline(self.t[0], color=T["text"], lw=1.0,
                            alpha=0.8, zorder=5)
        self.dynamic.append(cursor)

        ax.set_xlim(self.t[0], self.t[-1])
        ax.set_ylim(*nice_limits(*[y for _, y in live],
                                 symmetric=symmetric, floor=floor))
        if legend:
            self._legend(ax, [(c, ls, lab) for _, c, lab, ls in series],
                         ncol=len(series))
        return live, cursor

    def _build_position(self):
        d, T = self.d, self.T
        self.pos_live, self.pos_cursor = self._timeseries(
            self.ax_pos,
            [(d["px"], T["meas"], "x", "-"),
             (d["py"], T["accent"], "y", "-"),
             (d["alt"], T["rc"], "z", "-"),
             (d["alt_t"], T["cmd"], "z target", "--")],
            "m", "POSITION", floor=0.4)

    def _build_attitude(self):
        """
        Roll and pitch against their demand, plus heading.

        Heading gets its own axis. Roll and pitch live inside +/-25 deg on any
        flight that did not end in the tilt cut-out, while yaw sweeps the full
        circle during the choreography - on one shared scale the two tilt axes
        collapse to a flat line and the panel shows nothing.
        """
        d, T = self.d, self.T
        self.att_live, self.att_cursor = self._timeseries(
            self.ax_att,
            [(d["roll"], T["meas"], "roll", "-"),
             (d["roll_d"], T["meas"], "roll dmd", "--"),
             (d["pitch"], T["accent"], "pitch", "-"),
             (d["pitch_d"], T["accent"], "pitch dmd", "--")],
            "deg  tilt", "ATTITUDE", symmetric=True, floor=10.0, legend=False)

        axr = self._twin(self.ax_att, "deg  heading")
        for y, color, label, ls in [
            (wrap180(d["yaw"]), T["rc"], "yaw", "-"),
            (wrap180(d["yaw_t"]), T["cmd"], "yaw target", "--"),
        ]:
            axr.plot(self.t, y, color=color, lw=0.8, alpha=0.22, ls=ls,
                     zorder=2)
            ln, = axr.plot([], [], color=color, lw=1.8, ls=ls, zorder=4)
            self.att_live.append((ln, y))
            self.dynamic.append(ln)
        axr.set_ylim(-190, 190)
        axr.set_yticks([-180, -90, 0, 90, 180])
        self._legend(self.ax_att,
                     [(T["meas"], "-", "roll"), (T["meas"], "--", "roll dmd"),
                      (T["accent"], "-", "pitch"), (T["accent"], "--", "pitch dmd"),
                      (T["rc"], "-", "yaw (right)"), (T["cmd"], "--", "yaw tgt")],
                     ncol=3)

    def _build_rotors(self):
        """
        Model prediction against the machine.

        w*_sp is what the allocator asked for - the speed the thrust model
        says that rotor needs for the commanded wrench - and w* is what the
        ESC measured. Four rotors would be eight lines and unreadable at video
        size, so the panel shows the four measured speeds solid with the four
        demands dashed behind them, and calls out the worst tracking error of
        the moment in the corner. That number is the summary: if the model and
        the rotor loop are honest it stays small everywhere except during
        deliberate steps.
        """
        ax, d, T = self.ax_rpm, self.d, self.T
        style_axes(ax, T, "ROTOR SPEED   model demand vs ESC measured",
                   "rad/s")

        shades = [T["meas"], T["accent"], T["rc"], "#c792ea"]
        self.rpm_live = []
        for i in range(4):
            sp, mv = d["w%d_sp" % (i + 1)], d["w%d" % (i + 1)]
            ax.plot(self.t, mv, color=shades[i], lw=0.8, alpha=0.20, zorder=2)
            lm, = ax.plot([], [], color=shades[i], lw=1.6, zorder=4,
                          label="w%d" % (i + 1))
            ls, = ax.plot([], [], color=shades[i], lw=1.1, ls="--",
                          alpha=0.85, zorder=3)
            self.rpm_live.append((lm, mv, ls, sp))
            self.dynamic += [lm, ls]

        self.rpm_cursor = ax.axvline(self.t[0], color=T["text"], lw=1.0,
                                     alpha=0.8, zorder=5)
        self.dynamic.append(self.rpm_cursor)

        allw = [d["w%d" % (i + 1)] for i in range(4)] + \
               [d["w%d_sp" % (i + 1)] for i in range(4)]
        ax.set_xlim(self.t[0], self.t[-1])
        ax.set_ylim(*nice_limits(*allw, floor=100.0))

        """
        The tracking error, on its own axis.

        At hover all four rotors sit within a few percent of each other, so on
        the speed scale the demand and the measurement lie on top of one
        another and the panel looks impressive while proving nothing. The
        difference is the quantity with information in it: it shows the rotor
        loop's lag during a step and its steady-state offset in between, and
        those are what the RotorSpeedLoop gains are tuned against.
        """
        axr = self._twin(ax, "demand - measured  [rad/s]")
        self.rpm_err_live = []
        for i in range(4):
            err = d["w%d_sp" % (i + 1)] - d["w%d" % (i + 1)]
            axr.plot(self.t, err, color=shades[i], lw=0.6, alpha=0.10,
                     zorder=1)
            ln, = axr.plot([], [], color=shades[i], lw=0.8, alpha=0.55,
                           zorder=3)
            self.rpm_err_live.append((ln, err))
            self.dynamic.append(ln)

        """
        The error strip is deliberately squeezed into the bottom third.

        Given a full-height axis the error - which is mostly ESC measurement
        noise at hover - fills the panel and buries the speeds it is supposed
        to annotate. Stretching the top of its range pushes it down into a
        band, where it reads as a texture whose thickness is the tracking
        quality and whose excursions are the transients.
        """
        allerr = [e for _, e in self.rpm_err_live]
        elo, ehi = nice_limits(*allerr, symmetric=True, floor=40.0)
        axr.set_ylim(elo, ehi + 2.2 * (ehi - elo))
        axr.axhline(0.0, color=T["grid"], lw=0.7, zorder=1)

        # Sits just above the error strip, which occupies the bottom third.
        self.rpm_err = ax.text(
            0.015, 0.35, "", transform=ax.transAxes,
            color=T["dim"], fontsize=8.5, zorder=6)
        self.dynamic.append(self.rpm_err)

        self._legend(ax, [(shades[i], "-", "w%d" % (i + 1)) for i in range(4)]
                     + [(T["dim"], "--", "demand")], ncol=5)

    def _build_estimator(self):
        """
        ESKF state against the raw measurement it was fusing.

        Altitude against the rangefinder, and heading against the
        magnetometer. Both are plotted on one axes with heading scaled into
        the altitude range, because two separate panels would cost a row and
        the interesting thing is the same in both: how far the estimate is
        willing to sit from the sensor.
        """
        ax, d, T = self.ax_est, self.d, self.T
        style_axes(ax, T, "ESTIMATOR   ESKF state vs raw sensor", "m  /  deg")

        # Heading is degrees and altitude is metres. Rather than a twin axis,
        # which doubles the tick clutter at video size, heading is drawn on
        # its own scale and labelled as such.
        self.est_live = []
        for y, color, label, ls in [
            (d["alt"], T["meas"], "ESKF alt [m]", "-"),
            (d["range"], T["cmd"], "rangefinder [m]", "--"),
        ]:
            ax.plot(self.t, y, color=color, lw=0.8, alpha=0.22, ls=ls,
                    zorder=2)
            ln, = ax.plot([], [], color=color, lw=1.8, ls=ls, label=label,
                          zorder=4)
            self.est_live.append((ln, y))
            self.dynamic.append(ln)

        ax.set_xlim(self.t[0], self.t[-1])
        ax.set_ylim(*nice_limits(d["alt"], d["range"], floor=0.4))
        ax.set_xlabel("time  [s]", color=T["dim"], fontsize=8)

        axr = self._twin(ax, "deg  heading")
        self.ax_est_r = axr

        for y, color, ls in [
            (wrap180(d["yaw"]), T["rc"], "-"),
            (wrap180(d["mag_yaw"]), "#c792ea", "--"),
        ]:
            axr.plot(self.t, y, color=color, lw=0.8, alpha=0.22, ls=ls,
                     zorder=2)
            ln, = axr.plot([], [], color=color, lw=1.8, ls=ls, zorder=4)
            self.est_live.append((ln, y))
            self.dynamic.append(ln)

        axr.set_ylim(-190, 190)
        axr.set_yticks([-180, -90, 0, 90, 180])

        self.est_cursor = ax.axvline(self.t[0], color=T["text"], lw=1.0,
                                     alpha=0.8, zorder=5)
        self.dynamic.append(self.est_cursor)

        self._legend(ax, [
            (T["meas"], "-", "ESKF alt [m]"),
            (T["cmd"], "--", "rangefinder [m]"),
            (T["rc"], "-", "ESKF yaw (right)"),
            ("#c792ea", "--", "magnetometer"),
        ], ncol=2)

    def _build_readout(self, title):
        """Header: the clock, the flight state, and anything that is wrong."""
        T = self.T
        if title:
            self.fig.text(0.055, 0.972, title, color=T["text"], fontsize=13,
                          fontweight="bold", va="center")
        self.clock = self.fig.text(
            0.985, 0.972, "", color=T["text"], fontsize=13, ha="right",
            va="center", fontweight="bold", family="monospace")

        # Centred, so it cannot collide with the title on the left or the
        # clock on the right however long the state string grows.
        self.status = self.fig.text(
            0.52, 0.972, "", color=T["dim"], fontsize=9.5, va="center",
            ha="center", family="monospace")
        self.dynamic += [self.clock, self.status]

    # -- playhead ----------------------------------------------------------

    STATE_NAMES = {0: "DISARMED", 1: "ARMING", 2: "CLIMB", 3: "HOVER",
                   4: "LANDING", 5: "STOPPED"}
    HOLD_NAMES = {0: "free", 1: "settling", 2: "HELD"}

    def _upto(self, y, i):
        """Samples 0..i, strided, with sample i always included."""
        st = self.stride
        if st == 1:
            return y[:i + 1]
        head = y[:i + 1:st]
        if (i % st) == 0:
            return head
        return np.append(head, y[i])

    def _update(self, i):
        d, T, t = self.d, self.T, self.t
        now = t[i]
        tt = self._upto(t, i)
        j0 = max(0, np.searchsorted(t, now - self.trail_s))

        self.g_trail.set_data(d["px"][j0:i + 1], d["py"][j0:i + 1])
        self.g_dot.set_data([d["px"][i]], [d["py"][i]])

        # Nose vector. Yaw is CCW-positive about +Z with +y forward, so the
        # heading direction is (-sin, cos) - not the (cos, sin) that a
        # +x-forward convention would give.
        yaw = math.radians(d["yaw"][i])
        span = self.ax_ground.get_xlim()
        L = 0.07 * (span[1] - span[0])
        self.g_nose.set_data(
            [d["px"][i], d["px"][i] - L * math.sin(yaw)],
            [d["py"][i], d["py"][i] + L * math.cos(yaw)])

        for (cx, dot), (xv, yv) in zip(
                self.stick_dots,
                [(d["rc_yaw"][i], d["rc_thr"][i]),
                 (d["rc_roll"][i], d["rc_pitch"][i])]):
            dot.set_data([cx + float(np.clip(xv, -1, 1))],
                         [float(np.clip(yv, -1, 1))])

        armed = d["armed"][i] > 0.5
        self.arm_text.set_text("ARMED" if armed else "disarmed")
        self.arm_text.set_color(T["warn"] if armed else T["dim"])

        for group, cursor in ((self.pos_live, self.pos_cursor),
                              (self.att_live, self.att_cursor),
                              (self.est_live, self.est_cursor)):
            for ln, y in group:
                ln.set_data(tt, self._upto(y, i))
            cursor.set_xdata([now, now])

        worst = 0.0
        for lm, mv, ls, sp in self.rpm_live:
            lm.set_data(tt, self._upto(mv, i))
            ls.set_data(tt, self._upto(sp, i))
            worst = max(worst, abs(float(sp[i] - mv[i])))
        for ln, err in self.rpm_err_live:
            ln.set_data(tt, self._upto(err, i))
        self.rpm_cursor.set_xdata([now, now])
        self.rpm_err.set_text("worst rotor error now: %5.0f rad/s" % worst)

        self.clock.set_text("t = %6.2f s" % now)

        state = self.STATE_NAMES.get(int(d["state"][i]), "state %d"
                                     % int(d["state"][i]))
        bits = ["%s" % state,
                "alt %s" % self.HOLD_NAMES.get(int(d["hold_alt"][i]), "?"),
                "xy %s" % self.HOLD_NAMES.get(int(d["hold_xy"][i]), "?"),
                "hdg %s" % self.HOLD_NAMES.get(int(d["hold_hdg"][i]), "?")]

        # Saturation is the failure mode this airframe actually has, so it is
        # called out rather than left in a column nobody reads.
        sat = []
        if d["sat_yaw_demand"][i] > 0.5: sat.append("yaw demand")
        if d["sat_yaw_alloc"][i] > 0.5: sat.append("yaw alloc")
        if d["sat_rollpitch"][i] > 0.5: sat.append("roll/pitch")
        if sat:
            bits.append("SATURATED: " + ", ".join(sat))

        self.status.set_text("     ".join(bits))
        self.status.set_color(T["warn"] if sat else T["dim"])

    def render(self, i):
        """Return the frame at sample index i as an HxWx4 uint8 RGBA array."""
        canvas = self.fig.canvas
        if self.background is None:
            for artist in self.dynamic:
                artist.set_visible(False)
            canvas.draw()
            self.background = canvas.copy_from_bbox(self.fig.bbox)
            for artist in self.dynamic:
                artist.set_visible(True)

        canvas.restore_region(self.background)
        self._update(i)
        for artist in self.dynamic:
            if artist.axes is not None:
                artist.axes.draw_artist(artist)
            else:
                self.fig.draw_artist(artist)
        canvas.blit(self.fig.bbox)

        return np.asarray(canvas.buffer_rgba()).copy()


# ---------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------

def frame_indices(t, fps, span=None):
    """
    Map each output video frame to the log sample nearest its timestamp.

    Nearest-sample rather than interpolation: an interpolated row would mix
    two control cycles, and the point of the panel is to show what the
    controller actually did on a given cycle.
    """
    t0 = t[0] if span is None else max(t[0], span[0])
    t1 = t[-1] if span is None else min(t[-1], span[1])
    if t1 <= t0:
        sys.exit("Empty time span.")
    n = int(math.floor((t1 - t0) * fps))
    times = t0 + np.arange(n) / float(fps)
    return np.searchsorted(t, times).clip(0, len(t) - 1), times


def open_writer(path, w, h, fps):
    try:
        import cv2
    except ImportError:
        sys.exit("Writing video needs opencv-python:  pip install opencv-python")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (w, h))
    if not writer.isOpened():
        sys.exit("Could not open %s for writing." % path)
    return writer


def render_movie(panel, idx, out_path, fps, video=None, sync=0.0,
                 times=None, layout="right", fit=None):
    """
    Write the panel movie, optionally composited beside the flight footage.

    The footage is resampled by timestamp, not by frame count: its frame rate
    is whatever the camera chose and need not match --fps or the log rate.
    """
    import cv2

    ph, pw = panel.render(idx[0]).shape[:2]

    cap = None
    if video:
        cap = cv2.VideoCapture(video)
        if not cap.isOpened():
            sys.exit("Could not open video %s" % video)
        vfps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        vcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        vw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        vh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Scale the footage to the panel height so the two sit side by side
        # without letterboxing either.
        scale = ph / float(vh)
        tw, th = int(round(vw * scale)), ph
        out_w, out_h = pw + tw, ph
        print("  footage %dx%d @ %.2f fps, %d frames -> %dx%d beside panel"
              % (vw, vh, vfps, vcount, tw, th))
    else:
        out_w, out_h = pw, ph

    # Footage beside a dense panel is natively ultrawide - 16:9 video next to
    # a 1600x1000 panel is 3378x1000, which no player or slide deck wants.
    # --fit scales the finished composite into a delivery resolution, keeping
    # aspect and padding the remainder with the panel background.
    # Letterbox padding takes the panel's background colour, so the bars read
    # as part of the panel rather than as a black border round the video.
    bg = panel.T["bg"].lstrip("#")
    pad_bgr = np.array([int(bg[4:6], 16), int(bg[2:4], 16), int(bg[0:2], 16)],
                       np.uint8)

    fit_to = None
    if fit:
        fw, fh = fit
        sc = min(fw / float(out_w), fh / float(out_h))
        fit_to = (fw, fh, int(round(out_w * sc)), int(round(out_h * sc)))
        out_w, out_h = fw, fh
        print("  fitting %dx%d composite into %dx%d"
              % (fit_to[2], fit_to[3], fw, fh))

    writer = open_writer(out_path, out_w, out_h, fps)

    last_vframe = -1
    cached = None
    for k, i in enumerate(idx):
        frame = panel.render(i)[:, :, :3]             # drop alpha
        frame = frame[:, :, ::-1]                     # OpenCV wants BGR

        if cap is not None:
            # Log time -> video time. sync is the log time of video frame 0.
            vt = times[k] - sync
            want = int(round(vt * vfps))
            if want < 0:
                shot = np.zeros((th, tw, 3), np.uint8)
            else:
                if want != last_vframe:
                    if want != last_vframe + 1:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, want)
                    ok, raw = cap.read()
                    if ok:
                        cached = cv2.resize(raw, (tw, th),
                                            interpolation=cv2.INTER_AREA)
                    last_vframe = want
                shot = cached if cached is not None else \
                    np.zeros((th, tw, 3), np.uint8)

            frame = np.hstack([shot, frame] if layout == "right"
                              else [frame, shot])

        if fit_to is not None:
            fw, fh, iw, ih = fit_to
            small = cv2.resize(frame, (iw, ih), interpolation=cv2.INTER_AREA)
            canvas = np.zeros((fh, fw, 3), np.uint8)
            canvas[:, :] = pad_bgr           # the panel's own background
            y0, x0 = (fh - ih) // 2, (fw - iw) // 2
            canvas[y0:y0 + ih, x0:x0 + iw] = small
            frame = canvas

        writer.write(frame)
        if k % 60 == 0:
            print("    frame %d / %d" % (k, len(idx)), end="\r")

    writer.release()
    if cap is not None:
        cap.release()
    print("    frame %d / %d" % (len(idx), len(idx)))


def render_overlay(panel, idx, out_dir, every=1):
    """
    Transparent PNG sequence, for dropping onto a timeline in an editor.

    PNG rather than a movie because no mp4 codec carries an alpha channel, and
    every editor imports an image sequence.
    """
    import matplotlib.image as mpimg
    os.makedirs(out_dir, exist_ok=True)
    written = 0
    for k, i in enumerate(idx):
        if k % every:
            continue
        rgba = panel.render(i)
        mpimg.imsave(os.path.join(out_dir, "panel_%05d.png" % written), rgba)
        written += 1
        if written % 30 == 0:
            print("    png %d" % written, end="\r")
    print("    png %d written to %s" % (written, out_dir))
    return written


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Render an animated telemetry panel from a flight track "
                    "log, for video.")
    ap.add_argument("logfile")
    ap.add_argument("--out", default=None,
                    help="output file or directory (default: beside the log)")
    ap.add_argument("--video", default=None,
                    help="flight footage to composite beside the panel")
    ap.add_argument("--sync", type=float, default=0.0,
                    help="log time in seconds corresponding to the FIRST "
                         "frame of --video")
    ap.add_argument("--side", choices=["left", "right"], default="left",
                    help="which side the footage goes on (default left)")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--span", nargs=2, type=float, metavar=("T0", "T1"),
                    help="render only this window, in log seconds")
    ap.add_argument("--airborne", action="store_true",
                    help="trim to the airborne portion (Climb and Hover)")
    ap.add_argument("--size", default="1600x1000",
                    help="panel size in pixels, WxH (default 1600x1000)")
    ap.add_argument("--fit", default=None, metavar="WxH",
                    help="scale the finished composite into this resolution, "
                         "e.g. 1920x1080. Without it the output is the panel "
                         "and the footage at native size side by side, which "
                         "is very wide.")
    ap.add_argument("--theme", choices=["dark", "light"], default="dark")
    ap.add_argument("--trail", type=float, default=6.0,
                    help="seconds of ground track drawn behind the aircraft")
    ap.add_argument("--title", default=None)
    ap.add_argument("--overlay", action="store_true",
                    help="write a transparent PNG sequence instead of a movie")
    ap.add_argument("--overlay-every", type=int, default=1,
                    help="write every Nth frame of the PNG sequence")
    args = ap.parse_args()

    data, meta = load(args.logfile)
    print("loaded %d rows from %s" % (len(data["t_ms"]), args.logfile))
    if meta.get("link"):
        print("  captured over: %s" % meta["link"])

    t = validate(data)

    if args.airborne:
        m = airborne_mask(data)
        if m.sum() < 10:
            sys.exit("--airborne left almost nothing. Was the aircraft ever "
                     "in Climb or Hover?")
        data = {k: v[m] for k, v in data.items()}
        t = t[m] - t[m][0]
        print("  airborne only: %d rows, %.1f s" % (m.sum(), t[-1]))

    w, h = (int(x) for x in args.size.lower().split("x"))

    panel = FlightPanel(data, t, meta, w, h, theme=args.theme,
                        trail_s=args.trail, transparent=args.overlay,
                        title=args.title or os.path.basename(args.logfile))

    idx, times = frame_indices(t, args.fps, args.span)
    print("  rendering %d frames at %.1f fps (%.1f s)"
          % (len(idx), args.fps, len(idx) / args.fps))

    stem = os.path.splitext(args.logfile)[0]

    if args.overlay:
        out = args.out or (stem + "_overlay")
        render_overlay(panel, idx, out, every=args.overlay_every)
    else:
        out = args.out or (stem + "_panel.mp4")
        fit = None
        if args.fit:
            fit = tuple(int(x) for x in args.fit.lower().split("x"))
        render_movie(panel, idx, out, args.fps, video=args.video,
                     sync=args.sync, times=times, fit=fit,
                     layout="right" if args.side == "left" else "left")
        print("  wrote %s" % out)


if __name__ == "__main__":
    main()
