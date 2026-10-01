#!/usr/bin/env python3
"""
Pull just the transmitter stick commands out of a flight track log.

    python extract_sticks.py logs/circle_20260818_170745.log
    python extract_sticks.py logs/circle.log --airborne --out sticks.csv

Writes one row per logged sample: a timestamp and the four normalised RC
channels the transmitter actually sent, and nothing else. The full track log
is 68 columns wide and unreadable in a text editor; this is the four columns
you want when the question is "what did the pilot ask for, and when".

Reads the "#TRKCOLS" header out of the capture rather than hard-coding column
positions, same as analyze_track.py - a firmware change that adds a field
must not silently shift the stick columns.

Deliberately depends on nothing outside the standard library, so it runs on a
laptop in the field with no environment set up.

Output columns:

    t_s       seconds from the first sample of the session
    t_ms      the board's raw millis(), for lining up against other logs
    seq       sample sequence number; a gap here is a dropped row
    thr       throttle    [-1, 1]  (post-deadband, as transmitted)
    roll      roll stick  [-1, 1]  +right
    pitch     pitch stick [-1, 1]  +forward
    yaw       yaw stick   [-1, 1]  +counter-clockwise seen from above

--state adds armed, state, and link, which is how you tell a stick input the
aircraft acted on from one it ignored because it was disarmed.
"""

import argparse
import os
import sys


STICKS = ["rc_thr", "rc_roll", "rc_pitch", "rc_yaw"]
SHORT = {"rc_thr": "thr", "rc_roll": "roll",
         "rc_pitch": "pitch", "rc_yaw": "yaw"}
EXTRA = ["armed", "state", "link"]

# From the firmware's state machine. Only these two are airborne.
STATE_CLIMB = 3
STATE_HOVER = 4


def read_sessions(path):
    """Split a capture into tracklog sessions.

    A capture can hold more than one, because the console session outlives any
    single `tracklog on` / `tracklog off` pair. Each "#TRKCOLS" line starts a
    new one; rows before the first header have no column names and are
    unreadable, so they are dropped rather than guessed at.
    """
    sessions = []
    current = None
    # The "#TRK key=value" banner is printed just BEFORE the column header,
    # so metadata has to be held until the session it describes appears.
    pending = {}

    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()

            if line.startswith("#TRKCOLS,"):
                current = {"cols": line.split(",")[1:], "rows": [],
                           "meta": pending}
                pending = {}
                sessions.append(current)
                continue

            if line.startswith("#TRK "):
                target = current["meta"] if current is not None else pending
                for token in line[5:].split():
                    if "=" not in token:
                        continue
                    k, v = token.split("=", 1)
                    if k and (k[0].isalpha() or k[0] == "_") and \
                            all(c.isalnum() or c == "_" for c in k):
                        target[k] = v
                continue

            if line.startswith("TRK,") and current is not None:
                current["rows"].append(line.split(",")[1:])

    return sessions


def to_float(text):
    """Track logs carry one hex field (rpm_mask); everything read here is
    decimal, but a stray non-numeric token should not abort the whole run."""
    try:
        return float(text)
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser(
        description="Extract transmitter stick commands from a track log.")
    ap.add_argument("logfile")
    ap.add_argument("--out", default=None,
                    help="output path (default: <logfile>_sticks.csv)")
    ap.add_argument("--session", type=int, default=None, metavar="N",
                    help="which tracklog session to use, 1-based, when the "
                         "capture holds more than one. Default: all of them.")
    ap.add_argument("--airborne", action="store_true",
                    help="keep only rows in Climb or Hover")
    ap.add_argument("--span", nargs=2, type=float, metavar=("T0", "T1"),
                    help="keep only rows with t_s between T0 and T1")
    ap.add_argument("--state", action="store_true",
                    help="also emit armed, state, and link")
    ap.add_argument("--tsv", action="store_true",
                    help="tab-separated instead of comma-separated")
    ap.add_argument("--bare", action="store_true",
                    help="omit the leading # comment block")
    args = ap.parse_args()

    if not os.path.exists(args.logfile):
        sys.exit("No such file: %s" % args.logfile)

    sessions = read_sessions(args.logfile)
    if not sessions:
        sys.exit("No #TRKCOLS header in this capture. It was recorded before "
                 "`tracklog` was started, so it holds no track rows - only "
                 "the 5 Hz status telemetry, which carries no timestamp.")

    if args.session is not None:
        if not 1 <= args.session <= len(sessions):
            sys.exit("--session %d out of range; this capture has %d."
                     % (args.session, len(sessions)))
        sessions = [sessions[args.session - 1]]
    elif len(sessions) > 1:
        print("note: %d tracklog sessions in this capture; emitting all of "
              "them. Use --session N to pick one." % len(sessions))

    want = list(STICKS)
    if args.state:
        want += EXTRA
    elif args.airborne:
        # Needed to evaluate the filter, but not emitted.
        want.append("state")

    # t_s restarts at zero for each session, so without this column a
    # multi-session file looks like time running backwards.
    multi = len(sessions) > 1

    out_rows = []
    gaps = 0
    dropped_rows = 0
    meta = {}

    for s_i, sess in enumerate(sessions, 1):
        cols = sess["cols"]
        meta = meta or sess["meta"]

        missing = [c for c in ["t_ms", "seq"] + want if c not in cols]
        if missing:
            sys.exit("Session %d is missing column(s): %s. This log was "
                     "written by a different firmware version than this "
                     "script expects." % (s_i, ", ".join(missing)))

        idx = {name: cols.index(name) for name in ["t_ms", "seq"] + want}
        width = len(cols)

        t0 = None
        prev_seq = None

        for raw in sess["rows"]:
            if len(raw) != width:
                # Serial corruption truncates a row mid-line. A partial row
                # would silently shift every field after the break.
                dropped_rows += 1
                continue

            vals = {name: to_float(raw[i]) for name, i in idx.items()}
            if any(v is None for v in vals.values()):
                dropped_rows += 1
                continue

            if t0 is None:
                t0 = vals["t_ms"]

            seq = int(vals["seq"])
            if prev_seq is not None and seq != prev_seq + 1:
                gaps += 1
            prev_seq = seq

            t_s = (vals["t_ms"] - t0) / 1000.0

            if args.span and not (args.span[0] <= t_s <= args.span[1]):
                continue
            if args.airborne and \
                    int(vals["state"]) not in (STATE_CLIMB, STATE_HOVER):
                continue

            row = []
            if multi:
                row.append("%d" % s_i)
            row += ["%.3f" % t_s, "%d" % int(vals["t_ms"]), "%d" % seq]
            row += ["%+.4f" % vals[c] for c in STICKS]
            if args.state:
                row += ["%d" % int(vals[c]) for c in EXTRA]
            out_rows.append(row)

    if not out_rows:
        sys.exit("No rows survived the filters.")

    sep = "\t" if args.tsv else ","
    ext = ".tsv" if args.tsv else ".csv"
    out = args.out or (os.path.splitext(args.logfile)[0] + "_sticks" + ext)

    header = (["session"] if multi else []) \
        + ["t_s", "t_ms", "seq"] + [SHORT[c] for c in STICKS]
    if args.state:
        header += EXTRA

    with open(out, "w", newline="\n") as fh:
        if not args.bare:
            fh.write("# transmitter stick commands\n")
            fh.write("# source   %s\n" % os.path.basename(args.logfile))
            fh.write("# samples  %d\n" % len(out_rows))
            for k in ("rate_hz", "deadband_rp", "deadband_thr",
                      "deadband_yaw", "link"):
                if k in meta:
                    fh.write("# %-8s %s\n" % (k, meta[k]))
            fh.write("# channels are normalised to [-1, 1] after deadband, "
                     "exactly as transmitted\n")
        fh.write(sep.join(header) + "\n")
        for row in out_rows:
            fh.write(sep.join(row) + "\n")

    print("wrote %s" % out)
    print("  %d samples" % len(out_rows))
    if meta.get("rate_hz"):
        print("  logged at %s Hz" % meta["rate_hz"])
    if gaps:
        print("  WARNING: %d sequence gap(s) - rows were dropped in flight, "
              "so the sampling is not uniform" % gaps)
    if dropped_rows:
        print("  WARNING: %d unparseable row(s) discarded (serial corruption)"
              % dropped_rows)
    if not gaps and not dropped_rows:
        print("  sequence continuous, no corrupt rows")


if __name__ == "__main__":
    main()
