#!/usr/bin/env python3
"""
Record one flight's stick commands, start to finish, with one command.

    python record_run.py            start recording; Ctrl+C when you land

Opens the transmitter, starts the track log, waits, stops it on Ctrl+C, and
writes the four RC channels to a numbered text file. There is nothing to type
at the board and no serial commands to remember - the point is that this runs
between "props on" and "props off" without asking you to think about it.

    runs/run_007.txt        the stick commands, one row per sample
    runs/run_007_raw.log    everything the port produced, kept in case

The raw log is written first and never deleted. If anything downstream fails
the flight is still on disk, and `analyze_track.py` will read it.

By default this talks to the TRANSMITTER, because that is the board on your
desk during an untethered flight - the drone relays its log over ESP-NOW.
Use --drone if the drone itself is on USB, on the bench with props off.
"""

import argparse
import datetime
import pathlib
import re
import subprocess
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. Install with:  pip install pyserial")

BAUD_RECEIVER = 460800      # Receiver_RotorRate.ino
BAUD_TRANSMITTER = 921600   # Transmitter_RotorRate.ino

HERE = pathlib.Path(__file__).resolve().parent
RUN_DIR = HERE / "runs"

# The ESP32 is reset when the port opens; it prints its banner and reaches the
# command prompt well inside this. Sending `tracklog` any earlier is sent into
# a bootloader that discards it, and the flight then records nothing.
BOOT_WAIT_S = 2.5

# After `tracklog off` the board still has queued rows plus its closing
# "#TRK end" line to emit.
DRAIN_S = 2.0


def pick_port(explicit):
    if explicit:
        return explicit
    try:
        import serial.tools.list_ports as lp
    except ImportError:
        sys.exit("Cannot list serial ports. Pass one explicitly: "
                 "python record_run.py --port COM5")
    ports = list(lp.comports())
    if not ports:
        sys.exit("No serial ports found. Is the transmitter plugged in?")
    if len(ports) == 1:
        print("using %s  (%s)" % (ports[0].device, ports[0].description))
        return ports[0].device
    print("More than one serial port is present, so pick one:")
    for p in ports:
        print("   --port %-6s %s" % (p.device, p.description))
    sys.exit(1)


def next_run_number():
    """Highest run_NNN already present, plus one. Gaps are left alone: a
    deleted run must not cause the next flight to overwrite a later one."""
    RUN_DIR.mkdir(exist_ok=True)
    highest = 0
    for path in RUN_DIR.glob("run_*.txt"):
        m = re.match(r"run_(\d+)\.txt$", path.name)
        if m:
            highest = max(highest, int(m.group(1)))
    return highest + 1


def main():
    ap = argparse.ArgumentParser(
        description="Record one flight's transmitter stick commands.")
    ap.add_argument("--port", default=None,
                    help="serial port, e.g. COM5. Auto-detected if only one.")
    ap.add_argument("--drone", action="store_true",
                    help="the DRONE is on USB, not the transmitter. Bench "
                         "use, props off.")
    ap.add_argument("--rate", type=int, default=None, metavar="HZ",
                    help="log rate; default is the firmware's 50 Hz. Drop to "
                         "25 if a previous run reported drops.")
    ap.add_argument("--note", default=None,
                    help="one line recorded at the top of the run file, e.g. "
                         '--note "circle, breezy"')
    ap.add_argument("--no-reset", dest="no_reset", action="store_true",
                    help="attach without resetting the board. Skips the boot "
                         "wait; use if the board is already up and armed.")
    args = ap.parse_args()

    port = pick_port(args.port)
    baud = BAUD_RECEIVER if args.drone else BAUD_TRANSMITTER
    which = "drone" if args.drone else "transmitter"

    run_no = next_run_number()
    raw_path = RUN_DIR / ("run_%03d_raw.log" % run_no)
    out_path = RUN_DIR / ("run_%03d.txt" % run_no)

    try:
        if args.no_reset:
            link = serial.Serial()
            link.port = port
            link.baudrate = baud
            link.timeout = 0.1
            link.dtr = False
            link.rts = False
            link.open()
        else:
            link = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as exc:
        print("Could not open %s: %s" % (port, exc))
        print("If the port exists, something else may hold it open - close "
              "the Arduino serial monitor.")
        sys.exit(1)

    handle = open(raw_path, "w", encoding="utf-8", errors="replace")
    lock = threading.Lock()
    stop = threading.Event()
    counts = {"track": 0, "end_line": None, "text_seen": False}

    def reader():
        while not stop.is_set():
            try:
                chunk = link.readline()
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue
            text = chunk.decode("utf-8", errors="replace")
            with lock:
                handle.write(text)
                handle.flush()      # an unplug must not cost the flight
            stripped = text.strip()
            if stripped.startswith("TRK,"):
                counts["track"] += 1
            elif stripped.startswith("#TRK end"):
                counts["end_line"] = stripped
            if stripped and not stripped.startswith(("TRK,", "ST=")):
                counts["text_seen"] = True

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()

    def send(cmd):
        # Whole lines only. A partial write that gets concatenated with the
        # next one is how `tracklog off` once arrived as `tracklog offtracklog
        # off`, which parsed as a rate of 0 rather than as "off".
        with lock:
            handle.write(">>> %s\n" % cmd)
            handle.flush()
        link.write(cmd.encode() + b"\n")
        link.flush()

    print("run %d  -  %s on %s @ %d" % (run_no, which, port, baud))

    if not args.no_reset:
        print("waiting %.1fs for the board to boot..." % BOOT_WAIT_S)
        time.sleep(BOOT_WAIT_S)

    if args.rate:
        start_cmd = "tracklog %d" % args.rate
    else:
        start_cmd = "tracklog on" if args.drone else "tracklog"
    send(start_cmd)
    time.sleep(0.4)

    if not counts["text_seen"]:
        print("note: the board has not said anything yet. If this stays at "
              "0 rows, check the port and whether --drone applies.")

    print("")
    print("RECORDING.  Fly.  Ctrl+C when you have landed and disarmed.")
    print("")

    started = time.monotonic()
    last = 0.0
    try:
        while True:
            time.sleep(0.25)
            now = time.monotonic()
            if now - last >= 5.0:
                last = now
                print("  %5.0fs   %d rows" % (now - started, counts["track"]))
    except KeyboardInterrupt:
        print("\nstopping...")

    send("tracklog off")
    time.sleep(DRAIN_S)

    stop.set()
    thread.join(timeout=1.0)
    link.close()
    with lock:
        handle.close()

    print("")
    if counts["end_line"]:
        print(counts["end_line"])
    if counts["track"] == 0:
        print("NO track rows were recorded.")
        print("The raw capture is at %s" % raw_path)
        print("Most likely: wrong board (--drone), wrong port, or the "
              "transmitter was not linked to the drone.")
        sys.exit(1)

    # extract_sticks.py owns the output format; calling it rather than
    # reimplementing keeps the two files from drifting apart.
    cmd = [sys.executable, str(HERE / "extract_sticks.py"),
           str(raw_path), "--out", str(out_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout + result.stderr)
        print("The raw capture is intact at %s" % raw_path)
        sys.exit(1)
    print(result.stdout.strip())

    if args.note:
        # Prepended, so the note is the first thing visible in the file.
        body = out_path.read_text(encoding="utf-8")
        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        out_path.write_text("# run %d   %s\n# note     %s\n%s"
                            % (run_no, stamp, args.note, body),
                            encoding="utf-8")

    print("")
    print("=" * 46)
    print("  RUN %d" % run_no)
    print("  %s" % out_path)
    print("=" * 46)


if __name__ == "__main__":
    main()
