#!/usr/bin/env python3
"""
Serial logger for Transmitter_RotorRate.

Why this exists
---------------
Arduino IDE 2's Serial Monitor can show a timestamp column, but the column is
a rendering of the receive time, not part of the text - so its Copy button
drops it, which is exactly the problem this replaces. The monitor also keeps
only a bounded scrollback and has no "save to file" at all, so a long logging
run silently loses its head.

This writes every line straight to disk as it arrives, with the timestamp in
the text where it cannot be lost. The transmitter also stamps its own lines
with millis(), so each row carries both clocks:

    2026-08-17 14:03:22.481  +12.487  [    12.487] CMD T:+0.000 ...
    ^ host wall clock        ^ host   ^ device millis()
                               elapsed

Comparing the host elapsed column against the device column is how you spot
USB buffering: the device column is evenly spaced at 20 ms, the host column
will occasionally jump and then catch up in a burst.

The Arduino IDE Serial Monitor must be CLOSED - Windows gives one process
exclusive ownership of a COM port.

Usage
-----
    python serial_log.py                      auto-detect port, 921600
    python serial_log.py --port COM7
    python serial_log.py --baud 115200        if you did not raise SERIAL_BAUD
    python serial_log.py --out run3.log
    python serial_log.py --quiet              log to file only, no echo

Type a command (help, verbose, rate 0, calibrate, link) and press Enter to
send it to the transmitter; sent lines are logged too, marked with '>>'.

To stop: type `quit` and press Enter, or press Ctrl-C. Every line is already
flushed to disk as it arrives, so closing the window loses nothing either.
"""

import argparse
import datetime
import sys
import threading
import time

import serial
import serial.tools.list_ports


# USB-serial bridges used on ESP32 dev boards, by VID:PID.
KNOWN_BRIDGES = {
    (0x10C4, 0xEA60): "CP2102",
    (0x1A86, 0x7523): "CH340",
    (0x1A86, 0x55D4): "CH9102",
    (0x0403, 0x6001): "FT232R",
    (0x303A, 0x1001): "ESP32-S3 native USB",
}


def pick_port(explicit):
    ports = list(serial.tools.list_ports.comports())

    if explicit:
        return explicit

    if not ports:
        sys.exit("No serial ports found. Is the transmitter plugged in?")

    preferred = [p for p in ports if (p.vid, p.pid) in KNOWN_BRIDGES]

    if len(preferred) == 1:
        p = preferred[0]
        print("Using %s (%s)" % (p.device, KNOWN_BRIDGES[(p.vid, p.pid)]))
        return p.device

    print("Multiple candidate ports:")
    for p in ports:
        print("   %-8s %s" % (p.device, p.description))
    sys.exit("Pick one explicitly with --port COMx")


def reader(port, handle, quiet, started):
    """Read lines forever, stamp them, write them through to disk."""
    while True:
        try:
            raw = port.readline()
        except serial.SerialException as exc:
            stamp(handle, quiet, started, "!! serial error: %s" % exc)
            return

        if not raw:
            continue

        # errors="replace" so a garbled byte from a baud mismatch or a reset
        # never kills a long logging run.
        text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        stamp(handle, quiet, started, text)


def stamp(handle, quiet, started, text):
    now = datetime.datetime.now()
    line = "%s  %+9.3f  %s" % (
        now.strftime("%Y-%m-%d %H:%M:%S.") + "%03d" % (now.microsecond // 1000),
        time.monotonic() - started,
        text,
    )

    # Flush every line. A logging run usually ends with Ctrl-C or by yanking
    # the USB cable, and buffered tail data would be lost either way.
    handle.write(line + "\n")
    handle.flush()

    if not quiet:
        print(line, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="COM port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=921600,
                        help="must match SERIAL_BAUD in the sketch")
    parser.add_argument("--out", default=None, help="output file")
    parser.add_argument("--quiet", action="store_true",
                        help="write to file only, do not echo")
    args = parser.parse_args()

    device = pick_port(args.port)
    outfile = args.out or ("tx_log_%s.log"
                           % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

    try:
        port = serial.Serial(device, args.baud, timeout=1)
    except serial.SerialException as exc:
        sys.exit("Could not open %s: %s\n"
                 "Close the Arduino IDE Serial Monitor and try again."
                 % (device, exc))

    started = time.monotonic()

    with open(outfile, "w", encoding="utf-8", newline="\n") as handle:
        header = ("# %s @ %d baud, opened %s"
                  % (device, args.baud, datetime.datetime.now().isoformat()))
        handle.write(header + "\n")
        handle.write("# host_wallclock  host_elapsed  device_output\n")
        handle.flush()
        print(header)
        print("Logging to %s   (type `quit` or press Ctrl-C to stop)" % outfile)

        thread = threading.Thread(
            target=reader, args=(port, handle, args.quiet, started))
        thread.daemon = True
        thread.start()

        # input() rather than iterating sys.stdin: on Windows a Ctrl-C
        # arriving during a blocking stdin *iteration* is often not raised
        # until the next newline, which makes the logger feel stuck. input()
        # raises promptly, and `quit` gives a path out that does not depend
        # on signal handling at all.
        try:
            while True:
                try:
                    command = input().strip()
                except EOFError:
                    break

                if not command:
                    continue

                if command in ("quit", "exit", "stop"):
                    break

                port.write((command + "\n").encode())
                stamp(handle, args.quiet, started, ">> " + command)
        except KeyboardInterrupt:
            pass
        finally:
            port.close()
            print("\nStopped. Log written to %s" % outfile)


if __name__ == "__main__":
    main()
