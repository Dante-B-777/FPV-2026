#!/usr/bin/env python3
"""
Serial capture for the Rotor Rate boards.

Tees the board's serial output to a timestamped file AND to the console, while
still letting you type commands (kmlearn on, headingkp 0, set ...).

The point is that flight logs are far too long to paste into a chat. Capture
to a file, then hand over the PATH - it can be read and grepped selectively
instead of consuming a whole context window.

    python capture.py                       list the serial ports
    python capture.py COM3 --tag phase8     the DRONE, over USB
    python capture.py COM3 --tx --tag final the TRANSMITTER

The two boards run different baud rates. `--tx` selects the transmitter's, and
is what an untethered flight uses: the drone has no cable, so it sends its
track log to the transmitter over ESP-NOW and the transmitter prints it here.
See VIDEO_FLIGHT_PROCEDURE.md.

Writes to logs/<tag>_YYYYmmdd_HHMMSS.log next to this script.
"""

import argparse
import datetime
import pathlib
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing. Install with:  pip install pyserial")

"""
The two boards run different baud rates, and connecting at the wrong one gives
a screen of mojibake rather than an error - so the rate is selected by which
board you say you are talking to, not by remembering a number.
"""
BAUD_RECEIVER = 460800     # Serial.begin() in Receiver_RotorRate.ino
BAUD_TRANSMITTER = 921600  # Serial.begin() in Transmitter_RotorRate.ino
BAUD_DEFAULT = BAUD_RECEIVER


"""
WHAT GETS HIDDEN FROM THE SCREEN

The file always receives every byte. The screen does not, and that is not a
convenience - it is what makes the session usable at all.

Three streams run fast enough to make the console untypeable:

    ST=...          the drone's 5 Hz status telemetry
    [ 12.345] CMD   the transmitter's per-packet line, 50 Hz
    TRK,...         relayed flight-track rows, 50 Hz

At those rates a typed command scrolls away before it can be read, and on
Windows the terminal's line editing fights the incoming text so the word
cannot be finished. During a tracklog relay that is not cosmetic: you could
not type `tracklog off` to close the log at the end of a flight.

So they are hidden by default and counted instead, on a single rewritten
status line. `--verbose` restores the firehose.
"""

NOISY_PREFIXES = ("ST=", "TRK,")


def is_noisy(text):
    if text.startswith(NOISY_PREFIXES):
        return True
    # The transmitter's periodic line: "[   12.345] CMD T:... "
    return text.startswith("[") and "] CMD " in text


def list_ports():
    """Every serial port currently present, for the 'which COM port' problem."""
    try:
        import serial.tools.list_ports as list_ports_module
    except ImportError:
        return []
    return list(list_ports_module.comports())


def print_ports():
    ports = list_ports()
    if not ports:
        print("No serial ports found. Is the board plugged in?")
        return
    print("Available ports:")
    for p in ports:
        print(f"  {p.device}  {p.description}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default=None,
                        help="serial port, e.g. COM3. Omit to list them.")
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument(
        "--tx", "--transmitter", dest="transmitter", action="store_true",
        help="talking to the TRANSMITTER, not the drone. Sets 921600 baud. "
             "This is what an untethered `tracklog` flight uses - the "
             "transmitter relays the drone's log to this port.",
    )
    parser.add_argument(
        "--no-reset", dest="no_reset", action="store_true",
        help="attach without pulsing DTR/RTS, so the board is NOT reset. Use "
             "this to inspect a drone that has been powered for a while - a "
             "normal connect resets it and hides any state that accumulated.",
    )
    parser.add_argument("--tag", default="flight", help="filename prefix")
    parser.add_argument(
        "--verbose", action="store_true",
        help="show every line on screen, including the 50 Hz streams. The "
             "console becomes untypeable - use only for debugging.",
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="accepted for compatibility; hiding the high-rate streams is "
             "now the default. See --verbose.",
    )
    args = parser.parse_args()

    if args.port is None:
        print_ports()
        print("\nUsage:  python capture.py COM3 --tag final          (drone)")
        print("        python capture.py COM3 --tx --tag final      "
              "(transmitter, untethered log)")
        return

    if args.baud is None:
        args.baud = BAUD_TRANSMITTER if args.transmitter else BAUD_RECEIVER

    log_dir = pathlib.Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(exist_ok=True)

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = log_dir / f"{args.tag}_{stamp}.log"

    try:
        if args.no_reset:
            """
            Attach WITHOUT resetting the board.

            Opening a CP210x port normally pulses DTR/RTS, which resets the
            ESP32. That makes a whole class of fault unobservable: if a sensor
            degrades over a long powered session, the act of connecting to look
            at it restores it. On 2026-08-18 every capture with healthy
            rangefinder and flow was one that had just reset the drone, and
            every capture with a dead sensor was taken from the transmitter -
            so "recently reset" and "logging over the radio" were the same
            column and could not be told apart.

            Setting the lines before open() suppresses the pulse, so the board
            keeps running and its accumulated state can be inspected.
            """
            link = serial.Serial()
            link.port = args.port
            link.baudrate = args.baud
            link.timeout = 0.1
            link.dtr = False
            link.rts = False
            link.open()
        else:
            link = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        # The usual causes are a placeholder left in the command line, a board
        # that is not plugged in, and a port already held by the Arduino IDE's
        # serial monitor. Listing what IS there answers all three faster than
        # the exception does.
        print(f"Could not open {args.port}: {exc}\n")
        print_ports()
        print("\nIf the port exists, something else may already have it open "
              "- close the Arduino serial monitor.")
        sys.exit(1)

    which = "TRANSMITTER" if args.transmitter else "drone"
    print(f"logging {which} on {args.port} @ {args.baud} -> {log_path}")
    print("type commands here (gains need 'set', e.g. set headingkp 0)")
    if args.verbose:
        print("VERBOSE: showing every line. The console will be untypeable.")
    else:
        print("high-rate streams (ST=, TRK, and the 50 Hz CMD line) are hidden")
        print("so this stays typeable. Everything is still saved to the file.")
    print("Ctrl+C to stop\n")

    stop = threading.Event()

    # One writer, two producers: the reader thread appends board output, the
    # main thread appends the commands you type. Both go through this lock so
    # a typed command lands in the file between the lines it actually sat
    # between - which is what makes the log a record of the session rather
    # than just of the board.
    handle = open(log_path, "w", encoding="utf-8", errors="replace")
    handle_lock = threading.Lock()

    def emit(text, to_screen=True):
        """
        NEVER use '\\r' or any in-place erase here.

        The terminal is in cooked mode: the characters you are part-way
        through typing live in ITS buffer, and are echoed on the current line.
        A '\\r' + spaces wipes that echo from the screen while leaving the
        buffer intact, so the half-typed command becomes invisible. The
        natural response is to type it again - and both copies are then
        submitted as ONE line.

        That is exactly what happened on 2026-08-18: `tracklog off` arrived at
        the transmitter as `tracklog offtracklog off`, which is not "off", so
        it was parsed as a rate, atol() gave 0, and the relay restarted at
        1 Hz instead of stopping.

        Whole lines only, appended. The typed line may get visually split by
        incoming output, but what the terminal submits stays correct.
        """
        if to_screen:
            sys.stdout.write(text if text.endswith("\n") else text + "\n")
            sys.stdout.flush()
        with handle_lock:
            handle.write(text)
            handle.flush()   # a crash or an unplug must not cost the log

    counts = {"track": 0, "noise": 0, "last_report": 0.0,
              "bytes": 0, "junk": 0, "warned": False}

    def check_baud(raw):
        """
        Catch a baud mismatch, loudly, in the first few kilobytes.

        The two boards run at different rates and a wrong one does not fail -
        it delivers plausible-looking bytes that decode to nothing. On
        2026-08-18 that produced a 24 kB capture of pure mojibake, which read
        as a dead sensor rather than a wrong flag.

        The test is NOT a ratio of unprintable bytes. Every ESP32 boot dumps
        its 115200-baud bootloader chatter first, so a correct capture also
        starts with a burst of junk - measured at 33% over the first 4 kB,
        against 70% for the genuinely-wrong rate. Too close to separate.

        What does separate cleanly is whether ANY line looks like readable
        text. Measured over the same captures: 52 and 53 readable lines for
        the correct rate, and 0 for the wrong one. A board at the right baud
        prints sentences within its first few lines; a wrong one essentially
        never produces one by chance.
        """
        if counts["warned"] or counts["bytes"] > 6000:
            return

        counts["bytes"] += len(raw)
        stripped = raw.rstrip(b"\r\n")
        if len(stripped) >= 8:
            printable = sum(1 for b in stripped if 32 <= b <= 126)
            if printable >= 8 and printable >= 0.85 * len(stripped):
                counts["junk"] = -1        # sentinel: real text seen, done

        if counts["junk"] == -1:
            counts["warned"] = True        # healthy; stop checking
            return

        if counts["bytes"] >= 3000:
            counts["warned"] = True
            other = BAUD_RECEIVER if args.baud == BAUD_TRANSMITTER \
                else BAUD_TRANSMITTER
            flag = "drop --tx" if args.transmitter else "add --tx"
            print("\n*** %d bytes in and not one readable line. This is almost "
                  "certainly the" % counts["bytes"])
            print("    wrong baud rate: you are at %d, the other board uses "
                  "%d." % (args.baud, other))
            print("    Stop (Ctrl+C), and %s.\n" % flag)

    def reader():
        while not stop.is_set():
            try:
                chunk = link.readline()
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue
            check_baud(chunk)
            text = chunk.decode("utf-8", errors="replace")

            # The FILE always gets everything, always.
            noisy = (not args.verbose) and is_noisy(text)
            emit(text, to_screen=not noisy)

            if not noisy:
                continue

            if text.startswith("TRK,"):
                counts["track"] += 1
            else:
                counts["noise"] += 1

            # A plain appended line, rarely, so you can see the relay is alive
            # without the rows themselves making the console untypeable.
            # Every 5 s rather than twice a second: this scrolls now that it
            # cannot erase, and the point is reassurance, not a readout.
            now = time.monotonic()
            if now - counts["last_report"] >= 5.0:
                counts["last_report"] = now
                sys.stdout.write(
                    "  ... %d track rows, %d status lines (hidden; all in "
                    "the file)\n" % (counts["track"], counts["noise"]))
                sys.stdout.flush()

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()

    try:
        # readline() in an explicit loop, NOT `for line in sys.stdin`.
        #
        # Iterating a file object uses an internal read-ahead buffer, so on
        # Windows a typed line sits in that buffer instead of being written to
        # the port - the command is silently never sent, and the only symptom
        # is that nothing happens. This cost a whole bench session.
        while True:
            line = sys.stdin.readline()
            if not line:            # EOF, e.g. piped input finished
                break

            text = line.strip()
            if not text:
                continue

            # Echoed into the log as well as the screen, marked, so it is
            # obvious afterwards which lines came from the pilot and which
            # came from the board.
            emit(f">>> {text}\n")
            link.write(text.encode() + b"\n")
            link.flush()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=1.0)
        link.close()
        with handle_lock:
            handle.close()
        print(f"\nsaved: {log_path}")


if __name__ == "__main__":
    main()
