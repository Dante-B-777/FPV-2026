# Final Video Flight — Procedure

Producing one untethered flight that yields both the footage and a track log
good enough to animate beside it.

Everything here is built and compiling. What has **not** happened is a flight.

---

## What the video shows

Four claims, one per panel, all from the same log:

| Panel | Claim | Columns |
|---|---|---|
| Pilot input + ground track | it is remotely controllable | `rc_thr, rc_roll, rc_pitch, rc_yaw`, `px, py` |
| Position / attitude | it goes where it is told | `px py alt` and `roll pitch yaw`, each vs its target |
| Rotor speed | the thrust model predicts the machine | `w*_sp` vs `w*` |
| Estimator | the ESKF agrees with its sensors | `alt` vs `range`, `yaw` vs `mag_yaw` |

Note the third row carefully. **The ESKF does not predict RPM.** `w*_sp` is the
*allocator's* model-based demand — the speed `kF` and `CT` say each rotor needs
for the commanded wrench — and `w*` is what Bluejay measures. That comparison
is what validates the `PROP_THRUST_CT 0.18 → 0.153` correction. The
estimator-vs-measurement comparison is the separate fourth row.

---

## Untethered logging

The drone has no USB cable in flight. The **transmitter does**, and it is
already talking to the drone at 50 Hz — so the log comes back over that link
and the transmitter prints it.

```
receiver: fills AsyncTrackSample on the flight core
       -> debug task on core 0
       -> ESP-NOW downlink (216 B/packet, 10.8 kB/s at 50 Hz)
transmitter: formats the CSV row, prints to USB at 921600 baud
laptop:   capture.py, exactly as before
```

Both sketches compile `TrackSample.h`, so the rows are **byte-identical** to
the tethered ones. `capture.py` and `analyze_track.py` need no changes and
cannot tell the two captures apart.

`ControlData` was left untouched. Logging travels as its own packet type,
dispatched on length and magic, so nothing was added to the packet the
failsafe watches.

### Files changed

| File | Change |
|---|---|
| `Receiver_RotorRate/TrackSample.h` | **new** — sample struct, packet types, CSV formatter, column header |
| `Transmitter_RotorRate/TrackSample.h` | **copy of the above** — keep in sync by hand |
| `AsyncDebugTelemetry.h/.cpp` | struct moved out; `setTrackSink()` / `setTrackSerialEnabled()` |
| `Receiver_RotorRate.ino` | learns the TX MAC, `tracklog net`, downlink send, command packet |
| `Transmitter_RotorRate.ino` | receives and prints rows, `tracklog` command |

> `TrackSample.h` is duplicated because the Arduino build will not follow a
> path out of the sketch folder. If you edit one, copy it to the other — a
> mismatch is caught (`TRACK_PACKET_VERSION`) and reported, not silently
> mis-parsed, but only if you bump the version.

Builds, both clean and warning-free:

| | flash | RAM |
|---|---|---|
| Receiver | 1025889 B (78%) | 54080 B (16%) |
| Transmitter | 919792 B (70%) | 45680 B (13%) |

Up 11 kB flash and 2.5 kB RAM on the receiver from the last clean build. Most
of the RAM is the 12 x 200 B preamble queue: the header burst must not block,
because the transmitter can request logging while the aircraft is armed and
the control step runs at 500 Hz.

---

## Before the day

1. Flash both sketches from the same checkout.
2. **Check `headingkp`, do not assume.** The compiled default is now
   `0.00`, so heading hold is off out of the box and a flash no longer
   re-arms the compass path - the source comment claiming otherwise was
   stale and has been corrected. There is no NVS on the receiver, so every
   gain returns to its compiled value at boot.

   Confirm over the drone's USB before you fly:
   ```
   gains
   ```
   and read back `headingkp=0.000`. If it is not zero, `set headingkp 0` -
   note the `set` prefix, without which the board answers `Unknown command`.

   Gains cannot be changed over the radio, so this is the one step that needs
   the drone's USB. Do it before you walk out, and do not power-cycle the
   drone afterwards if you have changed anything from the compiled defaults.

3. **`calibrate` at the transmitter.** The 2026-08-18 bench capture reports
   `NO STORED CALIBRATION FOUND` - the sticks are running on built-in
   defaults. The `rc_*` columns are the entire "it is remotely controllable"
   claim, and uncalibrated endpoints scale them wrongly.

   Note `YAW_MODE 0`: yaw is on **buttons**, not the stick, at a fixed
   45 deg/s. The choreography's "slow continuous yaw" is therefore *hold the
   button*, and the rendered stick panel will show discrete steps on the left
   gimbal's horizontal axis rather than smooth motion. That is correct.

4. Find the transmitter's port. With only the transmitter plugged in:
   ```
   python capture.py
   ```
   It lists what is present. On this machine the CP210x bridge enumerates as
   **COM3**.

5. Bench-test the downlink with props off. **`--tx` is required** - the
   transmitter runs at 921600 baud and the drone at 460800, and connecting at
   the wrong rate gives mojibake rather than an error:
   ```
   python capture.py COM3 --tx --tag downlink_test
   ```
   then type `tracklog 50` at the transmitter. You should see the `#TRK`
   preamble and then rows. Type `tracklog off`; the drone's `#TRK end` line
   reports `dl_sent` and `dl_failed`. **`dl_failed` must be 0.**

   Done twice on 2026-08-18, **both passed**:

   Done three times on 2026-08-18, **all passed**:

   | run | rows | duration | dropped | dl_failed | seq gaps | interval |
   |---|---|---|---|---|---|---|
   | 1 | 1570 | 31.4 s | 0 | 0 | none | 20.0 ms, sd 0.15 |
   | 2 | 3022 | 60.5 s | 0 | 0 | none | 20.0 ms, sd 0.15 |
   | 3 | 1205 | 24.7 s | 0 | 0 | none | 20.5 ms, sd 0.56 |

   Run 3 was the first with a clean stop. **Still to do: repeat with props
   ON** - motor EMI is the case none of these covered, and it is the one that
   could still bite.

   Verify the STOP path, not just the start. After `tracklog off` you should
   see BOTH:
   ```
   track relay stopped after N rows
   #TRK end seq=N dropped=0 dl_sent=N dl_failed=0
   ```
   The second line comes from the drone. If it is missing, logging did not
   stop - which is what happened on run 2, leaving two sessions in one file.

   **Which number actually proves delivery.** `dl_failed` counts
   `esp_now_send()` QUEUEING failures on the drone. There is no send callback,
   so `dl_failed=0` does not by itself prove the packets arrived. The proof is
   the file: a continuous sequence with no gaps, which is exactly what
   `analyze_track.py` checks and reports. Trust that line over the counters.

   `link` at the transmitter reports delivery CUMULATIVELY since boot, so
   packets sent before the drone was powered are counted as failures forever.
   Read the increments between two `link` calls, not the percentage.

6. Dry-run the whole render pipeline on the synthetic log so nothing about
   the video toolchain is discovered on flight day:
   ```
   python make_test_log.py logs/synthetic_choreography.log
   python render_flight.py logs/synthetic_choreography.log --fps 30
   ```

---

## Synchronising video to log

Do not eyeball it.

The arm event is in both streams: the log's first row with `armed=1`, and the
video frame where the props start moving. The difference is `--sync`.

```bash
python render_flight.py logs/final.log --video DJI_0042.MP4 --sync 12.4
```

`--sync T` is *the log time corresponding to the first frame of the video*. A
wrong `T` is obvious immediately — stick motion leads or lags the aircraft.

For a tighter lock, film the drone's status LED at the moment you start
logging; a single video frame then pins the log's `t=0` exactly.

---

## The rangefinder check - do not skip this

**The single failure that has actually stopped this aircraft flying.**

On 2026-08-18 four consecutive runs sat on the ground with the vertical loop
commanding 3.09 N against a 4.45 N weight - 69% of what is needed to hover.
The cause was the rangefinder returning no valid data: with no altitude
measurement the ESKF's vertical velocity integrated to **+3 m/s on a
stationary aircraft**, and the vertical loop correctly cut thrust to stop a
climb that was not happening. Roll, pitch and yaw were unaffected, which is
exactly why the symptom read as "throttle is broken".

It is NOT caused by the downlink and NOT caused by motor EMI. Both were
tested directly: with `tracklog net` running and all four rotors at hover RPM
(~1000 rad/s), `RNG=1` on every one of 145 status lines.

What the four failures shared is that the drone had been powered on battery
for an unknown time with no reset, and `range_ok` was **already 0 on the
first logged row, before arming**. The sensor dies at some point in a long
session and never recovers - `rangeUsable()` has no recovery path once the
reading goes stale.

**So: power-cycle the drone immediately before the flight, and verify.**

Over the drone's USB, or in the first rows of any track log:

```
status
```
```
range=0.036 m age=2 ms usable=1
```

`usable=1` and an age in milliseconds. If `usable=0`, power-cycle and check
again - do not fly it, because the aircraft physically cannot leave the
ground in that state and no gain change will alter it.

In flight, the field to watch is `RNG=` in the `ST=` line, or `range_ok` in
the track log.

---

## Flight-day sequence

**All of this happens at the transmitter. The drone's USB is never connected.**

1. `python capture.py COM3 --tx --tag final`  (`--tx` sets 921600 baud)
2. Power the drone. Wait for the link (`link` at the transmitter).
3. Start the camera. Get the drone in frame.
4. Type `tracklog 50` at the transmitter. Preamble and rows should appear.
5. Arm and fly the choreography.
6. Land, disarm.
7. Type `tracklog off`. Read the `#TRK end` line: **`dropped=0` and
   `dl_failed=0`**, or the log is not fit for a time-domain figure.
8. If either counter is non-zero, re-fly at `tracklog 25`. The loops still
   resolve at 25 Hz — the attitude loop runs at 1.6 Hz and the rate loops at
   5.6 Hz.

### The choreography

Fly it as **pilot-flown RC manoeuvres, not autonomous waypoints.** Per
`SESSION_HANDOFF.md`, Phase 5 (rate loops) is still marked NEXT, `kM` is still
the seed value so yaw authority may be over-commanded ~2×, and XY hold is
flow-based and blind below ~0.3 m. Programmed waypoints would be flying on
uncommissioned loops.

That is also the better video. The subject is *remote controllability*, so
hand-flown sticks beside the aircraft's response is the stronger artifact —
and it is exactly what the ground-track panel was built to show, circle fit
and stick trace side by side.

One continuous log:

| Phase | Duration | Purpose |
|---|---|---|
| Hover, hands off | ~10 s | holds capture — `A=HELD`, `H=HELD`, `YP=1` |
| Large circle | ~30 s | the ground-track panel; the circle fit and its rms |
| Climb/descend while yawing slowly | ~25 s | altitude and heading together |
| Land | ~8 s | closes the log cleanly |

Keep the yaw rate low. Yaw is the weakest axis by 8× and `kM` is
unidentified; a fast yaw is the one input most likely to reproduce the first
flight's saturation collapse. `SAT=` in telemetry and the `SATURATED` banner
in the rendered panel will both call it out if it happens.

---

## Rendering

```bash
# side by side, footage on the left
python render_flight.py logs/final_<stamp>.log --video DJI_0042.MP4 --sync 12.4

# transparent PNG sequence for an editor
python render_flight.py logs/final_<stamp>.log --overlay

# just the airborne part, at 24 fps
python render_flight.py logs/final_<stamp>.log --airborne --fps 24
```

Useful flags: `--span T0 T1`, `--size 1920x1080`, `--theme light`,
`--side right`, `--trail` (seconds of ground track behind the aircraft).

Measured: a 70 s log at 30 fps is 2099 frames in **195 s**. `--airborne` or
`--span` cuts that further.

Output is written with OpenCV, not ffmpeg — ffmpeg is not on this machine's
PATH and is not required.

Still figures for the report come from the existing script, unchanged:

```bash
python analyze_track.py logs/final_<stamp>.log --airborne
```

---

## Faults found and fixed on 2026-08-18

Recorded because each one was silent - none announced itself, and two of them
would have cost the flight rather than the bench session.

| Fault | Symptom | Fix |
|---|---|---|
| `capture.py` used `for line in sys.stdin` | typed commands never reached the board, intermittently. Matches the `kmlearn on never issued` entry for Phase 8 run 2 | explicit `readline()` loop; the `>>>` echo now also goes into the log, so "was it issued" is checkable |
| 50 Hz output flooded the console | could not finish typing a word; on the day, could not have typed `tracklog off` to close the log | high-rate streams hidden from screen by default, counted instead; all still in the file |
| status line used `
` + spaces | erased the terminal's echo of a half-typed command while the input buffer kept it, so retyping submitted both copies as one line | whole appended lines only, every 5 s |
| transmitter `atol()` on the tracklog argument | the doubled `tracklog offtracklog off` parsed as rate 0, clamped to 1, and **restarted** logging instead of stopping | `strtol` with end-pointer validation; unrecognised arguments are refused, never rounded |
| `analyze_track.py` summed sequence gaps across a reset | a two-session capture reported `-3024 samples missing` | sessions detected and listed; continuity checked within each |
| `analyze_track.py` validated before `--span` | could not use `--span` to answer the complaint validation had just raised | selection now happens first |
| `analyze_track.py` labelled `dt_us` "control step (budget 2500)" | it is the control PERIOD, gated at 2500, so every log looked pinned at 100% of budget | reported as jitter against target; warns only on a genuinely missed cycle |
| `headingKp` comment said flashing restores 3.00 | the compiled default is 0.00; the comment described an older build | comment corrected to match the code |

---

## Known limits going in

These are from `SESSION_HANDOFF.md` and are unchanged by this work:

- **`kM` unidentified.** Phase 8 has failed three times. Yaw authority is
  possibly over-commanded ~2×.
- **QMC5883P shifts several degrees with motor current.** Fly with
  `headingkp 0` until it is relocated. The estimator panel will show the ESKF
  heading and the magnetometer diverging — that is the known bug on display,
  not a rendering fault.
- **Optical flow is blind below ~0.3 m**, so the ground track degrades if the
  circle is flown low. `HOVER_SETPOINT_M` is 0.50 m.
- **Rate loops are untuned** (Phase 5 not done). Roll/pitch gains were raised
  to `attitudekp 10 / rollratekp 35 / pitchratekp 35` after the second flight
  but never tuned against a step.
