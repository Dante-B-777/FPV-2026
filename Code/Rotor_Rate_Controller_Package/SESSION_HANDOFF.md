# FPV 2026 — Rotor Rate Controller: Session Handoff

Last updated: 2026-08-15

## Standing constraints (do not violate)
- "pinouts and physical axises should be stated in the old code. **trust the polarity signs.**"
- "**do not write new code frot he pitch roll or yaw directions just varify for thay oen**" — pitch/roll/yaw direction work is verification only.
- IMU mounted upside down (`R_body_from_chip = diag(-1,+1,-1)`), rangefinder points straight down, magnetometer mounted backwards.
- Frame: +X right, +Y forward, +Z up. Yaw CCW positive. **Roll rotates about +Y (`bodyRateYRh`); pitch rotates about +X (`bodyRateXRh`).**

## Logs — do not paste them into chat
Pasting a flight log burns the whole context window. Capture to a file instead and hand over the path:

```bash
python capture.py COM5 --tag phase8
```

Writes `logs/phase8_<timestamp>.log`, tees to console, and still accepts typed serial commands. A log file can be read and grepped selectively; a paste cannot.

## Build
- `arduino-cli` at `/c/Users/dmben/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe`
- Libraries: `/c/Users/dmben/OneDrive/Documents/Arduino/libraries`, FQBN `esp32:esp32:esp32`
- Receiver last clean compile: 1013381 B (77% flash), 51552 B (15% RAM)
- Transmitter: 916360 B (69% flash), 45664 B (13% RAM)

## Fixed and verified
| Bug | Status |
|---|---|
| Arm cycling — unsigned underflow in failsafe (`nowMs - lastPacketMs`, ISR race). Fixed with signed `int32_t packetAgeMs`. | Verified: zero `arm released` lines, worst gap 26 ms, 0 failsafe gaps. User: "on and off ramps subsided." |
| Disarm-reason chain mis-ordered (`motorsEnabled` tested before `signalLost`) | Fixed |
| Throttle dead in Climb — fixed `CLIMB_RATE_MPS` ignored the stick, `T` pinned at 5.00 N. Auto-climb now yields to `sticks.throttleActive`. | Verified: "throttle was smooth" |
| Takeoff thrust dropout (ground idle bypassed `RotorSpeedLoop`) | Fixed via `GROUND_IDLE_OMEGA_RAD_PER_SEC = 500` |
| Altitude hold capturing on the ground | Fixed via `physicallyAirborne()` (≥0.15 m for 300 ms) |
| Rate loops invisible in telemetry | Fixed: new `ratelog` command |

## Phase 4 axis verification (passed, from log)
`RC = throttle, roll, pitch, yaw`
- Pitch +1.00 → `τx = −0.0427`, OMG 874,880,965,982; −1.00 reverses. Antisymmetric ✓
- Roll +1.00 → `τy = +0.0465`, OMG 960,860,891,986; −1.00 reverses ✓
- Yaw CCW +0.47 → `τz = +0.0269`, motors **2 & 4 up**, {1,3} down; CW reverses ✓ — **original yaw sign bug is dead**
- Derived layout: M1 back-right, M2 back-left, M3 front-left, M4 front-right (X quad)

## 2026-08-15 first props-ON flight — yaw saturation collapse
First powered flight. Ended in `SAFETY STOP: excessive tilt` at 39.9° roll.

**Airframe authority (computed, not measured):**
| | value |
|---|---|
| kF | 1.4531e-6 |
| kM (seed) | 1.4531e-8 |
| Izz | 2.027e-3 kg·m² |
| hover u / ω | 7.656e5 / 875 rad/s |
| roll·pitch authority | **0.336 N·m** |
| yaw authority @ hover | **0.041 N·m** |
| peak yaw demand in log | **0.162 N·m → 4× over** |

Yaw is the weakest axis by 8×, and it was the one being over-driven.

**Failure chain:**
1. Heading hold captured at 5.5°; heading estimate then walked away faster than the gyro justified (compass/motor-current coupling, still unresolved).
2. Yaw rate PID (`yawratekp=15`, Izz=2.027e-3) demanded up to 0.162 N·m — 4× achievable. Saturates at only 77 °/s of rate error.
3. `RATE_INTEGRAL_LIMIT=60 rad/s²` vs ~20 rad/s² achievable → integral term alone could demand 3× total yaw authority, and latched against the saturated actuator.
4. Old allocator policy slid the **collective** to make room for torque and scaled **all three** torque axes by one common factor → thrust dumped/surged 4.26–5.90 N with throttle at zero, altitude wandered 0.31–0.68 m against a 0.41 m hold, and roll/pitch lost authority simultaneously.
5. `DS=750,250,750,113` held for many consecutive cycles — two rotors on the ceiling, two on the floor. Yaw limit cycle ±194 °/s. Tilt trip.

Explains all three pilot reports: "thrust drop significantly" on yaw, "altitude hold disabled", "very unstable" — one bug, three symptoms.

**Fixes applied (compiled 1013977 B / 77%, 51552 B / 15%):**
- `RotorAllocator`: priority is now **roll/pitch → thrust → yaw**. Collective slides for roll/pitch only; yaw is fitted into leftover rotor range and scaled down (to zero if needed). Roll/pitch still share one scale factor to preserve tilt direction.
- `updateYawController`: clamps τz to `achievableYawTorqueNm(thrustN, kM)` and **backs the integrator out** by exactly the discarded amount (conditional integration).
- kM identifier now also gated on `yawScaled` / `yawTorqueSaturated` — learning from a clipped cycle biases kM low, which is the original over-command error.
- New telemetry field `SAT=abc` — a: yaw demand clipped, b: yaw allocation clipped, c: roll/pitch scaled.

**Not yet done:** compass coupling still unresolved; kM still the 0.0100 seed.

## 2026-08-15 second flight — fixes verified, gains locked
`SAT=000` on essentially every line (one lone `SAT=010`). No tilt trip. Altitude hold, XY hold and heading hold all captured and stayed captured. Flown with `headingkp 0`.

Pilot: "yaw control was very good but roll and pitch were weaker than I would want."

Diagnosis: roll/pitch were **under-driven, not short on authority**. Peak torque either axis requested all flight was 0.113 N·m against 0.336 N·m available (34%). A 22.9° roll excursion drew only 0.09 N·m and took ~1 s to recover.

**Gains locked into the source (compiled 1013977 B / 77%, 51552 B / 15%):**
| gain | was | now |
|---|---|---|
| `attitudekp` | 7.00 | **10.00** |
| `rollratekp` | 25.0 | **35.0** |
| `pitchratekp` | 25.0 | **35.0** |

Cascade ratio preserved at 35/10 = 3.5 (rate loop must stay ~3.5× faster than the attitude loop).

⚠️ **`headingkp` still compiles as 3.00.** Flashing re-arms the compass-driven yaw path that caused the first flight's collapse. Run `headingkp 0` after every flash until the magnetometer is relocated and Phase 9 passes.

## 2026-08-15 Phase 8 attempt — FAILED, and the fixes
Trim worked (`RP=+0.6` at rest, X stayed +0.06 to +0.13, no runaway drift). But Phase 8 did not pass.

**Reported result:** N=8813, R=0.0025, kM=3.602e-09, `converged=true`.
**Real result:** garbage. `converged()` was `updateCount_ >= 2000` — a sample counter. R swung 0.0020 ↔ 0.0197 (10×) with no settling, pinned on the 0.0020 clamp floor for long stretches. R=0.0025 m implies `C_Q/C_T = 0.039`; physical band is **0.010–0.016 m**. No such propeller.

**Three causes, all fixed (compiled 1014681 B / 77%, 51584 B / 15%):**

1. **Never airborne.** No `Airborne confirmed` line anywhere; `A=FREE X=FREE` all flight; ALT 0.09–0.14 m vs 0.25 target. Identifier gated on `flightActive` (true at spool-up) → now gates on `physicallyAirborne`.

2. **Takeoff deadlock** (interaction between the auto-climb fix and the airborne gate):
   - Throttle blip → `throttleActive` → auto-climb cancelled **permanently**
   - Release → `else` branch commands `throttle × MAX_VERTICAL_VELOCITY` = **0** → holds zero vertical *velocity* at 0.10 m
   - Under the 0.15 m `physicallyAirborne()` gate → hold never captures → can never climb
   - `T=4.5 N` vs 4.45 N hover: the loop was doing exactly what it was told
   - **Fix:** auto-climb yields to stick only once `physicallyAirborne()`; below that, up-stick *adds* to climb rate. Down-stick aborts unconditionally at any height.
   - **NOT fixed by adding thrust at zero throttle** — that deletes the neutral point that makes stick release safe and turns zero throttle into an open-loop climb.

3. **No aerodynamic yaw damping in the model.** In a sustained turn `αz≈0` and `dHz/dt≈0`, so `observation≈0` with `D≠0` — the only fit was kM→0. Model is now `yHat = −kM·D − c·ωz` with `c` jointly estimated, clamped ≥0, bounded at 0.05. Regressors normalised (D~1e5 vs ωz~1) so damping isn't swamped.
   - Log-implied plausible `c` ≈ **0.009 N·m per rad/s**. If `C` settles near that, it works; if it pins at 0.05, something's wrong.

**`converged()`** now requires kM within 5% of its own slow mean AND off both clamp rails, on top of the count.

**New telemetry:** `C=` damping, `SPR=` kM spread fraction, `CV=` real converged flag. **Watch `SPR`, not `N`.**

## 2026-08-15 — airframe mass corrected (the four-session altitude blocker)
`AIRFRAME_MASS_KG` **0.45359237 → 0.4929 kg** (1.000 → 1.087 lb). Compiled 1014677 B / 77%, 51584 B / 15%.

Identified from 133 steady-hover samples (|VZ|≤0.02) in `logs/phase8_20260815_005942.log`: delivered `T = kF·Σω² = 4.835 N` → 0.4929 kg. Widening to |VZ|≤0.05 (n=339) gives 4.866 N / 0.4960 kg — stable.

**Key cross-check:** commanded thrust 4.826 N vs delivered 4.835 N — 0.2% agreement. Allocator and rotor loop are faithful; `kF` was never wrong.

Why it mattered: no integral term anywhere in the vertical cascade, so mass error → permanent altitude sag. Loop commanded 4.83 N, aircraft weighed 4.83 N, it sat at 0.15 m against a 0.25 m target. Every Phase 8 run was flown in ground effect because of this.

⚠️ Measured *in* ground effect, so 0.4929 is a **lower bound** — if sag persists at real hover height, raise it again.
⚠️ Inertias scale with mass → all three rate loops now command ~8.7% more torque for the same error. Roll/pitch gains were tuned at the old mass.

## Phase 8 status after three attempts (all invalid)
| run | N | R final | C final | SPR | why invalid |
|---|---|---|---|---|---|
| 1 | 8813 | 0.0025 | — | — | `converged()` was a sample counter; never airborne |
| 2 | 0 | — | — | — | `kmlearn on` never issued |
| 3 | 22778 | 0.0020 (floor) | 0.0126 rising | 0.387 | flown in ground effect at 0.15 m |

Run 3 had `headingkp = 0.0000` correctly set — **removing the compass did NOT stop kM collapsing to the floor.** That kills the compass hypothesis.

**New finding: kM and C are collinear.** C climbed 0.0062 → 0.0126 (past the ~0.009 plausible value, heading for the 0.05 bound) while R fell to the floor. The regression can explain the same observation as rotor drag *or* airframe damping, and with slow yaw in ground effect `D` and `ωz` move in lockstep so it can't separate them. Do not tune the identifier further — it now correctly refuses to claim convergence. Fix the flight condition instead.

## 2026-08-15 — altitude hold was truncating every climb
**This is why every flight was stuck at 0.14–0.20 m.** Log order gives it away:

```
Airborne confirmed at 0.18 m
Altitude hold captured at 0.250 m
Climb complete at 0.202 m; hovering.
```

`physicallyAirborne()` latches at 0.18 m → permits the hold → hold captures at ~0.20 m → `MIN_ALTITUDE_TARGET_M` clamps the target **up** to 0.25 → the Climb exit test `altitudeM >= target − 0.05` is now satisfied at 0.202 → climb declares complete. The aircraft never went near `HOVER_SETPOINT_M`.

**Fix:** `holdPermitted` now also requires `flightState != FlightState::Climb`. Climb owns the axis until it finishes.
**Also:** `HOVER_SETPOINT_M` 0.60 → **0.50 m** (~4 rotor diameters, clear of ground effect).

Compiled 1014689 B / 77%, 51584 B / 15%.

⚠️ This does NOT explain the residual sag (sitting at 0.14 with a 0.11 m error and excess thrust commanded, not climbing). Next flight is the decisive test:
- **Climbs to ~0.5 m and holds** → truncation was the whole story, and the 0.4929 kg mass change may have been unnecessary. Check whether it settles above or below 0.5 to get the sign of any remaining mass error.
- **Still sits at ~0.14 m** → the aircraft is mechanically constrained (tether/stand/safety line), and no parameter change will fix it.

⚠️ `AIRFRAME_MASS_KG = 0.4929` remains **unvalidated**. It was derived assuming free-flight equilibrium, which the 13:04 flight falsified (7.4% more commanded thrust produced zero altitude change). Revert to 0.45359237 if the next flight shows the aircraft was never thrust-limited.

## 2026-08-15 full-battery test — thrust model FIXED, rotor loop deadlock found
Compiled 1014717 B / 77%, 51584 B / 15%.

**`PROP_THRUST_CT` 0.18 → 0.153.** Was a textbook guess, 17% optimistic.
- Identified from 213 near-zero-VZ samples: modelled T at hover 5.224 N vs actual weight 4.450 N → kF ratio 0.852 → CT 0.153.
- **Validated on full battery:** hovers at 935 rad/s. CT=0.153 predicts 949; CT=0.18 predicts 875. Modelled T at hover now 4.400 N vs 4.450 N weight — **1.1% error**.
- Independent check: `hoverOmegaRadPerSec()` went from predicting 875 to 949 without being fitted to it.

**`AIRFRAME_MASS_KG` reverted to 0.45359237** (pilot-measured 1 lb). The 0.4929 was wrong — see the long note in FrameConventions.h. Two lessons recorded there: the "commanded ≈ delivered thrust" check was **circular** (both use the model kF, so they agree whenever the rotor loop tracks, regardless of kF); and the implied mass *followed* each edit (0.480→0.526→0.533 kg), which is the signature of a scale error elsewhere, not a real mass.

**Battery: pilot was right, and so was CT.** Dead pack showed RPM *collapse* above DShot 650 (700→1015, 750→904 rad/s) — voltage sag. Full pack is monotonic, ~8% more RPM per DShot. But thrust depends on RPM, not on how RPM was reached, so CT stands.

**ROTOR LOOP DEADLOCK (fixed).** `INTEGRAL_ENABLE_BAND` was a flat 60 rad/s. Steady-state error was 81 rad/s → integrator frozen → only `kp·error` = 0.15×81 ≈ 12 DShot counts against ~90 needed → error stays 81 → integrator stays frozen. Aircraft sat in Climb at DShot ~420 with the 750 cap unused.
**Fix:** band is now `max(60, 0.10 × setpoint)`. New `INTEGRAL_ENABLE_FRACTION = 0.10`. Spool-up errors (hundreds of rad/s) still excluded, so the bench-overshoot guard is intact.

⚠️ `DSHOT_FLIGHT_MAX = 750` — worth knowing it exists; the dead-battery flight hit it exactly.

## Commissioning status
1–3 self-test / rotor loop / timing — PASS
4 torque direction — PASS
5 rate loops — **NEXT**
6 attitude — pending
7 altitude — partially seen working
8 kM identification — pending (`kmlearn on`)
9 heading hold — captures, but compass coupling unresolved
10 XY hold — `YP=1` proves; flow blind below ~0.3 m

## Open risks
- **kM unidentified** (`KM=1.453e-08`, `R=0.0100` seed). 0.47 yaw stick gave a ~2:1 rotor spread → yaw authority likely over-commanded ~2×. Run Phase 8 before real yaw flying.
- **QMC5883P shifts several degrees with motor current.** Relocate away from power leads before Phase 9.
- Unexplained: "steadied when I lifted it off the ground." No account given; moot if cycling stays gone.
- Residual first-`ST=2`-line DShot dip after each arm ramp (DS=208/223 vs steady 183–185).

## Phase 5 procedure (rate loops)
**Reflash the receiver first — `ratelog` is new firmware.**

Serial: `ratelog on` → streams
```
RATE r=  +12.3/  +15.0 p=   -3.1/   -2.0 y=   +0.4/   +0.0 dps | TAU=+0.0031,+0.0057,-0.0006
```
measured/target deg/s per axis.

**Props ON required.** Rate loops cannot be excited with props off — there is no aerodynamic torque, so measured will never track target. Constrain the airframe to one axis at a time (single-axis pivot; for roll, hold by two opposite arms).

Gains (`set <name> <value>`, review with `gains`):
| Axis | Kp | Ki | Kd |
|---|---|---|---|
| roll | `rollratekp` 25 | `rollrateki` 8 | `rollratekd` 0.40 |
| pitch | `pitchratekp` 25 | `pitchrateki` 8 | `pitchratekd` 0.40 |
| yaw | `yawratekp` 15 | `yawrateki` 4 | `yawratekd` 0.20 |

Method per axis: zero Ki and Kd, raise Kp until measured tracks target with a small overshoot, back off ~20%, add Kd to damp, then restore Ki to kill steady-state offset.

## Other serial commands
`gains`, `link` (ESP-NOW packets/gaps), `kmlearn on`, `ratelog on|off`
