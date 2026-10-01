// Independent numeric check of the conventions and allocator used by
// Receiver_RotorRate. Reimplements the math from scratch in JS so a shared
// typo in the C++ cannot pass both.

let failures = 0;
function check(name, condition, detail = "") {
  if (condition) { console.log(`  PASS  ${name}`); }
  else { console.log(`  FAIL  ${name}  ${detail}`); failures++; }
}
function close(a, b, tol = 1e-6) { return Math.abs(a - b) <= tol; }

// ---------------- constants mirrored from FrameConventions.h -------------
const PI = Math.PI;
const G = 9.80665;
const MASS = 0.45359237;
const R = 0.0635;
const RHO = 1.225;
const CT = 0.18;
const KF = (4 / (PI * PI)) * CT * RHO * Math.pow(R, 4);
const SEP_FB = 0.1645, SEP_SS = 0.1630;
const ARM_X = SEP_SS / 2, ARM_Y = SEP_FB / 2;
const SPIN = [+1, -1, +1, -1];              // M1 CCW, M2 CW, M3 CCW, M4 CW
const POS_X = [-ARM_X, +ARM_X, +ARM_X, -ARM_X];
const POS_Y = [+ARM_Y, +ARM_Y, -ARM_Y, -ARM_Y];
const KM = KF * 0.010;

console.log(`kF = ${KF.toExponential(4)}  hover omega = ` +
  `${Math.sqrt(MASS * G / (4 * KF)).toFixed(1)} rad/s`);

// -------- 1. forward model derived from first principles (r x F) ---------
// Independent of the closed form in RotorAllocator: builds the wrench by
// summing cross products, exactly as the physics defines it.
function forwardFromCrossProducts(omega) {
  let T = 0, tx = 0, ty = 0, tz = 0;
  for (let i = 0; i < 4; i++) {
    const u = omega[i] * omega[i];
    const thrust = KF * u;                    // along body +Z
    T += thrust;
    // r x F with r=(x,y,0), F=(0,0,thrust)  ->  (y*thrust, -x*thrust, 0)
    tx += POS_Y[i] * thrust;
    ty += -POS_X[i] * thrust;
    // reaction to the drag the motor works against
    tz += -SPIN[i] * KM * u;
  }
  return { T, tx, ty, tz };
}

// -------- 2. the allocator's closed-form inversion ------------------------
function allocate(T, tx, ty, tz) {
  const cT = T / (4 * KF);
  const dX = tx / (4 * KF * ARM_Y);
  const dY = ty / (4 * KF * ARM_X);
  const dZ = tz / (4 * KM);
  const u = [
    cT + dX + dY - dZ,
    cT + dX - dY + dZ,
    cT - dX - dY - dZ,
    cT - dX + dY + dZ
  ];
  return u.map(v => Math.sqrt(Math.max(v, 0)));
}

console.log("\n[1] Allocator inverts the first-principles forward model");
{
  let worst = 0;
  for (let trial = 0; trial < 2000; trial++) {
    const T = MASS * G * (0.7 + 0.6 * Math.random());
    const tx = (Math.random() - 0.5) * 0.06;
    const ty = (Math.random() - 0.5) * 0.06;
    const tz = (Math.random() - 0.5) * 0.010;
    const omega = allocate(T, tx, ty, tz);
    if (omega.some(o => !isFinite(o) || o <= 0)) continue;
    const back = forwardFromCrossProducts(omega);
    worst = Math.max(worst,
      Math.abs(back.T - T), Math.abs(back.tx - tx),
      Math.abs(back.ty - ty), Math.abs(back.tz - tz));
  }
  check("round-trip error negligible over 2000 random wrenches",
    worst < 1e-9, `worst=${worst.toExponential(3)}`);
}

console.log("\n[2] Torque signs match the physical layout");
{
  const hover = Math.sqrt(MASS * G / (4 * KF));
  const base = MASS * G;

  // Nose up (+tauX about +X) must raise the FRONT rotors M1,M2.
  let o = allocate(base, 0.02, 0, 0);
  check("+tauX (nose up) raises M1,M2 and lowers M3,M4",
    o[0] > hover && o[1] > hover && o[2] < hover && o[3] < hover,
    o.map(v => v.toFixed(1)).join(","));

  // Roll right (+tauY about +Y) must raise the LEFT rotors M1,M4.
  o = allocate(base, 0, 0.02, 0);
  check("+tauY (roll right) raises M1,M4 and lowers M2,M3",
    o[0] > hover && o[3] > hover && o[1] < hover && o[2] < hover,
    o.map(v => v.toFixed(1)).join(","));

  // Yaw CCW (+tauZ about +Z) must raise the CW rotors M2,M4, because a
  // CW-spinning rotor imposes a CCW reaction torque on the airframe.
  o = allocate(base, 0, 0, 0.004);
  check("+tauZ (yaw CCW) raises the CW rotors M2,M4",
    o[1] > hover && o[3] > hover && o[0] < hover && o[2] < hover,
    o.map(v => v.toFixed(1)).join(","));
}

console.log("\n[3] The OLD mixer had the yaw column inverted");
{
  // Old code: m1,m3 += yawMix ; m2,m4 -= yawMix, with the rate PID treating
  // positive error as counter-clockwise. Raising M1,M3 (both CCW rotors)
  // produces a CLOCKWISE reaction, i.e. NEGATIVE tauZ.
  const hover = Math.sqrt(MASS * G / (4 * KF));
  const yawMixPositive = [hover + 20, hover - 20, hover + 20, hover - 20];
  const w = forwardFromCrossProducts(yawMixPositive);
  check("old mixer's positive yawMix yields NEGATIVE tauZ (positive feedback)",
    w.tz < 0, `tauZ=${w.tz.toExponential(3)}`);
}

console.log("\n[4] Body <-> navigation rotation");
{
  const bodyToNav = (yaw, bx, by) => [
    Math.cos(yaw) * bx - Math.sin(yaw) * by,
    Math.sin(yaw) * bx + Math.cos(yaw) * by
  ];
  const navToBody = (yaw, nx, ny) => [
    Math.cos(yaw) * nx + Math.sin(yaw) * ny,
    -Math.sin(yaw) * nx + Math.cos(yaw) * ny
  ];

  let worst = 0;
  for (let t = 0; t < 1000; t++) {
    const yaw = (Math.random() - 0.5) * 2 * PI;
    const bx = Math.random() * 4 - 2, by = Math.random() * 4 - 2;
    const [nx, ny] = bodyToNav(yaw, bx, by);
    const [rx, ry] = navToBody(yaw, nx, ny);
    worst = Math.max(worst, Math.abs(rx - bx), Math.abs(ry - by));
  }
  check("navToBody is the exact inverse of bodyToNav", worst < 1e-12,
    `worst=${worst.toExponential(3)}`);

  // At +90 deg (CCW), body +X (right) must point along nav +Y.
  const [nx, ny] = bodyToNav(PI / 2, 1, 0);
  check("at yaw=+90deg CCW, body +X maps to nav +Y",
    close(nx, 0, 1e-9) && close(ny, 1, 1e-9),
    `(${nx.toFixed(6)}, ${ny.toFixed(6)})`);

  // The OLD form, fed a right-handed angle, is the transpose.
  const oldBodyToNav = (yaw, bx, by) => [
    Math.cos(yaw) * bx + Math.sin(yaw) * by,
    -Math.sin(yaw) * bx + Math.cos(yaw) * by
  ];
  const [ox, oy] = oldBodyToNav(PI / 2, 1, 0);
  check("old form sends body +X to nav -Y (a 180 deg error at 90 deg heading)",
    close(ox, 0, 1e-9) && close(oy, -1, 1e-9),
    `(${ox.toFixed(6)}, ${oy.toFixed(6)})`);
}

console.log("\n[5] Magnetic heading sign");
{
  // Drone heading psi counter-clockwise from magnetic north. North expressed
  // in body coordinates is obtained by rotating the nav-frame north vector
  // (0,1) by -psi.
  const northInBody = (psi) => [
    Math.cos(-psi) * 0 - Math.sin(-psi) * 1,
    Math.sin(-psi) * 0 + Math.cos(-psi) * 1
  ];

  let worstNew = 0, worstOld = 0;
  for (let t = 0; t < 720; t++) {
    const psi = (t - 360) * PI / 360;
    const [right, forward] = northInBody(psi);
    const newYaw = Math.atan2(right, forward);   // corrected
    const oldYaw = -Math.atan2(right, forward);  // previous build
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    worstNew = Math.max(worstNew, Math.abs(wrap(newYaw - psi)));
    worstOld = Math.max(worstOld, Math.abs(wrap(oldYaw - psi)));
  }
  check("corrected +atan2(right,forward) recovers the true RH heading",
    worstNew < 1e-9, `worst=${worstNew.toExponential(3)}`);
  check("old -atan2(...) is wrong by up to 180 deg",
    worstOld > 3.0, `worst=${(worstOld * 180 / PI).toFixed(1)} deg`);
}

console.log("\n[6] IMU mounting transform (180 deg flip about body +Y)");
{
  const CHIP_TO_BODY = [-1, +1, -1];
  const LSB = 16384;

  // Level and at rest: chip Z points along body -Z, so the chip reads -1 g.
  const chip = [0, 0, -LSB];
  const body = chip.map((v, i) => CHIP_TO_BODY[i] * (v / LSB) * G);
  check("level at rest gives body specific force (0,0,+g)",
    close(body[0], 0, 1e-9) && close(body[1], 0, 1e-9) &&
    close(body[2], G, 1e-6), `(${body.map(v => v.toFixed(4)).join(", ")})`);

  // Tilt right by phi: body specific force is (-g sin phi, 0, g cos phi),
  // and the recovered right-handed roll must equal phi.
  let worst = 0;
  for (let d = -30; d <= 30; d += 1) {
    const phi = d * PI / 180;
    const fx = -G * Math.sin(phi), fz = G * Math.cos(phi);
    worst = Math.max(worst, Math.abs(Math.atan2(-fx, fz) - phi));
  }
  check("atan2(-fx, fz) recovers right-handed roll", worst < 1e-9,
    `worst=${worst.toExponential(3)}`);

  // Pitch nose up by theta: (0, g sin theta, g cos theta).
  worst = 0;
  for (let d = -30; d <= 30; d += 1) {
    const th = d * PI / 180;
    const fy = G * Math.sin(th), fz = G * Math.cos(th);
    worst = Math.max(worst, Math.abs(Math.atan2(fy, fz) - th));
  }
  check("atan2(fy, fz) recovers right-handed pitch", worst < 1e-9,
    `worst=${worst.toExponential(3)}`);
}

console.log("\n[7] Tilt required for horizontal acceleration");
{
  // Roll phi about +Y points the thrust axis at (sin phi, 0, cos phi):
  // positive roll accelerates toward +X (right).
  const phi = 5 * PI / 180;
  check("positive roll accelerates toward +X (right)",
    Math.sin(phi) > 0);

  // Pitch theta about +X points it at (0, -sin theta, cos theta):
  // accelerating toward +Y (forward) needs NEGATIVE pitch (nose down).
  const accelY = 1.0;
  const theta = Math.atan2(-accelY, G);
  check("forward acceleration commands nose-down (negative) pitch",
    theta < 0, `theta=${(theta * 180 / PI).toFixed(2)} deg`);
}

console.log("\n[8] Optical-flow lever-arm correction sign");
{
  // Sensor at r=(rx,ry,0); v_sensor = v_body + omega x r with omega=(0,0,wz)
  // gives omega x r = (-wz*ry, +wz*rx, 0), so v_body = v_sensor + (wz*ry, -wz*rx).
  const rx = 0.00, ry = 0.04, wz = 1.0;  // 1 rad/s CCW
  const vBody = [0, 0];
  const vSensor = [vBody[0] - wz * ry, vBody[1] + wz * rx];
  const recovered = [vSensor[0] + wz * ry, vSensor[1] - wz * rx];
  check("corrected lever-arm term recovers zero body velocity",
    close(recovered[0], 0, 1e-12) && close(recovered[1], 0, 1e-12),
    `(${recovered[0].toExponential(2)}, ${recovered[1].toExponential(2)})`);

  // The old build subtracted where it should have added.
  const oldRecovered = [vSensor[0] - wz * ry, vSensor[1] + wz * rx];
  check("old lever-arm term doubles the error instead of removing it",
    Math.abs(oldRecovered[0]) > Math.abs(vSensor[0]) - 1e-12 &&
    Math.abs(oldRecovered[0]) > 0.07,
    `vx=${oldRecovered[0].toFixed(4)} m/s from 1 rad/s yaw`);
}

console.log(`\n${failures === 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED"}`);
process.exit(failures === 0 ? 0 : 1);
