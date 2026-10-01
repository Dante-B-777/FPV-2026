%% motor_prop_voltage_sizing_vertical_takeoff.m
% ------------------------------------------------------------
% Vertical-only quadrotor takeoff + motor/prop/battery sizing
%
% USER INPUT PARAMETERS are written in ALL CAPS.
% Internal calculated variables use normal MATLAB style.
%
% Model:
%   Fi = kf * omega_i^2
%   Qi = kq * omega_i^2
%
%   m*z_ddot = F1 + F2 + F3 + F4 - m*g - cDrag*z_dot
%
% Brushless DC motor steady-state model:
%   V = Ke*omega + R*(I - I0)
%   Q = Kt*(I - I0)
%
%   Kt approximately equals Ke in SI units.
%
% z is positive upward.
%
% This simulation ignores:
%   - x/y motion
%   - roll, pitch, yaw
%   - ESC switching dynamics
%   - battery sag
%   - prop unloading
%   - ground effect
%   - full aerodynamic effects

clear; clc; close all;

%% ============================================================
%  USER INPUTS - ONLY THESE ARE ALL CAPS
% ============================================================

% ----------------------------
% Flight command
% ----------------------------
Z_REF_FINAL = 1.0;                  % commanded hover height [m]
TAKEOFF_RAMP_TIME = 1.0;            % command ramp time [s]

% ----------------------------
% Drone physical parameters
% ----------------------------
DRONE_MASS_KG = 0.500;              % all-up drone mass [kg]
GRAVITY = 9.81;                     % gravity [m/s^2]
N_ROTORS = 4;                       % quadrotor = 4 rotors

% Lumped vertical damping.
% Higher value means more thrust required at higher vertical speed.
C_DRAG = 2.0;                       % [N/(m/s)]

% ----------------------------
% Motor specs
% ----------------------------
MOTOR_NAME = "EMAX ECO II 2306 2400KV";

MOTOR_KV_RPM_PER_V = 2400;          % nominal motor KV [RPM/V]
MOTOR_RESISTANCE_OHM = 0.12;        % estimated; replace if datasheet gives winding resistance
MOTOR_NO_LOAD_CURRENT_A = 0.70;     % estimated starter value
MOTOR_MAX_CURRENT_A = 39.8;         % max current from ECO II 2306 2400KV listing

MOTOR_CURRENT_SAFETY_FACTOR = 0.80;

% ----------------------------
% Prop / rotor coefficients
% ----------------------------
PROP_NAME = "HQ v1s 5x4x3 estimated from ECO 2306 2400KV test data";

% Thrust coefficient:
%   F = KF * omega^2
%
% Units:
%   KF = N/(rad/s)^2
%
% Replace using real thrust table data when available.
KF = 1.20e-6;       % N/(rad/s)^2

% Torque coefficient:
%   Q = KQ * omega^2
%
% Units:
%   KQ = N*m/(rad/s)^2
%
% Replace using real RPM/current data when available.
KQ = 7.0e-9;        % N*m/(rad/s)^2

% ----------------------------
% Battery assumptions
% ----------------------------
BATTERY_CELL_VOLTAGE_NOMINAL = 3.70;    % LiPo nominal cell voltage [V]
BATTERY_CELL_VOLTAGE_FULL = 4.20;       % LiPo full cell voltage [V]
BATTERY_CELL_VOLTAGE_LOW = 3.50;        % conservative low cell voltage [V]

BATTERY_CELL_COUNTS = 3:4;              % 3S-4S

% Required battery voltage should exceed required motor voltage.
% This margin accounts for ESC loss, battery sag, and design margin.
BATTERY_VOLTAGE_MARGIN = 1.15;

% ----------------------------
% Vertical velocity sizing sweep
% ----------------------------
VERTICAL_SPEED_MIN = 0.1;           % [m/s]
VERTICAL_SPEED_MAX = 10.0;          % [m/s]
VERTICAL_SPEED_STEP = 0.1;          % [m/s]

% For steady climb, leave this at zero.
% To size for acceleration during climb, increase this.
DESIRED_ACCELERATION = 0.0;         % [m/s^2]

% ----------------------------
% PI altitude controller gains
% ----------------------------
KP_PER_ROTOR = 0.25;                % [N/m] per rotor
KI_PER_ROTOR = 0.0001;              % [N/(m*s)] per rotor

INTEGRAL_LIMIT = 8.0;

% ----------------------------
% Simulation timing
% ----------------------------
DT = 0.01;                          % simulation time step [s]
T_FINAL_MINIMUM = 15.0;             % minimum sim time [s]

% ----------------------------
% 3D animation geometry
% ----------------------------
ARM_LENGTH = 0.20;                  % visual arm length [m]
ANIMATION_SKIP = 8;                 % larger = faster animation

%% ============================================================
%  INTERNAL VARIABLES - NORMAL STYLE
% ============================================================

zRefFinal = Z_REF_FINAL;
takeoffRampTime = TAKEOFF_RAMP_TIME;

m = DRONE_MASS_KG;
g = GRAVITY;
nRotors = N_ROTORS;
cDrag = C_DRAG;

motorName = MOTOR_NAME;
motorKV = MOTOR_KV_RPM_PER_V;
motorR = MOTOR_RESISTANCE_OHM;
motorI0 = MOTOR_NO_LOAD_CURRENT_A;
motorImax = MOTOR_MAX_CURRENT_A;
motorCurrentSafetyFactor = MOTOR_CURRENT_SAFETY_FACTOR;

propName = PROP_NAME;
kf = KF;
kq = KQ;

batteryCellVoltageNominal = BATTERY_CELL_VOLTAGE_NOMINAL;
batteryCellVoltageFull = BATTERY_CELL_VOLTAGE_FULL;
batteryCellVoltageLow = BATTERY_CELL_VOLTAGE_LOW;
batteryCellCounts = BATTERY_CELL_COUNTS;
batteryVoltageMargin = BATTERY_VOLTAGE_MARGIN;

verticalSpeedMin = VERTICAL_SPEED_MIN;
verticalSpeedMax = VERTICAL_SPEED_MAX;
verticalSpeedStep = VERTICAL_SPEED_STEP;
desiredAcceleration = DESIRED_ACCELERATION;

kpPerRotor = KP_PER_ROTOR;
kiPerRotor = KI_PER_ROTOR;
integralLimit = INTEGRAL_LIMIT;

dt = DT;
tFinalMinimum = T_FINAL_MINIMUM;

armLength = ARM_LENGTH;
animationSkip = ANIMATION_SKIP;

%% ============================================================
%  DERIVED MOTOR CONSTANTS
% ============================================================

% Convert KV to SI motor constants.
% Ke [V/(rad/s)]
% Kt [N*m/A]
motorKe = 60/(2*pi*motorKV);
motorKt = motorKe;

%% ============================================================
%  HOVER CALCULATION
% ============================================================

fHoverTotal = m*g;
fHoverPerRotor = fHoverTotal/nRotors;

omegaHover = sqrt(fHoverPerRotor/kf);
rpmHover = omegaHover*60/(2*pi);

qHover = kq*omegaHover^2;
iHover = motorI0 + qHover/motorKt;
vHover = motorKe*omegaHover + motorR*(iHover - motorI0);
pHover = vHover*iHover;

fprintf('\nMotor / prop inputs:\n');
fprintf('Motor: %s\n', motorName);
fprintf('Prop:  %s\n', propName);
fprintf('KV = %.0f RPM/V\n', motorKV);
fprintf('R  = %.4f ohm\n', motorR);
fprintf('I0 = %.3f A\n', motorI0);
fprintf('Imax = %.2f A\n', motorImax);
fprintf('Ke = %.6f V/(rad/s)\n', motorKe);
fprintf('Kt = %.6f N*m/A\n', motorKt);
fprintf('kf = %.4e N/(rad/s)^2\n', kf);
fprintf('kq = %.4e N*m/(rad/s)^2\n', kq);

fprintf('\nHover estimates:\n');
fprintf('Drone mass = %.3f kg\n', m);
fprintf('Drone weight = %.3f N = %.0f gf\n', m*g, m*1000);
fprintf('Hover thrust per rotor = %.3f N = %.1f gf\n', ...
    fHoverPerRotor, fHoverPerRotor/g*1000);
fprintf('Hover RPM = %.0f RPM\n', rpmHover);
fprintf('Hover current per motor = %.2f A\n', iHover);
fprintf('Hover voltage per motor = %.2f V\n', vHover);
fprintf('Hover electrical power per motor = %.2f W\n', pHover);
fprintf('Total hover electrical power = %.2f W\n', nRotors*pHover);

%% ============================================================
%  VERTICAL CLIMB VOLTAGE SWEEP
% ============================================================

vSweep = verticalSpeedMin:verticalSpeedStep:verticalSpeedMax;
nV = length(vSweep);

fPerRotorSweep = zeros(nV,1);
omegaSweep = zeros(nV,1);
rpmSweep = zeros(nV,1);
qSweep = zeros(nV,1);
iSweep = zeros(nV,1);
vMotorSweep = zeros(nV,1);
pSweep = zeros(nV,1);
tTotalSweep = zeros(nV,1);

for j = 1:nV

    vz = vSweep(j);

    % Required total thrust for steady vertical climb:
    %   T = mg + m*a + cDrag*vz
    tTotalRequired = m*g + m*desiredAcceleration + cDrag*vz;

    fRequiredPerRotor = tTotalRequired/nRotors;

    omegaRequired = sqrt(fRequiredPerRotor/kf);
    qRequired = kq*omegaRequired^2;
    iRequired = motorI0 + qRequired/motorKt;
    vRequired = motorKe*omegaRequired + motorR*(iRequired - motorI0);
    pRequired = vRequired*iRequired;

    tTotalSweep(j) = tTotalRequired;
    fPerRotorSweep(j) = fRequiredPerRotor;
    omegaSweep(j) = omegaRequired;
    rpmSweep(j) = omegaRequired*60/(2*pi);
    qSweep(j) = qRequired;
    iSweep(j) = iRequired;
    vMotorSweep(j) = vRequired;
    pSweep(j) = pRequired;
end

vRequiredMax = max(vMotorSweep);
vBatteryRecommendedMin = batteryVoltageMargin*vRequiredMax;

fprintf('\nVoltage sweep summary for %.1f to %.1f m/s vertical climb:\n', ...
    min(vSweep), max(vSweep));
fprintf('Max required motor voltage = %.2f V\n', vRequiredMax);
fprintf('Recommended minimum battery voltage with margin = %.2f V\n', ...
    vBatteryRecommendedMin);

%% ============================================================
%  BATTERY CELL COUNT CHECK
% ============================================================

fprintf('\nBattery cell-count check:\n');
fprintf('Cells | V low | V nominal | V full | Status\n');

recommendedCells = NaN;

for cells = batteryCellCounts

    vLow = cells*batteryCellVoltageLow;
    vNominal = cells*batteryCellVoltageNominal;
    vFull = cells*batteryCellVoltageFull;

    if vNominal >= vBatteryRecommendedMin
        status = "PASS nominal";
        if isnan(recommendedCells)
            recommendedCells = cells;
        end
    elseif vFull >= vBatteryRecommendedMin
        status = "MARGINAL only when full";
    else
        status = "FAIL";
    end

    fprintf('%5d | %5.2f | %9.2f | %6.2f | %s\n', ...
        cells, vLow, vNominal, vFull, status);
end

if ~isnan(recommendedCells)
    fprintf('\nRecommended minimum LiPo: %dS\n', recommendedCells);
else
    fprintf('\nNo battery in the checked range meets the requested voltage.\n');
end

%% ============================================================
%  SELECTED SPEED TABLE
% ============================================================

reportSpeeds = [0.1 0.5 1 2 3 5 7.5 10];

fprintf('\nPer-motor requirements at selected vertical speeds:\n');
fprintf('vz [m/s] | Thrust [gf] | RPM | Current [A] | Voltage [V] | Power [W]\n');

for s = 1:length(reportSpeeds)

    [~, idx] = min(abs(vSweep - reportSpeeds(s)));

    fprintf('%8.2f | %11.1f | %7.0f | %11.2f | %11.2f | %9.2f\n', ...
        vSweep(idx), ...
        fPerRotorSweep(idx)/g*1000, ...
        rpmSweep(idx), ...
        iSweep(idx), ...
        vMotorSweep(idx), ...
        pSweep(idx));
end

iLimitSafe = motorCurrentSafetyFactor*motorImax;

if max(iSweep) > motorImax
    fprintf('\nWARNING: Required current exceeds motor max current.\n');
elseif max(iSweep) > iLimitSafe
    fprintf('\nWARNING: Required current exceeds %.0f%% of motor max current.\n', ...
        motorCurrentSafetyFactor*100);
else
    fprintf('\nCurrent check: OK for selected velocity range.\n');
end

%% ============================================================
%  VERTICAL TAKEOFF SIMULATION
% ============================================================

kp = kpPerRotor*ones(nRotors,1);
ki = kiPerRotor*ones(nRotors,1);

intError = zeros(nRotors,1);

tFinal = max(tFinalMinimum, 8 + 3*zRefFinal);
t = 0:dt:tFinal;
nt = length(t);

% State
z = 0;
zDot = 0;

% Storage
zHist = zeros(nt,1);
zRefHist = zeros(nt,1);
zDotHist = zeros(nt,1);
aHist = zeros(nt,1);

omegaHist = zeros(nt,nRotors);
rpmHist = zeros(nt,nRotors);
fHist = zeros(nt,nRotors);
tHist = zeros(nt,1);

errHist = zeros(nt,1);
vHist = zeros(nt,nRotors);
iHist = zeros(nt,nRotors);
pHist = zeros(nt,nRotors);

for k = 1:nt

    tk = t(k);

    % Smooth altitude command
    if tk < takeoffRampTime
        zRef = zRefFinal*(tk/takeoffRampTime);
    else
        zRef = zRefFinal;
    end

    err = zRef - z;

    % PI integrator for each rotor
    intError = intError + err*dt;
    intError = max(min(intError, integralLimit), -integralLimit);

    % Per-rotor thrust command
    fCmd = fHoverPerRotor + kp.*err + ki.*intError;
    fCmd = max(fCmd, 0);

    % Convert thrust command to rotor speed
    omega = sqrt(fCmd/kf);

    % Motor torque, current, voltage, power
    q = kq*(omega.^2);
    current = motorI0 + q/motorKt;
    voltage = motorKe*omega + motorR*(current - motorI0);
    powerElec = voltage.*current;

    % Current limit approximation
    overCurrent = current > motorImax;

    if any(overCurrent)

        current(overCurrent) = motorImax;

        qAvailable = motorKt*(current(overCurrent) - motorI0);

        omega(overCurrent) = sqrt(max(qAvailable,0)/kq);

        fCmd(overCurrent) = kf*(omega(overCurrent).^2);

        voltage(overCurrent) = motorKe*omega(overCurrent) + ...
            motorR*(current(overCurrent) - motorI0);

        powerElec(overCurrent) = voltage(overCurrent).*current(overCurrent);
    end

    % Actual thrust
    f = kf*(omega.^2);

    % Total thrust
    tTotal = sum(f);

    % Vertical acceleration
    zDDot = (tTotal - m*g - cDrag*zDot)/m;

    % Integrate vertical motion
    zDot = zDot + zDDot*dt;
    z = z + zDot*dt;

    % Ground constraint
    if z < 0
        z = 0;
        if zDot < 0
            zDot = 0;
        end
    end

    % Save data
    zHist(k) = z;
    zRefHist(k) = zRef;
    zDotHist(k) = zDot;
    aHist(k) = zDDot;

    omegaHist(k,:) = omega.';
    rpmHist(k,:) = omega.'*60/(2*pi);
    fHist(k,:) = f.';
    tHist(k) = tTotal;

    errHist(k) = err;
    vHist(k,:) = voltage.';
    iHist(k,:) = current.';
    pHist(k,:) = powerElec.';
end

fprintf('\nTakeoff simulation results:\n');
fprintf('Commanded hover height = %.3f m\n', zRefFinal);
fprintf('Final altitude = %.3f m\n', zHist(end));
fprintf('Maximum altitude = %.3f m\n', max(zHist));
fprintf('Maximum vertical speed = %.3f m/s\n', max(zDotHist));
fprintf('Maximum rotor RPM in takeoff sim = %.0f RPM\n', max(rpmHist(:)));
fprintf('Maximum motor voltage in takeoff sim = %.2f V\n', max(vHist(:)));
fprintf('Maximum motor current in takeoff sim = %.2f A\n', max(iHist(:)));
fprintf('Maximum total electrical power in takeoff sim = %.2f W\n', ...
    max(sum(pHist,2)));

%% ============================================================
%  AUTO-SCALING VALUES
% ============================================================

zMaxData = max([zHist; zRefHist; zRefFinal]);
zPlotMax = max(1.2, 1.25*zMaxData);

rpmPlotMax = 1.15*max(rpmHist(:));
fPlotMax = 1.15*max(fHist(:));
tPlotMax = 1.15*max(tHist);
vPlotMax = 1.15*max([vHist(:); vMotorSweep(:)]);
iPlotMax = 1.15*max([iHist(:); iSweep(:)]);

%% ============================================================
%  PLOTS - SIZING SWEEP
% ============================================================

figure;
plot(vSweep, vMotorSweep, 'LineWidth', 1.8);
grid on;
xlabel('Vertical climb speed [m/s]');
ylabel('Required voltage per motor [V]');
title('Required Motor Voltage vs Vertical Climb Speed');

figure;
plot(vSweep, iSweep, 'LineWidth', 1.8); hold on;
yline(motorImax, '--', 'Motor max current');
yline(iLimitSafe, '--', 'Recommended current limit');
grid on;
xlabel('Vertical climb speed [m/s]');
ylabel('Required current per motor [A]');
title('Required Motor Current vs Vertical Climb Speed');

figure;
plot(vSweep, rpmSweep, 'LineWidth', 1.8);
grid on;
xlabel('Vertical climb speed [m/s]');
ylabel('Required RPM per motor');
title('Required Motor RPM vs Vertical Climb Speed');

figure;
plot(vSweep, pSweep, 'LineWidth', 1.8);
grid on;
xlabel('Vertical climb speed [m/s]');
ylabel('Required electrical power per motor [W]');
title('Required Electrical Power vs Vertical Climb Speed');

%% ============================================================
%  PLOTS - TAKEOFF SIMULATION
% ============================================================

figure;
plot(t, zHist, 'LineWidth', 1.8); hold on;
plot(t, zRefHist, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time [s]');
ylabel('Altitude z [m]');
title('Altitude Response');
legend('Actual altitude', 'Commanded altitude');
ylim([0 zPlotMax]);

figure;
plot(t, zDotHist, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('Vertical Velocity [m/s]');
title('Vertical Velocity');

figure;
plot(t, aHist, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('Vertical Acceleration [m/s^2]');
title('Vertical Acceleration');

figure;
plot(t, rpmHist, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Rotor Speed [RPM]');
title('Rotor Speeds During Takeoff');
legend('Rotor 1', 'Rotor 2', 'Rotor 3', 'Rotor 4');
ylim([0 rpmPlotMax]);

figure;
plot(t, fHist, 'LineWidth', 1.3); hold on;
yline(fHoverPerRotor, '--', 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Rotor Thrust F_i [N]');
title('Individual Rotor Thrusts');
legend('F_1', 'F_2', 'F_3', 'F_4', 'Hover thrust per rotor');
ylim([0 fPlotMax]);

figure;
plot(t, tHist, 'LineWidth', 1.8); hold on;
yline(m*g, '--', 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Total Thrust [N]');
title('Total Upward Thrust');
legend('Total thrust', 'Weight mg');
ylim([0 tPlotMax]);

figure;
plot(t, vHist, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Voltage per motor [V]');
title('Motor Voltage During Takeoff');
legend('V_1', 'V_2', 'V_3', 'V_4');
ylim([0 vPlotMax]);

figure;
plot(t, iHist, 'LineWidth', 1.3); hold on;
yline(motorImax, '--', 'Motor max current');
grid on;
xlabel('Time [s]');
ylabel('Current per motor [A]');
title('Motor Current During Takeoff');
legend('I_1', 'I_2', 'I_3', 'I_4', 'Motor max');
ylim([0 iPlotMax]);

%% ============================================================
%  3D ANIMATION
% ============================================================

rotorPosBody = [ ...
     armLength,  0,          0;
     0,          armLength,  0;
    -armLength,  0,          0;
     0,         -armLength,  0];

figure;
hold on; grid on; axis equal;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title(sprintf('3D Animation: Vertical Takeoff to %.2f m', zRefFinal));

xlim([-0.6 0.6]);
ylim([-0.6 0.6]);
zlim([0 zPlotMax]);
view(35, 25);

plot3(zeros(size(zHist)), zeros(size(zHist)), zHist, 'k--');

[xg, yg] = meshgrid(-0.6:0.2:0.6, -0.6:0.2:0.6);
zg = zeros(size(xg));
mesh(xg, yg, zg, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2);

[xr, yr] = meshgrid(-0.4:0.2:0.4, -0.4:0.2:0.4);
zr = zRefFinal*ones(size(xr));
mesh(xr, yr, zr, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2);

hArm1 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
hArm2 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
hCenter = plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);

hRotors = gobjects(nRotors,1);

for i = 1:nRotors
    hRotors(i) = plot3(0, 0, 0, 'o', 'MarkerSize', 12, 'LineWidth', 2);
end

for k = 1:animationSkip:nt

    center = [0, 0, zHist(k)];
    rotorWorld = rotorPosBody + center;

    set(hArm1, ...
        'XData', [rotorWorld(1,1), rotorWorld(3,1)], ...
        'YData', [rotorWorld(1,2), rotorWorld(3,2)], ...
        'ZData', [rotorWorld(1,3), rotorWorld(3,3)]);

    set(hArm2, ...
        'XData', [rotorWorld(2,1), rotorWorld(4,1)], ...
        'YData', [rotorWorld(2,2), rotorWorld(4,2)], ...
        'ZData', [rotorWorld(2,3), rotorWorld(4,3)]);

    set(hCenter, ...
        'XData', center(1), ...
        'YData', center(2), ...
        'ZData', center(3));

    for i = 1:nRotors
        set(hRotors(i), ...
            'XData', rotorWorld(i,1), ...
            'YData', rotorWorld(i,2), ...
            'ZData', rotorWorld(i,3));
    end

    drawnow;
end