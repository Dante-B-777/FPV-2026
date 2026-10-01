%% KEYBOARD LQR QUADROTOR SIMULATION WITH HOOPS
% Keyboard -> Euler commands -> LQR -> Mixer -> Motors -> Nonlinear dynamics -> 3D animation
%
% Controls:
%   W / S       pitch forward / backward
%   A / D       roll left / right
%   Q / E       yaw left / right
%   Up / Down   climb / descend
%   Space       zero roll and pitch commands
%   R           reset simulation
%   Esc         stop simulation
%
% Goal:
%   Fly through all 3 hoops. After the third hoop:
%   "happy fathers day!" appears.

clear; clc; close all;

%% ================= USER PARAMETERS =================

MASS = 0.50;                 % kg
G = 9.81;                    % m/s^2

ARM_LENGTH = 0.15;           % m, center to motor
IXX = 0.0023;                % kg*m^2
IYY = 0.0023;                % kg*m^2
IZZ = 0.0040;                % kg*m^2

KF = 1.8e-6;                 % N/(rad/s)^2, thrust coefficient
KM = 2.5e-8;                 % Nm/(rad/s)^2, yaw drag coefficient
YAW_MOMENT_RATIO = KM / KF;  % tau_z = c * F

MAX_TOTAL_THRUST = 3.0 * MASS * G;
MAX_MOTOR_THRUST = MAX_TOTAL_THRUST / 4;

MAX_TORQUE_ROLL = 0.20;      % Nm
MAX_TORQUE_PITCH = 0.20;     % Nm
MAX_TORQUE_YAW = 0.08;       % Nm

TS = 0.01;                   % s, physics timestep, 100 Hz
ANIMATION_SKIP = 3;          % update graphics every N physics steps

MAX_ROLL_CMD = deg2rad(15);  % rad
MAX_PITCH_CMD = deg2rad(15); % rad
YAW_RATE_CMD = deg2rad(45);  % rad/s
Z_RATE_CMD = 0.5;            % m/s

INITIAL_ALTITUDE = 1.0;      % m
SIM_TIME_LIMIT = inf;        % seconds, use inf for unlimited

%% ================= HOOP PARAMETERS =================

% Since W pitches the drone forward in the negative-X direction,
% the hoops are placed in the negative-X direction.
HOOPS = [ ...
   -0.9,  0.0, 1.00;
   -1.8,  0.5, 1.25;
   -2.7, -0.4, 1.05];

HOOP_RADIUS = 0.45;          % visual radius [m]
HOOP_PASS_RADIUS = 0.35;     % must pass within this radius in the hoop opening
HOOP_PLANE_TOL = 0.18;       % distance from hoop plane required to count pass

NUM_HOOPS = size(HOOPS,1);
passedHoops = false(NUM_HOOPS,1);
currentHoop = 1;

%% ================= PARAMETER STRUCT =================

params.m = MASS;
params.g = G;
params.L = ARM_LENGTH;
params.I = diag([IXX, IYY, IZZ]);
params.Ixx = IXX;
params.Iyy = IYY;
params.Izz = IZZ;
params.kf = KF;
params.km = KM;
params.c = YAW_MOMENT_RATIO;

params.maxTotalThrust = MAX_TOTAL_THRUST;
params.maxMotorThrust = MAX_MOTOR_THRUST;
params.maxTauRoll = MAX_TORQUE_ROLL;
params.maxTauPitch = MAX_TORQUE_PITCH;
params.maxTauYaw = MAX_TORQUE_YAW;

%% ================= LQR DESIGN =================

% State:
% x = [X Y Z VX VY VZ PHI THETA PSI P Q R]'
%
% Input to LQR:
% u = [delta_T tau_phi tau_theta tau_psi]'
%
% Total thrust:
% T = m*g + delta_T

A = zeros(12,12);
B = zeros(12,4);

% Position derivatives
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Small-angle hover translational acceleration
A(4,8) = G;       % Xddot approx g*theta
A(5,7) = -G;      % Yddot approx -g*phi

% Euler angle derivatives
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

% Inputs
B(6,1)  = 1/MASS;
B(10,2) = 1/IXX;
B(11,3) = 1/IYY;
B(12,4) = 1/IZZ;

% LQR weights
Q_lqr = diag([ ...
    0.5, 0.5, 45, ...      % X Y Z
    1.0, 1.0, 12, ...      % VX VY VZ
    140, 140, 60, ...      % PHI THETA PSI
    12, 12, 6]);           % P Q R

R_lqr = diag([ ...
    1.0, ...               % delta thrust
    0.12, ...              % roll torque
    0.12, ...              % pitch torque
    0.35]);                % yaw torque

K = lqr(A, B, Q_lqr, R_lqr);

disp('LQR gain K:');
disp(K);

%% ================= MIXER MATRIX =================
% Plus configuration:
%
% Motor layout:
%
%        Motor 1 front
%             +X
%              |
% Motor 4 -----+----- Motor 2
%   left       |       right
%             -X
%        Motor 3 rear
%
% Inputs:
% U = [T_total; tau_phi; tau_theta; tau_psi]

MIXER = [ ...
    1,  1,  1,  1;
    0,  ARM_LENGTH, 0, -ARM_LENGTH;
   -ARM_LENGTH, 0, ARM_LENGTH, 0;
    YAW_MOMENT_RATIO, -YAW_MOMENT_RATIO, YAW_MOMENT_RATIO, -YAW_MOMENT_RATIO];

%% ================= GLOBAL KEYBOARD COMMAND STATE =================

global KEY_CMD SIM_RUNNING RESET_REQUEST
KEY_CMD.forward = 0;
KEY_CMD.side = 0;
KEY_CMD.yaw = 0;
KEY_CMD.climb = 0;

SIM_RUNNING = true;
RESET_REQUEST = false;

%% ================= FIGURE AND 3D DRONE =================

fig = figure( ...
    'Name', 'Keyboard LQR Quadrotor Simulator With Hoops', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'KeyPressFcn', @keyDown, ...
    'KeyReleaseFcn', @keyUp);

ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
view(ax, 35, 25);

% Expanded limits so info box can sit outside the flyable hoop zone
xlim(ax, [-5 1]);
ylim(ax, [-4 2.5]);
zlim(ax, [0 3]);

title(ax, 'Fly Through the 3 Hoops');

% Ground
[Xg, Yg] = meshgrid(-5:1:5, -5:1:5);
Zg = zeros(size(Xg));
surf(ax, Xg, Yg, Zg, ...
    'FaceAlpha', 0.08, ...
    'EdgeAlpha', 0.25);

% Hoops
hoopHandles = gobjects(NUM_HOOPS,1);
for i = 1:NUM_HOOPS
    hoopHandles(i) = drawHoop(ax, HOOPS(i,:), HOOP_RADIUS, i, [0.1 0.35 1.0]);
end

% Drone graphics
drone.arm1 = plot3(ax, [0 0], [0 0], [0 0], 'k-', 'LineWidth', 3);
drone.arm2 = plot3(ax, [0 0], [0 0], [0 0], 'k-', 'LineWidth', 3);
drone.body = plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

drone.rotor = gobjects(4,1);
for i = 1:4
    drone.rotor(i) = plot3(ax, 0, 0, 0, 'o', ...
        'MarkerSize', 14, ...
        'LineWidth', 2);
end

drone.trail = animatedline(ax, 'LineWidth', 1.5);

% Moved far left/outside flyable zone so it does not cover the Father's Day text
infoText = text(ax, -4.8, -3.8, 2.8, '', ...
    'FontName', 'Consolas', ...
    'FontSize', 10, ...
    'BackgroundColor', 'w', ...
    'EdgeColor', [0.7 0.7 0.7], ...
    'Margin', 6, ...
    'Clipping', 'off');

fatherText = text(ax, -1.8, 0, 2.45, '', ...
    'FontSize', 28, ...
    'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', ...
    'Color', [0.9 0.05 0.05]);

%% ================= INITIAL CONDITIONS =================

x = zeros(12,1);
x(3) = INITIAL_ALTITUDE;

phi_ref = 0;
theta_ref = 0;
psi_ref = 0;
z_ref = INITIAL_ALTITUDE;

t = 0;
step = 0;

disp('Simulation started.');
disp('Click the figure window before pressing keys.');
disp('Controls: W/S pitch, A/D roll, Q/E yaw, Up/Down altitude, Space hover, R reset, Esc stop.');
disp('Goal: Fly through Hoop 1, Hoop 2, and Hoop 3.');

%% ================= MAIN REAL-TIME LOOP =================

while SIM_RUNNING && ishandle(fig) && t < SIM_TIME_LIMIT

    loopStart = tic;

    if RESET_REQUEST
        x = zeros(12,1);
        x(3) = INITIAL_ALTITUDE;

        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 0;
        z_ref = INITIAL_ALTITUDE;

        passedHoops(:) = false;
        currentHoop = 1;

        clearpoints(drone.trail);
        set(fatherText, 'String', '');

        for i = 1:NUM_HOOPS
            set(hoopHandles(i), 'Color', [0.1 0.35 1.0]);
        end

        RESET_REQUEST = false;
    end

    % -------- Keyboard to Euler references --------
    phi_ref   =  KEY_CMD.side    * MAX_ROLL_CMD;
    theta_ref = -KEY_CMD.forward * MAX_PITCH_CMD;

    psi_ref = psi_ref + KEY_CMD.yaw   * YAW_RATE_CMD * TS;
    z_ref   = z_ref   + KEY_CMD.climb * Z_RATE_CMD   * TS;

    z_ref = max(0.1, min(3.0, z_ref));

    % -------- Reference state --------
    x_ref = zeros(12,1);

    % Do not regulate X/Y position in Euler command mode.
    % This lets the drone translate when you command roll/pitch.
    x_ref(1) = x(1);
    x_ref(2) = x(2);

    % Regulate altitude and Euler attitude.
    x_ref(3) = z_ref;
    x_ref(7) = phi_ref;
    x_ref(8) = theta_ref;
    x_ref(9) = psi_ref;

    % -------- LQR control --------
    x_err = x - x_ref;

    % Wrap yaw error to avoid discontinuity around pi.
    x_err(9) = wrapToPiLocal(x_err(9));

    u_lqr = -K * x_err;

    delta_T = u_lqr(1);
    tau_phi = u_lqr(2);
    tau_theta = u_lqr(3);
    tau_psi = u_lqr(4);

    T_total = MASS * G + delta_T;

    % -------- Saturation --------
    T_total = clamp(T_total, 0, MAX_TOTAL_THRUST);
    tau_phi = clamp(tau_phi, -MAX_TORQUE_ROLL, MAX_TORQUE_ROLL);
    tau_theta = clamp(tau_theta, -MAX_TORQUE_PITCH, MAX_TORQUE_PITCH);
    tau_psi = clamp(tau_psi, -MAX_TORQUE_YAW, MAX_TORQUE_YAW);

    U = [T_total; tau_phi; tau_theta; tau_psi];

    % -------- Mixer --------
    F = MIXER \ U;
    F = clamp(F, 0, MAX_MOTOR_THRUST);

    omega = sqrt(F / KF); %#ok<NASGU>

    % -------- Nonlinear dynamics with RK4 integration --------
    x = rk4Step(@quadDynamicsEuler, x, F, TS, params);

    % Prevent ground penetration
    if x(3) < 0
        x(3) = 0;
        if x(6) < 0
            x(6) = 0;
        end
    end

    % -------- Hoop detection --------
    if currentHoop <= NUM_HOOPS
        hoopCenter = HOOPS(currentHoop,:);

        planeError = abs(x(1) - hoopCenter(1));
        radialError = sqrt((x(2) - hoopCenter(2))^2 + (x(3) - hoopCenter(3))^2);

        if planeError <= HOOP_PLANE_TOL && radialError <= HOOP_PASS_RADIUS
            passedHoops(currentHoop) = true;
            set(hoopHandles(currentHoop), 'Color', [0.0 0.75 0.15]);

            fprintf('Passed hoop %d!\n', currentHoop);

            currentHoop = currentHoop + 1;

            if currentHoop > NUM_HOOPS
                set(fatherText, 'String', 'happy fathers day!');
                disp('happy fathers day!');
            end
        end
    end

    % -------- Update animation --------
    if mod(step, ANIMATION_SKIP) == 0
        updateDroneGraphics(drone, x, ARM_LENGTH);
        addpoints(drone.trail, x(1), x(2), x(3));

        if currentHoop <= NUM_HOOPS
            hoopStatus = sprintf('target hoop = %d / %d', currentHoop, NUM_HOOPS);
        else
            hoopStatus = 'all hoops complete';
        end

        infoString = sprintf([ ...
            't = %.2f s\n' ...
            'pos  [%.2f %.2f %.2f] m\n' ...
            'eul  [%.1f %.1f %.1f] deg\n' ...
            'cmd  [roll %.1f pitch %.1f yaw %.1f z %.2f]\n' ...
            'T = %.2f N\n' ...
            'motors F = [%.2f %.2f %.2f %.2f] N\n' ...
            'hoops passed = %d / %d\n' ...
            '%s'], ...
            t, ...
            x(1), x(2), x(3), ...
            rad2deg(x(7)), rad2deg(x(8)), rad2deg(x(9)), ...
            rad2deg(phi_ref), rad2deg(theta_ref), rad2deg(psi_ref), z_ref, ...
            T_total, ...
            F(1), F(2), F(3), F(4), ...
            sum(passedHoops), NUM_HOOPS, ...
            hoopStatus);

        set(infoText, 'String', infoString);

        drawnow limitrate;
    end

    % -------- Real-time loop timing --------
    elapsed = toc(loopStart);
    pause(max(0, TS - elapsed));

    t = t + TS;
    step = step + 1;
end

disp('Simulation stopped.');

%% ================= LOCAL FUNCTIONS =================

function x_next = rk4Step(f, x, u, dt, params)
    k1 = f(x, u, params);
    k2 = f(x + 0.5*dt*k1, u, params);
    k3 = f(x + 0.5*dt*k2, u, params);
    k4 = f(x + dt*k3, u, params);

    x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    % Keep yaw bounded
    x_next(9) = wrapToPiLocal(x_next(9));
end

function xdot = quadDynamicsEuler(x, F, params)
    % State:
    % x = [X Y Z VX VY VZ PHI THETA PSI P Q R]'

    m = params.m;
    g = params.g;
    L = params.L;
    I = params.I;
    c = params.c;

    phi = x(7);
    theta = x(8);
    psi = x(9);

    p = x(10);
    q = x(11);
    r = x(12);

    % Motor thrusts
    F1 = F(1); % front
    F2 = F(2); % right
    F3 = F(3); % rear
    F4 = F(4); % left

    T = F1 + F2 + F3 + F4;

    tau_phi   = L*(F2 - F4);
    tau_theta = L*(F3 - F1);
    tau_psi   = c*(F1 - F2 + F3 - F4);

    tau = [tau_phi; tau_theta; tau_psi];

    Rwb = eulToRotmZYX(phi, theta, psi);

    % Body thrust points along body +Z.
    thrust_world = Rwb * [0; 0; T];

    accel = thrust_world / m - [0; 0; g];

    omega = [p; q; r];
    omega_dot = I \ (tau - cross(omega, I*omega));

    % Euler angle kinematics
    E = [ ...
        1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
        0, cos(phi),            -sin(phi);
        0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

    euler_dot = E * omega;

    xdot = zeros(12,1);

    xdot(1) = x(4);
    xdot(2) = x(5);
    xdot(3) = x(6);

    xdot(4) = accel(1);
    xdot(5) = accel(2);
    xdot(6) = accel(3);

    xdot(7) = euler_dot(1);
    xdot(8) = euler_dot(2);
    xdot(9) = euler_dot(3);

    xdot(10) = omega_dot(1);
    xdot(11) = omega_dot(2);
    xdot(12) = omega_dot(3);
end

function R = eulToRotmZYX(phi, theta, psi)
    % Rotation from body frame to world frame
    % ZYX Euler sequence: yaw psi, pitch theta, roll phi

    cphi = cos(phi);   sphi = sin(phi);
    cth = cos(theta);  sth = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    Rz = [ ...
        cpsi, -spsi, 0;
        spsi,  cpsi, 0;
        0,     0,    1];

    Ry = [ ...
        cth, 0, sth;
        0,   1, 0;
       -sth, 0, cth];

    Rx = [ ...
        1, 0,    0;
        0, cphi, -sphi;
        0, sphi,  cphi];

    R = Rz * Ry * Rx;
end

function updateDroneGraphics(drone, x, L)
    pos = x(1:3);

    phi = x(7);
    theta = x(8);
    psi = x(9);

    R = eulToRotmZYX(phi, theta, psi);

    % Body-frame motor positions
    front = [ L; 0; 0];
    right = [0;  L; 0];
    rear  = [-L; 0; 0];
    left  = [0; -L; 0];

    p_front = pos + R*front;
    p_right = pos + R*right;
    p_rear  = pos + R*rear;
    p_left  = pos + R*left;

    set(drone.arm1, ...
        'XData', [p_front(1), p_rear(1)], ...
        'YData', [p_front(2), p_rear(2)], ...
        'ZData', [p_front(3), p_rear(3)]);

    set(drone.arm2, ...
        'XData', [p_left(1), p_right(1)], ...
        'YData', [p_left(2), p_right(2)], ...
        'ZData', [p_left(3), p_right(3)]);

    set(drone.body, ...
        'XData', pos(1), ...
        'YData', pos(2), ...
        'ZData', pos(3));

    rotorPts = [p_front, p_right, p_rear, p_left];

    for i = 1:4
        set(drone.rotor(i), ...
            'XData', rotorPts(1,i), ...
            'YData', rotorPts(2,i), ...
            'ZData', rotorPts(3,i));
    end
end

function h = drawHoop(ax, center, radius, hoopNumber, colorValue)
    % Hoop lies in the Y-Z plane, normal to the X axis.
    th = linspace(0, 2*pi, 200);

    x = center(1) * ones(size(th));
    y = center(2) + radius*cos(th);
    z = center(3) + radius*sin(th);

    h = plot3(ax, x, y, z, ...
        'LineWidth', 5, ...
        'Color', colorValue);

    plot3(ax, center(1), center(2), center(3), ...
        'kx', ...
        'MarkerSize', 8, ...
        'LineWidth', 1.5);

    text(ax, center(1), center(2), center(3) + radius + 0.18, ...
        sprintf('Hoop %d', hoopNumber), ...
        'FontWeight', 'bold', ...
        'FontSize', 11, ...
        'HorizontalAlignment', 'center', ...
        'Color', colorValue);
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

function keyDown(~, event)
    global KEY_CMD SIM_RUNNING RESET_REQUEST

    switch lower(event.Key)
        case 'w'
            KEY_CMD.forward = 1;
        case 's'
            KEY_CMD.forward = -1;
        case 'a'
            KEY_CMD.side = -1;
        case 'd'
            KEY_CMD.side = 1;
        case 'q'
            KEY_CMD.yaw = -1;
        case 'e'
            KEY_CMD.yaw = 1;
        case 'uparrow'
            KEY_CMD.climb = 1;
        case 'downarrow'
            KEY_CMD.climb = -1;
        case 'space'
            KEY_CMD.forward = 0;
            KEY_CMD.side = 0;
        case 'r'
            RESET_REQUEST = true;
        case 'escape'
            SIM_RUNNING = false;
    end
end

function keyUp(~, event)
    global KEY_CMD

    switch lower(event.Key)
        case {'w','s'}
            KEY_CMD.forward = 0;
        case {'a','d'}
            KEY_CMD.side = 0;
        case {'q','e'}
            KEY_CMD.yaw = 0;
        case {'uparrow','downarrow'}
            KEY_CMD.climb = 0;
    end
end