%% LQR QUADROTOR CIRCULAR HOVER SIMULATION
% State:
% x = [X Y Z XDOT YDOT ZDOT PHI THETA PSI P Q R]'
%
% Input:
% u = [DELTA_THRUST TAU_ROLL TAU_PITCH TAU_YAW]'
%
% The controller is designed from the linearized hover model:
% xdot = A*x + B*u
%
% The plant simulation uses nonlinear Euler-angle dynamics.

clear; clc; close all;

%% ============================================================
%  USER INPUT PARAMETERS
%  ============================================================

MASS = 0.50;                 % kg
GRAVITY = 9.81;              % m/s^2

IXX = 0.0023;                % kg*m^2
IYY = 0.0023;                % kg*m^2
IZZ = 0.0040;                % kg*m^2

ARM_LENGTH = 0.10;           % meters, center to motor

KF = 6.0e-6;                 % thrust coefficient, N/(rad/s)^2
KM = 1.5e-7;                 % drag torque coefficient, N*m/(rad/s)^2

MAX_MOTOR_RPM = 30000;       % maximum motor rpm
MIN_MOTOR_RPM = 0;           % minimum motor rpm

CIRCLE_RADIUS = 1.0;         % meters
CIRCLE_ALTITUDE = 1.0;       % meters
CIRCLE_SPEED = 0.50;         % m/s tangential speed

SIM_TIME = 30;               % seconds

INITIAL_X = 1.0;             % meters
INITIAL_Y = 0.0;             % meters
INITIAL_Z = 1.0;             % meters

INITIAL_XDOT = 0.0;          % m/s
INITIAL_YDOT = 0.50;         % m/s
INITIAL_ZDOT = 0.0;          % m/s

INITIAL_PHI = 0.0;           % rad
INITIAL_THETA = 0.0;         % rad
INITIAL_PSI = 0.0;           % rad

INITIAL_P = 0.0;             % rad/s
INITIAL_Q = 0.0;             % rad/s
INITIAL_R = 0.0;             % rad/s

%% ============================================================
%  LQR WEIGHTS
%  ============================================================

% State order:
% X Y Z XDOT YDOT ZDOT PHI THETA PSI P Q R

Q_LQR = diag([ ...
    20, 20, 80, ...      % X, Y, Z
    10, 10, 20, ...      % XDOT, YDOT, ZDOT
    150, 150, 80, ...    % PHI, THETA, PSI
    10, 10, 10]);        % P, Q, R

% Input order:
% DELTA_THRUST, TAU_ROLL, TAU_PITCH, TAU_YAW

R_LQR = diag([ ...
    1.0, ...             % DELTA_THRUST
    0.05, ...            % TAU_ROLL
    0.05, ...            % TAU_PITCH
    0.10]);              % TAU_YAW

%% ============================================================
%  BUILD LINEARIZED STATE-SPACE MODEL ABOUT HOVER
%  ============================================================

A = zeros(12,12);
B = zeros(12,4);

% Position derivatives
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Small-angle translational coupling near hover
% XDDOT =  g*THETA
% YDDOT = -g*PHI
% ZDDOT = DELTA_THRUST / MASS
A(4,8) = GRAVITY;
A(5,7) = -GRAVITY;

% Euler angle derivatives near hover
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

% Input mapping
B(6,1) = 1/MASS;
B(10,2) = 1/IXX;
B(11,3) = 1/IYY;
B(12,4) = 1/IZZ;

C = eye(12);
D = zeros(12,4);

%% ============================================================
%  CHECK CONTROLLABILITY AND COMPUTE LQR GAIN
%  ============================================================

CONTROLLABILITY_RANK = rank(ctrb(A,B));

fprintf('Controllability rank = %d out of 12\n', CONTROLLABILITY_RANK);

if CONTROLLABILITY_RANK < 12
    warning('The linearized system is not fully controllable.');
end

K = lqr(A,B,Q_LQR,R_LQR);

CLOSED_LOOP_EIGS = eig(A - B*K);

disp('LQR gain matrix K = ');
disp(K);

disp('Closed-loop eigenvalues = ');
disp(CLOSED_LOOP_EIGS);

%% ============================================================
%  INITIAL STATE
%  ============================================================

X0 = [ ...
    INITIAL_X;
    INITIAL_Y;
    INITIAL_Z;
    INITIAL_XDOT;
    INITIAL_YDOT;
    INITIAL_ZDOT;
    INITIAL_PHI;
    INITIAL_THETA;
    INITIAL_PSI;
    INITIAL_P;
    INITIAL_Q;
    INITIAL_R];

%% ============================================================
%  SIMULATE WITH ODE45
%  ============================================================

PARAMS.MASS = MASS;
PARAMS.GRAVITY = GRAVITY;
PARAMS.IXX = IXX;
PARAMS.IYY = IYY;
PARAMS.IZZ = IZZ;
PARAMS.ARM_LENGTH = ARM_LENGTH;
PARAMS.KF = KF;
PARAMS.KM = KM;
PARAMS.MAX_MOTOR_RPM = MAX_MOTOR_RPM;
PARAMS.MIN_MOTOR_RPM = MIN_MOTOR_RPM;
PARAMS.CIRCLE_RADIUS = CIRCLE_RADIUS;
PARAMS.CIRCLE_ALTITUDE = CIRCLE_ALTITUDE;
PARAMS.CIRCLE_SPEED = CIRCLE_SPEED;
PARAMS.K = K;

TSPAN = [0 SIM_TIME];

OPTIONS = odeset( ...
    'RelTol',1e-6, ...
    'AbsTol',1e-8);

[TIME, STATE] = ode45(@(t,x) quad_lqr_dynamics(t,x,PARAMS), TSPAN, X0, OPTIONS);

%% ============================================================
%  POST-PROCESS REFERENCE AND CONTROL DATA
%  ============================================================

N = length(TIME);

REF = zeros(N,12);
CONTROL = zeros(N,4);
MOTOR_RPM = zeros(N,4);
MOTOR_THRUST = zeros(N,4);

for i = 1:N
    [REF(i,:), CONTROL(i,:), MOTOR_RPM(i,:), MOTOR_THRUST(i,:)] = ...
        controller_output(TIME(i), STATE(i,:)', PARAMS);
end

%% ============================================================
%  PLOT 3D TRAJECTORY
%  ============================================================

figure;
plot3(STATE(:,1), STATE(:,2), STATE(:,3), 'LineWidth', 2);
hold on;
plot3(REF(:,1), REF(:,2), REF(:,3), '--', 'LineWidth', 2);
grid on;
axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
zlabel('Z Position [m]');
title('3D Quadrotor Circular Hover Trajectory');
legend('Actual', 'Reference');

%% ============================================================
%  PLOT POSITION STATES
%  ============================================================

figure;
plot(TIME, STATE(:,1), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,1), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('X [m]');
title('X Position');
legend('Actual', 'Reference');

figure;
plot(TIME, STATE(:,2), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,2), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Y [m]');
title('Y Position');
legend('Actual', 'Reference');

figure;
plot(TIME, STATE(:,3), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,3), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Z [m]');
title('Z Position');
legend('Actual', 'Reference');

%% ============================================================
%  PLOT VELOCITY STATES
%  ============================================================

figure;
plot(TIME, STATE(:,4), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,4), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('XDOT [m/s]');
title('X Velocity');
legend('Actual', 'Reference');

figure;
plot(TIME, STATE(:,5), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,5), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('YDOT [m/s]');
title('Y Velocity');
legend('Actual', 'Reference');

figure;
plot(TIME, STATE(:,6), 'LineWidth', 1.5); hold on;
plot(TIME, REF(:,6), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('ZDOT [m/s]');
title('Z Velocity');
legend('Actual', 'Reference');

%% ============================================================
%  PLOT EULER ANGLES
%  ============================================================

figure;
plot(TIME, rad2deg(STATE(:,7)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,7)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Roll PHI [deg]');
title('Roll Angle');
legend('Actual', 'Reference');

figure;
plot(TIME, rad2deg(STATE(:,8)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,8)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Pitch THETA [deg]');
title('Pitch Angle');
legend('Actual', 'Reference');

figure;
plot(TIME, rad2deg(STATE(:,9)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,9)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Yaw PSI [deg]');
title('Yaw Angle');
legend('Actual', 'Reference');

%% ============================================================
%  PLOT BODY RATES
%  ============================================================

figure;
plot(TIME, rad2deg(STATE(:,10)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,10)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('P [deg/s]');
title('Roll Rate P');
legend('Actual', 'Reference');

figure;
plot(TIME, rad2deg(STATE(:,11)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,11)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Q [deg/s]');
title('Pitch Rate Q');
legend('Actual', 'Reference');

figure;
plot(TIME, rad2deg(STATE(:,12)), 'LineWidth', 1.5); hold on;
plot(TIME, rad2deg(REF(:,12)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('R [deg/s]');
title('Yaw Rate R');
legend('Actual', 'Reference');

%% ============================================================
%  PLOT CONTROL INPUTS
%  ============================================================

figure;
plot(TIME, CONTROL(:,1), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\Delta T [N]');
title('LQR Thrust Correction');

figure;
plot(TIME, CONTROL(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\tau_\phi [N*m]');
title('Roll Torque Command');

figure;
plot(TIME, CONTROL(:,3), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\tau_\theta [N*m]');
title('Pitch Torque Command');

figure;
plot(TIME, CONTROL(:,4), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('\tau_\psi [N*m]');
title('Yaw Torque Command');

%% ============================================================
%  PLOT MOTOR SPEEDS
%  ============================================================

figure;
plot(TIME, MOTOR_RPM(:,1), 'LineWidth', 1.2); hold on;
plot(TIME, MOTOR_RPM(:,2), 'LineWidth', 1.2);
plot(TIME, MOTOR_RPM(:,3), 'LineWidth', 1.2);
plot(TIME, MOTOR_RPM(:,4), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Motor Speed [RPM]');
title('Motor Speeds');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');

%% ============================================================
%  ANIMATION
%  ============================================================

figure;
for i = 1:10:N
    clf;

    plot3(REF(:,1), REF(:,2), REF(:,3), '--', 'LineWidth', 1.2);
    hold on;
    plot3(STATE(1:i,1), STATE(1:i,2), STATE(1:i,3), 'LineWidth', 2);

    X = STATE(i,1);
    Y = STATE(i,2);
    Z = STATE(i,3);
    PHI = STATE(i,7);
    THETA = STATE(i,8);
    PSI = STATE(i,9);

    R_BI = rotation_matrix_zyx(PHI, THETA, PSI);

    ARM_1_BODY = [ ARM_LENGTH; 0; 0];
    ARM_2_BODY = [0; ARM_LENGTH; 0];
    ARM_3_BODY = [-ARM_LENGTH; 0; 0];
    ARM_4_BODY = [0; -ARM_LENGTH; 0];

    CENTER = [X; Y; Z];

    P1 = CENTER + R_BI*ARM_1_BODY;
    P2 = CENTER + R_BI*ARM_2_BODY;
    P3 = CENTER + R_BI*ARM_3_BODY;
    P4 = CENTER + R_BI*ARM_4_BODY;

    plot3([P1(1) P3(1)], [P1(2) P3(2)], [P1(3) P3(3)], 'k', 'LineWidth', 3);
    plot3([P2(1) P4(1)], [P2(2) P4(2)], [P2(3) P4(3)], 'k', 'LineWidth', 3);

    plot3(P1(1), P1(2), P1(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
    plot3(P2(1), P2(2), P2(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
    plot3(P3(1), P3(2), P3(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
    plot3(P4(1), P4(2), P4(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);

    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title(sprintf('Quadrotor LQR Circular Hover, t = %.2f s', TIME(i)));

    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    zlim([0 1.5]);

    view(45,25);
    drawnow;
end

%% ============================================================
%  LOCAL FUNCTIONS
%  ============================================================

function XDOT = quad_lqr_dynamics(t, x, PARAMS)

    MASS = PARAMS.MASS;
    GRAVITY = PARAMS.GRAVITY;
    IXX = PARAMS.IXX;
    IYY = PARAMS.IYY;
    IZZ = PARAMS.IZZ;

    PHI = x(7);
    THETA = x(8);
    PSI = x(9);

    P = x(10);
    Q = x(11);
    R = x(12);

    [~, CONTROL, ~, MOTOR_THRUST] = controller_output(t, x, PARAMS);

    TOTAL_THRUST_COMMAND = MASS*GRAVITY + CONTROL(1);
    TAU_ROLL_COMMAND = CONTROL(2);
    TAU_PITCH_COMMAND = CONTROL(3);
    TAU_YAW_COMMAND = CONTROL(4);

    % Reconstruct achieved thrust/torque after motor saturation.
    ARM_LENGTH = PARAMS.ARM_LENGTH;
    KF = PARAMS.KF;
    KM = PARAMS.KM;

    YAW_TORQUE_PER_THRUST = KM/KF;

    F1 = MOTOR_THRUST(1);
    F2 = MOTOR_THRUST(2);
    F3 = MOTOR_THRUST(3);
    F4 = MOTOR_THRUST(4);

    TOTAL_THRUST = F1 + F2 + F3 + F4;

    TAU_ROLL = ARM_LENGTH*(F2 - F4);
    TAU_PITCH = ARM_LENGTH*(F3 - F1);
    TAU_YAW = YAW_TORQUE_PER_THRUST*(F1 - F2 + F3 - F4);

    % If you want no motor saturation effect, uncomment this block:
    % TOTAL_THRUST = TOTAL_THRUST_COMMAND;
    % TAU_ROLL = TAU_ROLL_COMMAND;
    % TAU_PITCH = TAU_PITCH_COMMAND;
    % TAU_YAW = TAU_YAW_COMMAND;

    R_BI = rotation_matrix_zyx(PHI, THETA, PSI);

    THRUST_BODY = [0; 0; TOTAL_THRUST];
    ACCEL_INERTIAL = (R_BI*THRUST_BODY)/MASS - [0; 0; GRAVITY];

    OMEGA = [P; Q; R];
    INERTIA = diag([IXX, IYY, IZZ]);
    TAU = [TAU_ROLL; TAU_PITCH; TAU_YAW];

    OMEGA_DOT = INERTIA \ (TAU - cross(OMEGA, INERTIA*OMEGA));

    EULER_DOT = euler_rates(PHI, THETA, OMEGA);

    XDOT = zeros(12,1);

    XDOT(1) = x(4);
    XDOT(2) = x(5);
    XDOT(3) = x(6);

    XDOT(4) = ACCEL_INERTIAL(1);
    XDOT(5) = ACCEL_INERTIAL(2);
    XDOT(6) = ACCEL_INERTIAL(3);

    XDOT(7) = EULER_DOT(1);
    XDOT(8) = EULER_DOT(2);
    XDOT(9) = EULER_DOT(3);

    XDOT(10) = OMEGA_DOT(1);
    XDOT(11) = OMEGA_DOT(2);
    XDOT(12) = OMEGA_DOT(3);
end

function [REF_ROW, CONTROL_ROW, MOTOR_RPM_ROW, MOTOR_THRUST_ROW] = controller_output(t, x, PARAMS)

    MASS = PARAMS.MASS;
    GRAVITY = PARAMS.GRAVITY;
    IXX = PARAMS.IXX;
    IYY = PARAMS.IYY;
    IZZ = PARAMS.IZZ;
    ARM_LENGTH = PARAMS.ARM_LENGTH;
    KF = PARAMS.KF;
    KM = PARAMS.KM;
    K = PARAMS.K;

    CIRCLE_RADIUS = PARAMS.CIRCLE_RADIUS;
    CIRCLE_ALTITUDE = PARAMS.CIRCLE_ALTITUDE;
    CIRCLE_SPEED = PARAMS.CIRCLE_SPEED;

    OMEGA_CIRCLE = CIRCLE_SPEED / CIRCLE_RADIUS;

    XD = CIRCLE_RADIUS*cos(OMEGA_CIRCLE*t);
    YD = CIRCLE_RADIUS*sin(OMEGA_CIRCLE*t);
    ZD = CIRCLE_ALTITUDE;

    XDOTD = -CIRCLE_RADIUS*OMEGA_CIRCLE*sin(OMEGA_CIRCLE*t);
    YDOTD =  CIRCLE_RADIUS*OMEGA_CIRCLE*cos(OMEGA_CIRCLE*t);
    ZDOTD = 0;

    XDDOTD = -CIRCLE_RADIUS*OMEGA_CIRCLE^2*cos(OMEGA_CIRCLE*t);
    YDDOTD = -CIRCLE_RADIUS*OMEGA_CIRCLE^2*sin(OMEGA_CIRCLE*t);
    ZDDOTD = 0;

    % Small-angle feedforward:
    % XDDOT =  g*THETA
    % YDDOT = -g*PHI
    PHID = -YDDOTD/GRAVITY;
    THETAD = XDDOTD/GRAVITY;
    PSID = 0;

    PHIDOTD = (CIRCLE_RADIUS*OMEGA_CIRCLE^3*cos(OMEGA_CIRCLE*t))/GRAVITY;
    THETADOTD = (CIRCLE_RADIUS*OMEGA_CIRCLE^3*sin(OMEGA_CIRCLE*t))/GRAVITY;
    PSIDOTD = 0;

    PHIDDOTD = -(CIRCLE_RADIUS*OMEGA_CIRCLE^4*sin(OMEGA_CIRCLE*t))/GRAVITY;
    THETADDOTD =  (CIRCLE_RADIUS*OMEGA_CIRCLE^4*cos(OMEGA_CIRCLE*t))/GRAVITY;
    PSIDDOTD = 0;

    PD = PHIDOTD;
    QD = THETADOTD;
    RD = PSIDOTD;

    P_DOT_D = PHIDDOTD;
    Q_DOT_D = THETADDOTD;
    R_DOT_D = PSIDDOTD;

    XREF = [ ...
        XD;
        YD;
        ZD;
        XDOTD;
        YDOTD;
        ZDOTD;
        PHID;
        THETAD;
        PSID;
        PD;
        QD;
        RD];

    ERROR = x - XREF;

    % Wrap yaw error to avoid 2*pi jump.
    ERROR(9) = atan2(sin(ERROR(9)), cos(ERROR(9)));

    % Feedforward input around hover.
    DELTA_THRUST_FF = MASS*ZDDOTD;
    TAU_ROLL_FF = IXX*P_DOT_D;
    TAU_PITCH_FF = IYY*Q_DOT_D;
    TAU_YAW_FF = IZZ*R_DOT_D;

    U_FF = [ ...
        DELTA_THRUST_FF;
        TAU_ROLL_FF;
        TAU_PITCH_FF;
        TAU_YAW_FF];

    U = U_FF - K*ERROR;

    TOTAL_THRUST_COMMAND = MASS*GRAVITY + U(1);

    VIRTUAL_COMMAND = [ ...
        TOTAL_THRUST_COMMAND;
        U(2);
        U(3);
        U(4)];

    % Plus-configuration motor mixing.
    % Motor layout:
    % Motor 1: front
    % Motor 2: right
    % Motor 3: rear
    % Motor 4: left
    %
    % [T; tau_phi; tau_theta; tau_psi] =
    % [1  1  1  1;
    %  0  L  0 -L;
    % -L  0  L  0;
    %  c -c  c -c] * [F1; F2; F3; F4]

    YAW_TORQUE_PER_THRUST = KM/KF;

    MIXER = [ ...
        1, 1, 1, 1;
        0, ARM_LENGTH, 0, -ARM_LENGTH;
        -ARM_LENGTH, 0, ARM_LENGTH, 0;
        YAW_TORQUE_PER_THRUST, -YAW_TORQUE_PER_THRUST, YAW_TORQUE_PER_THRUST, -YAW_TORQUE_PER_THRUST];

    MOTOR_THRUST = MIXER \ VIRTUAL_COMMAND;

    MOTOR_THRUST = max(MOTOR_THRUST, 0);

    MOTOR_OMEGA = sqrt(MOTOR_THRUST / KF);

    MOTOR_RPM = MOTOR_OMEGA * 60/(2*pi);

    MOTOR_RPM = min(MOTOR_RPM, PARAMS.MAX_MOTOR_RPM);
    MOTOR_RPM = max(MOTOR_RPM, PARAMS.MIN_MOTOR_RPM);

    MOTOR_OMEGA = MOTOR_RPM * 2*pi/60;
    MOTOR_THRUST = KF * MOTOR_OMEGA.^2;

    REF_ROW = XREF';
    CONTROL_ROW = U';
    MOTOR_RPM_ROW = MOTOR_RPM';
    MOTOR_THRUST_ROW = MOTOR_THRUST';
end

function R = rotation_matrix_zyx(phi, theta, psi)

    CPHI = cos(phi);
    SPHI = sin(phi);

    CTHETA = cos(theta);
    STHETA = sin(theta);

    CPSI = cos(psi);
    SPSI = sin(psi);

    R = [ ...
        CPSI*CTHETA, CPSI*STHETA*SPHI - SPSI*CPHI, CPSI*STHETA*CPHI + SPSI*SPHI;
        SPSI*CTHETA, SPSI*STHETA*SPHI + CPSI*CPHI, SPSI*STHETA*CPHI - CPSI*SPHI;
        -STHETA,      CTHETA*SPHI,                       CTHETA*CPHI];
end

function EULER_DOT = euler_rates(phi, theta, omega)

    P = omega(1);
    Q = omega(2);
    R = omega(3);

    CPHI = cos(phi);
    SPHI = sin(phi);
    CTHETA = cos(theta);
    TTHETA = tan(theta);

    PHIDOT = P + SPHI*TTHETA*Q + CPHI*TTHETA*R;
    THETADOT = CPHI*Q - SPHI*R;
    PSIDOT = (SPHI/CTHETA)*Q + (CPHI/CTHETA)*R;

    EULER_DOT = [PHIDOT; THETADOT; PSIDOT];
end