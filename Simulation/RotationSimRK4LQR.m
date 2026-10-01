%% CLEAN LQR QUADROTOR CIRCULAR HOVER SIMULATION
% Euler-angle nonlinear quadrotor model
% RK4 integration
% LQR designed from hover-linearized state-space model
%
% State:
% X = [x y z xdot ydot zdot phi theta psi p q r]'
%
% Virtual control:
% U = [delta_thrust tau_roll tau_pitch tau_yaw]'
%
% This version intentionally applies virtual thrust/torque directly
% to the nonlinear plant. Motor mixing is computed only for diagnostics.

clear; clc; close all;

%% ============================================================
%  USER INPUT PARAMETERS
%  ============================================================

MASS = 0.50;                     % kg
GRAVITY = 9.81;                  % m/s^2

IXX = 0.0023;                    % kg*m^2
IYY = 0.0023;                    % kg*m^2
IZZ = 0.0040;                    % kg*m^2

ARM_LENGTH = 0.10;               % m

KF = 6.0e-6;                     % N/(rad/s)^2
KM = 1.5e-7;                     % N*m/(rad/s)^2

MAX_MOTOR_RPM = 30000;           % rpm

CIRCLE_RADIUS = 1.0;             % m
CIRCLE_ALTITUDE = 1.0;           % m
CIRCLE_SPEED = 0.50;             % m/s

SIM_TIME = 30.0;                 % s
DT = 0.01;                       % s

ANIMATION_FPS = 30;
REAL_TIME_ANIMATION = true;
RUN_ANIMATION = true;
RUN_PLOTS = true;

INITIAL_X = 1.0;
INITIAL_Y = 0.0;
INITIAL_Z = 1.0;

INITIAL_XDOT = 0.0;
INITIAL_YDOT = 0.50;
INITIAL_ZDOT = 0.0;

INITIAL_PHI = 0.0;
INITIAL_THETA = 0.0;
INITIAL_PSI = 0.0;

INITIAL_P = 0.0;
INITIAL_Q = 0.0;
INITIAL_R = 0.0;

%% ============================================================
%  LQR WEIGHTS
%  ============================================================

Q_LQR = diag([ ...
    50, 50, 120, ...       % x y z
    20, 20, 35, ...        % xdot ydot zdot
    250, 250, 80, ...      % phi theta psi
    25, 25, 20]);          % p q r

R_LQR = diag([ ...
    1.0, ...               % delta_thrust
    0.03, ...              % tau_roll
    0.03, ...              % tau_pitch
    0.15]);                % tau_yaw

%% ============================================================
%  LINEARIZED HOVER MODEL
%  ============================================================

A = zeros(12,12);
B = zeros(12,4);

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% For the rotation matrix used below:
% xddot =  g*theta
% yddot = -g*phi
A(4,8) = GRAVITY;
A(5,7) = -GRAVITY;

A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B(6,1) = 1/MASS;
B(10,2) = 1/IXX;
B(11,3) = 1/IYY;
B(12,4) = 1/IZZ;

C = eye(12);
D = zeros(12,4);

fprintf('Controllability rank = %d out of 12\n', rank(ctrb(A,B)));

K = lqr(A,B,Q_LQR,R_LQR);

disp('Closed-loop eigenvalues:');
disp(eig(A - B*K));

%% ============================================================
%  PACK PARAMETERS
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
PARAMS.CIRCLE_RADIUS = CIRCLE_RADIUS;
PARAMS.CIRCLE_ALTITUDE = CIRCLE_ALTITUDE;
PARAMS.CIRCLE_SPEED = CIRCLE_SPEED;
PARAMS.K = K;

%% ============================================================
%  PREALLOCATE
%  ============================================================

TIME = 0:DT:SIM_TIME;
N = length(TIME);

STATE = zeros(N,12);
REF = zeros(N,12);
CONTROL = zeros(N,4);
MOTOR_RPM = zeros(N,4);
MOTOR_THRUST = zeros(N,4);

STATE(1,:) = [ ...
    INITIAL_X, INITIAL_Y, INITIAL_Z, ...
    INITIAL_XDOT, INITIAL_YDOT, INITIAL_ZDOT, ...
    INITIAL_PHI, INITIAL_THETA, INITIAL_PSI, ...
    INITIAL_P, INITIAL_Q, INITIAL_R];

%% ============================================================
%  RK4 SIMULATION
%  ============================================================

fprintf('Running RK4 simulation...\n');
tic;

for k = 1:N-1
    t = TIME(k);
    x = STATE(k,:)';

    k1 = quad_dynamics_virtual(t, x, PARAMS);
    k2 = quad_dynamics_virtual(t + DT/2, x + DT*k1/2, PARAMS);
    k3 = quad_dynamics_virtual(t + DT/2, x + DT*k2/2, PARAMS);
    k4 = quad_dynamics_virtual(t + DT, x + DT*k3, PARAMS);

    x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

    x_next(9) = wrap_angle(x_next(9));

    STATE(k+1,:) = x_next';
end

fprintf('Simulation completed in %.3f seconds.\n', toc);

%% ============================================================
%  POST-PROCESS
%  ============================================================

for k = 1:N
    [REF(k,:), CONTROL(k,:), MOTOR_RPM(k,:), MOTOR_THRUST(k,:)] = ...
        controller_output(TIME(k), STATE(k,:)', PARAMS);
end

POSITION_ERROR = STATE(:,1:3) - REF(:,1:3);
POSITION_ERROR_MAG = sqrt(sum(POSITION_ERROR.^2,2));

fprintf('\nSimulation summary:\n');
fprintf('RMS position error = %.4f m\n', sqrt(mean(POSITION_ERROR_MAG.^2)));
fprintf('Max position error = %.4f m\n', max(POSITION_ERROR_MAG));
fprintf('Max roll angle     = %.2f deg\n', max(abs(rad2deg(STATE(:,7)))));
fprintf('Max pitch angle    = %.2f deg\n', max(abs(rad2deg(STATE(:,8)))));
fprintf('Max altitude       = %.3f m\n', max(STATE(:,3)));
fprintf('Min altitude       = %.3f m\n', min(STATE(:,3)));
fprintf('Max motor RPM diag = %.0f rpm\n', max(MOTOR_RPM(:)));

if RUN_ANIMATION
    animate_quadrotor(TIME, STATE, REF, PARAMS, ANIMATION_FPS, REAL_TIME_ANIMATION);
end

if RUN_PLOTS
    plot_results(TIME, STATE, REF, CONTROL, MOTOR_RPM);
end

%% ============================================================
%  LOCAL FUNCTIONS
%  ============================================================

function XDOT = quad_dynamics_virtual(t, x, P)

    MASS = P.MASS;
    GRAVITY = P.GRAVITY;

    IXX = P.IXX;
    IYY = P.IYY;
    IZZ = P.IZZ;

    PHI = x(7);
    THETA = x(8);
    PSI = x(9);

    P_RATE = x(10);
    Q_RATE = x(11);
    R_RATE = x(12);

    [~, U, ~, ~] = controller_output(t, x, P);

    DELTA_THRUST = U(1);
    TAU_ROLL = U(2);
    TAU_PITCH = U(3);
    TAU_YAW = U(4);

    TOTAL_THRUST = MASS*GRAVITY + DELTA_THRUST;

    % Prevent impossible negative total thrust.
    TOTAL_THRUST = max(TOTAL_THRUST, 0);

    R_BI = rotation_matrix_zyx(PHI, THETA, PSI);

    FORCE_BODY = [0; 0; TOTAL_THRUST];
    ACCEL = (R_BI*FORCE_BODY)/MASS - [0; 0; GRAVITY];

    OMEGA = [P_RATE; Q_RATE; R_RATE];
    INERTIA = diag([IXX, IYY, IZZ]);
    TAU = [TAU_ROLL; TAU_PITCH; TAU_YAW];

    OMEGA_DOT = INERTIA \ (TAU - cross(OMEGA, INERTIA*OMEGA));

    EULER_DOT = euler_rates(PHI, THETA, OMEGA);

    XDOT = zeros(12,1);

    XDOT(1) = x(4);
    XDOT(2) = x(5);
    XDOT(3) = x(6);

    XDOT(4) = ACCEL(1);
    XDOT(5) = ACCEL(2);
    XDOT(6) = ACCEL(3);

    XDOT(7) = EULER_DOT(1);
    XDOT(8) = EULER_DOT(2);
    XDOT(9) = EULER_DOT(3);

    XDOT(10) = OMEGA_DOT(1);
    XDOT(11) = OMEGA_DOT(2);
    XDOT(12) = OMEGA_DOT(3);
end

function [REF_ROW, CONTROL_ROW, MOTOR_RPM_ROW, MOTOR_THRUST_ROW] = controller_output(t, x, P)

    MASS = P.MASS;
    GRAVITY = P.GRAVITY;
    K = P.K;

    RADIUS = P.CIRCLE_RADIUS;
    ALTITUDE = P.CIRCLE_ALTITUDE;
    SPEED = P.CIRCLE_SPEED;

    OMEGA_C = SPEED/RADIUS;

    XD = RADIUS*cos(OMEGA_C*t);
    YD = RADIUS*sin(OMEGA_C*t);
    ZD = ALTITUDE;

    XDOTD = -RADIUS*OMEGA_C*sin(OMEGA_C*t);
    YDOTD =  RADIUS*OMEGA_C*cos(OMEGA_C*t);
    ZDOTD = 0;

    XDDOTD = -RADIUS*OMEGA_C^2*cos(OMEGA_C*t);
    YDDOTD = -RADIUS*OMEGA_C^2*sin(OMEGA_C*t);
    ZDDOTD = 0;

    % Since:
    % xddot =  g*theta
    % yddot = -g*phi
    PHID = -YDDOTD/GRAVITY;
    THETAD = XDDOTD/GRAVITY;
    PSID = 0;

    PHIDOTD =  (RADIUS*OMEGA_C^3*cos(OMEGA_C*t))/GRAVITY;
    THETADOTD = (RADIUS*OMEGA_C^3*sin(OMEGA_C*t))/GRAVITY;
    PSIDOTD = 0;

    PHIDDOTD =  -(RADIUS*OMEGA_C^4*sin(OMEGA_C*t))/GRAVITY;
    THETADDOTD =  (RADIUS*OMEGA_C^4*cos(OMEGA_C*t))/GRAVITY;
    PSIDDOTD = 0;

    PD = PHIDOTD;
    QD = THETADOTD;
    RD = PSIDOTD;

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
    ERROR(9) = wrap_angle(ERROR(9));

    % Feedforward input.
    DELTA_THRUST_FF = MASS*ZDDOTD;
    TAU_ROLL_FF = P.IXX*PHIDDOTD;
    TAU_PITCH_FF = P.IYY*THETADDOTD;
    TAU_YAW_FF = P.IZZ*PSIDDOTD;

    U_FF = [ ...
        DELTA_THRUST_FF;
        TAU_ROLL_FF;
        TAU_PITCH_FF;
        TAU_YAW_FF];

    U = U_FF - K*ERROR;

    % Safety clamp for simulation stability.
    MAX_DELTA_THRUST = 2.0*MASS*GRAVITY;
    MAX_TAU_ROLL = 0.20;
    MAX_TAU_PITCH = 0.20;
    MAX_TAU_YAW = 0.10;

    U(1) = clamp(U(1), -0.90*MASS*GRAVITY, MAX_DELTA_THRUST);
    U(2) = clamp(U(2), -MAX_TAU_ROLL, MAX_TAU_ROLL);
    U(3) = clamp(U(3), -MAX_TAU_PITCH, MAX_TAU_PITCH);
    U(4) = clamp(U(4), -MAX_TAU_YAW, MAX_TAU_YAW);

    [MOTOR_RPM, MOTOR_THRUST] = motor_diagnostic(MASS*GRAVITY + U(1), U(2), U(3), U(4), P);

    REF_ROW = XREF';
    CONTROL_ROW = U';
    MOTOR_RPM_ROW = MOTOR_RPM';
    MOTOR_THRUST_ROW = MOTOR_THRUST';
end

function [MOTOR_RPM, MOTOR_THRUST] = motor_diagnostic(TOTAL_THRUST, TAU_ROLL, TAU_PITCH, TAU_YAW, P)

    L = P.ARM_LENGTH;
    C = P.KM/P.KF;

    MIXER = [ ...
        1, 1, 1, 1;
        0, L, 0, -L;
        -L, 0, L, 0;
        C, -C, C, -C];

    VIRTUAL = [TOTAL_THRUST; TAU_ROLL; TAU_PITCH; TAU_YAW];

    MOTOR_THRUST = MIXER \ VIRTUAL;

    MOTOR_THRUST = max(MOTOR_THRUST, 0);

    MOTOR_OMEGA = sqrt(MOTOR_THRUST/P.KF);
    MOTOR_RPM = MOTOR_OMEGA*60/(2*pi);

    MOTOR_RPM = min(MOTOR_RPM, P.MAX_MOTOR_RPM);

    MOTOR_OMEGA = MOTOR_RPM*2*pi/60;
    MOTOR_THRUST = P.KF*MOTOR_OMEGA.^2;
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
        -STHETA,      CTHETA*SPHI,                  CTHETA*CPHI];
end

function EULER_DOT = euler_rates(phi, theta, omega)

    P_RATE = omega(1);
    Q_RATE = omega(2);
    R_RATE = omega(3);

    CPHI = cos(phi);
    SPHI = sin(phi);

    CTHETA = cos(theta);
    TTHETA = tan(theta);

    if abs(CTHETA) < 1e-6
        CTHETA = sign(CTHETA + 1e-9)*1e-6;
    end

    PHIDOT = P_RATE + SPHI*TTHETA*Q_RATE + CPHI*TTHETA*R_RATE;
    THETADOT = CPHI*Q_RATE - SPHI*R_RATE;
    PSIDOT = (SPHI/CTHETA)*Q_RATE + (CPHI/CTHETA)*R_RATE;

    EULER_DOT = [PHIDOT; THETADOT; PSIDOT];
end

function Y = clamp(X, LOW, HIGH)
    Y = min(max(X, LOW), HIGH);
end

function angle_out = wrap_angle(angle_in)
    angle_out = atan2(sin(angle_in), cos(angle_in));
end

function animate_quadrotor(TIME, STATE, REF, P, ANIMATION_FPS, REAL_TIME_ANIMATION)

    N = length(TIME);
    DT_AVG = mean(diff(TIME));
    FRAME_SKIP = max(1, round(1/(ANIMATION_FPS*DT_AVG)));

    L = P.ARM_LENGTH;

    figure('Name','LQR Quadrotor Circular Hover Animation');
    hold on;
    grid on;
    axis equal;
    axis vis3d;

    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');

    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    zlim([0 1.6]);

    view(45,25);

    ref_path = plot3(REF(:,1), REF(:,2), REF(:,3), 'b--', 'LineWidth', 1.5);
    actual_path = plot3(STATE(1,1), STATE(1,2), STATE(1,3), 'r-', 'LineWidth', 2.0);

    arm_x = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 4);
    arm_y = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 4);

    motors = plot3(nan, nan, nan, 'ko', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'y');

    center_marker = plot3(nan, nan, nan, 'ro', ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', 'r');

    legend([ref_path, actual_path], 'Reference circle', 'Actual path', 'Location', 'best');

    t_start = tic;

    for k = 1:FRAME_SKIP:N

        XPOS = STATE(k,1);
        YPOS = STATE(k,2);
        ZPOS = STATE(k,3);

        PHI = STATE(k,7);
        THETA = STATE(k,8);
        PSI = STATE(k,9);

        R_BI = rotation_matrix_zyx(PHI, THETA, PSI);

        CENTER = [XPOS; YPOS; ZPOS];

        P1 = CENTER + R_BI*[ L; 0; 0];
        P2 = CENTER + R_BI*[0; L; 0];
        P3 = CENTER + R_BI*[-L; 0; 0];
        P4 = CENTER + R_BI*[0; -L; 0];

        set(actual_path, ...
            'XData', STATE(1:k,1), ...
            'YData', STATE(1:k,2), ...
            'ZData', STATE(1:k,3));

        set(arm_x, ...
            'XData', [P1(1), P3(1)], ...
            'YData', [P1(2), P3(2)], ...
            'ZData', [P1(3), P3(3)]);

        set(arm_y, ...
            'XData', [P2(1), P4(1)], ...
            'YData', [P2(2), P4(2)], ...
            'ZData', [P2(3), P4(3)]);

        set(motors, ...
            'XData', [P1(1), P2(1), P3(1), P4(1)], ...
            'YData', [P1(2), P2(2), P3(2), P4(2)], ...
            'ZData', [P1(3), P2(3), P3(3), P4(3)]);

        set(center_marker, ...
            'XData', XPOS, ...
            'YData', YPOS, ...
            'ZData', ZPOS);

        title(sprintf('LQR Circular Hover - t = %.2f s', TIME(k)));

        drawnow limitrate;

        if REAL_TIME_ANIMATION
            target_time = TIME(k) - TIME(1);
            elapsed_time = toc(t_start);
            pause_time = target_time - elapsed_time;

            if pause_time > 0
                pause(pause_time);
            end
        end
    end
end

function plot_results(TIME, STATE, REF, CONTROL, MOTOR_RPM)

    STATE_PLOT = STATE;
    REF_PLOT = REF;

    STATE_PLOT(:,7:12) = rad2deg(STATE_PLOT(:,7:12));
    REF_PLOT(:,7:12) = rad2deg(REF_PLOT(:,7:12));

    STATE_NAMES = { ...
        'X [m]', 'Y [m]', 'Z [m]', ...
        'XDOT [m/s]', 'YDOT [m/s]', 'ZDOT [m/s]', ...
        'PHI [deg]', 'THETA [deg]', 'PSI [deg]', ...
        'P [deg/s]', 'Q [deg/s]', 'R [deg/s]'};

    figure('Name','State Variables');
    tiledlayout(4,3);

    for i = 1:12
        nexttile;
        plot(TIME, STATE_PLOT(:,i), 'r-', 'LineWidth', 1.2);
        hold on;
        plot(TIME, REF_PLOT(:,i), 'b--', 'LineWidth', 1.0);
        grid on;
        xlabel('Time [s]');
        ylabel(STATE_NAMES{i});
        title(STATE_NAMES{i});
    end

    legend('Actual','Reference');

    figure('Name','3D Path Check');
    plot3(REF(:,1), REF(:,2), REF(:,3), 'b--', 'LineWidth', 1.8);
    hold on;
    plot3(STATE(:,1), STATE(:,2), STATE(:,3), 'r-', 'LineWidth', 1.8);
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('3D Path: Reference vs Actual');
    legend('Reference','Actual');

    figure('Name','XY Path Check');
    plot(REF(:,1), REF(:,2), 'b--', 'LineWidth', 1.8);
    hold on;
    plot(STATE(:,1), STATE(:,2), 'r-', 'LineWidth', 1.8);
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('XY Path: Reference vs Actual');
    legend('Reference','Actual');

    figure('Name','Altitude');
    plot(TIME, STATE(:,3), 'r-', 'LineWidth', 1.4);
    hold on;
    plot(TIME, REF(:,3), 'b--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Z [m]');
    title('Altitude Tracking');
    legend('Actual','Reference');

    figure('Name','Control Inputs');
    tiledlayout(2,2);

    nexttile;
    plot(TIME, CONTROL(:,1), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('\Delta T [N]');
    title('Delta Thrust');

    nexttile;
    plot(TIME, CONTROL(:,2), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('\tau_\phi [N*m]');
    title('Roll Torque');

    nexttile;
    plot(TIME, CONTROL(:,3), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('\tau_\theta [N*m]');
    title('Pitch Torque');

    nexttile;
    plot(TIME, CONTROL(:,4), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('\tau_\psi [N*m]');
    title('Yaw Torque');

    figure('Name','Motor RPM Diagnostic');
    plot(TIME, MOTOR_RPM(:,1), 'LineWidth', 1.2);
    hold on;
    plot(TIME, MOTOR_RPM(:,2), 'LineWidth', 1.2);
    plot(TIME, MOTOR_RPM(:,3), 'LineWidth', 1.2);
    plot(TIME, MOTOR_RPM(:,4), 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('RPM');
    title('Motor Speeds Diagnostic');
    legend('Motor 1','Motor 2','Motor 3','Motor 4');
end