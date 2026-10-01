clear; clc; close all;

%% ============================================================
%  QUADROTOR CIRCLE SIMULATION USING ODE45 RUNGE-KUTTA METHOD
%  ============================================================

%% =========================
%  INPUT PARAMETERS
%  =========================

MASS = 0.50;                    % kg
GRAVITY = 9.81;                 % m/s^2

IXX = 0.0023;                   % kg*m^2
IYY = 0.0023;                   % kg*m^2
IZZ = 0.0040;                   % kg*m^2

ARM_LENGTH = 0.10;              % m
KF = 1.0e-5;                    % N/(rad/s)^2
KM = 2.0e-7;                    % N*m/(rad/s)^2

MIN_MOTOR_SPEED = 0;            % rad/s
MAX_MOTOR_SPEED = 2500;         % rad/s

SIM_TIME = 20;                  % seconds
OUTPUT_TIME_STEP = 0.01;        % seconds

CIRCLE_RADIUS = 1.0;            % m
CIRCLE_ALTITUDE = 1.0;          % m
CIRCLE_ANGULAR_SPEED = 2.5;     % rad/s

KP_X = 4.0;
KD_X = 3.2;

KP_Y = 4.0;
KD_Y = 3.2;

KP_Z = 10.0;
KD_Z = 6.0;

KP_ROLL = 35.0;
KD_ROLL = 8.0;

KP_PITCH = 35.0;
KD_PITCH = 8.0;

KP_YAW = 10.0;
KD_YAW = 4.0;

MAX_TILT_ANGLE = deg2rad(25);   % rad

INITIAL_STATE = zeros(12,1);

INITIAL_STATE(1) = CIRCLE_RADIUS;       % x
INITIAL_STATE(2) = 0;                   % y
INITIAL_STATE(3) = CIRCLE_ALTITUDE;     % z

INITIAL_STATE(4) = 0;                                           % xdot
INITIAL_STATE(5) = CIRCLE_RADIUS * CIRCLE_ANGULAR_SPEED;         % ydot
INITIAL_STATE(6) = 0;                                           % zdot

INITIAL_STATE(7) = 0;                   % roll
INITIAL_STATE(8) = 0;                   % pitch
INITIAL_STATE(9) = pi/2;                % yaw

INITIAL_STATE(10) = 0;                  % roll rate
INITIAL_STATE(11) = 0;                  % pitch rate
INITIAL_STATE(12) = 0;                  % yaw rate

%% =========================
%  PARAMETER STRUCT
%  =========================

P.MASS = MASS;
P.GRAVITY = GRAVITY;

P.IXX = IXX;
P.IYY = IYY;
P.IZZ = IZZ;

P.ARM_LENGTH = ARM_LENGTH;
P.KF = KF;
P.KM = KM;

P.MIN_MOTOR_SPEED = MIN_MOTOR_SPEED;
P.MAX_MOTOR_SPEED = MAX_MOTOR_SPEED;

P.CIRCLE_RADIUS = CIRCLE_RADIUS;
P.CIRCLE_ALTITUDE = CIRCLE_ALTITUDE;
P.CIRCLE_ANGULAR_SPEED = CIRCLE_ANGULAR_SPEED;

P.KP_X = KP_X;
P.KD_X = KD_X;

P.KP_Y = KP_Y;
P.KD_Y = KD_Y;

P.KP_Z = KP_Z;
P.KD_Z = KD_Z;

P.KP_ROLL = KP_ROLL;
P.KD_ROLL = KD_ROLL;

P.KP_PITCH = KP_PITCH;
P.KD_PITCH = KD_PITCH;

P.KP_YAW = KP_YAW;
P.KD_YAW = KD_YAW;

P.MAX_TILT_ANGLE = MAX_TILT_ANGLE;

%% =========================
%  ODE45 RUNGE-KUTTA INTEGRATION
%  =========================

TIME_OUTPUT = 0:OUTPUT_TIME_STEP:SIM_TIME;

OPTIONS = odeset( ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-8 ...
);

[TIME, STATE_HISTORY] = ode45(@(TIME, STATE) quadrotor_circle_ode(TIME, STATE, P), ...
                              TIME_OUTPUT, ...
                              INITIAL_STATE, ...
                              OPTIONS);

%% =========================
%  COMPUTE DESIRED TRAJECTORY AND MOTOR SPEEDS FOR PLOTTING
%  =========================

NUM_STEPS = length(TIME);

DESIRED_HISTORY = zeros(NUM_STEPS, 4);
MOTOR_HISTORY = zeros(NUM_STEPS, 4);

for K = 1:NUM_STEPS

    CURRENT_TIME = TIME(K);
    CURRENT_STATE = STATE_HISTORY(K,:).';

    [~, DESIRED_OUTPUT, MOTOR_SPEEDS] = quadrotor_circle_dynamics(CURRENT_TIME, CURRENT_STATE, P);

    DESIRED_HISTORY(K,:) = DESIRED_OUTPUT.';
    MOTOR_HISTORY(K,:) = MOTOR_SPEEDS.';

end

%% =========================
%  PLOT ALL STATE VARIABLES
%  =========================

plot_all_states(TIME, STATE_HISTORY, DESIRED_HISTORY, MOTOR_HISTORY, ...
    'ODE45 Runge-Kutta Method');

%% =========================
%  3D VISUALIZATION
%  =========================

animate_quadrotor(TIME, STATE_HISTORY, DESIRED_HISTORY, P, ...
    'ODE45 Runge-Kutta 3D Quadrotor Circle Simulation');

%% ============================================================
%  LOCAL FUNCTIONS
%  ============================================================

function STATE_DOT = quadrotor_circle_ode(TIME, STATE, P)

    [STATE_DOT, ~, ~] = quadrotor_circle_dynamics(TIME, STATE, P);

end

function [STATE_DOT, DESIRED_OUTPUT, MOTOR_SPEEDS] = quadrotor_circle_dynamics(TIME, STATE, P)

    X = STATE(1);
    Y = STATE(2);
    Z = STATE(3);

    XD = STATE(4);
    YD = STATE(5);
    ZD = STATE(6);

    ROLL = STATE(7);
    PITCH = STATE(8);
    YAW = STATE(9);

    ROLL_RATE = STATE(10);
    PITCH_RATE = STATE(11);
    YAW_RATE = STATE(12);

    R = P.CIRCLE_RADIUS;
    W = P.CIRCLE_ANGULAR_SPEED;
    H = P.CIRCLE_ALTITUDE;

    X_DES = R * cos(W * TIME);
    Y_DES = R * sin(W * TIME);
    Z_DES = H;

    XD_DES = -R * W * sin(W * TIME);
    YD_DES =  R * W * cos(W * TIME);
    ZD_DES = 0;

    XDD_DES_FF = -R * W^2 * cos(W * TIME);
    YDD_DES_FF = -R * W^2 * sin(W * TIME);
    ZDD_DES_FF = 0;

    YAW_DES = atan2(YD_DES, XD_DES);

    X_ERROR = X_DES - X;
    Y_ERROR = Y_DES - Y;
    Z_ERROR = Z_DES - Z;

    XD_ERROR = XD_DES - XD;
    YD_ERROR = YD_DES - YD;
    ZD_ERROR = ZD_DES - ZD;

    AX_COMMAND = XDD_DES_FF + P.KP_X * X_ERROR + P.KD_X * XD_ERROR;
    AY_COMMAND = YDD_DES_FF + P.KP_Y * Y_ERROR + P.KD_Y * YD_ERROR;
    AZ_COMMAND = ZDD_DES_FF + P.KP_Z * Z_ERROR + P.KD_Z * ZD_ERROR;

    PITCH_DES = (AX_COMMAND * cos(YAW) + AY_COMMAND * sin(YAW)) / P.GRAVITY;
    ROLL_DES  = (AX_COMMAND * sin(YAW) - AY_COMMAND * cos(YAW)) / P.GRAVITY;

    PITCH_DES = saturate(PITCH_DES, -P.MAX_TILT_ANGLE, P.MAX_TILT_ANGLE);
    ROLL_DES  = saturate(ROLL_DES,  -P.MAX_TILT_ANGLE, P.MAX_TILT_ANGLE);

    DENOMINATOR = cos(ROLL) * cos(PITCH);

    if abs(DENOMINATOR) < 0.1
        DENOMINATOR = sign(DENOMINATOR) * 0.1;
    end

    U1 = P.MASS * (P.GRAVITY + AZ_COMMAND) / DENOMINATOR;
    U1 = max(U1, 0);

    ROLL_ERROR = ROLL_DES - ROLL;
    PITCH_ERROR = PITCH_DES - PITCH;
    YAW_ERROR = wrap_to_pi(YAW_DES - YAW);

    ROLL_RATE_ERROR = 0 - ROLL_RATE;
    PITCH_RATE_ERROR = 0 - PITCH_RATE;
    YAW_RATE_ERROR = 0 - YAW_RATE;

    U2 = P.IXX * (P.KP_ROLL * ROLL_ERROR + P.KD_ROLL * ROLL_RATE_ERROR);
    U3 = P.IYY * (P.KP_PITCH * PITCH_ERROR + P.KD_PITCH * PITCH_RATE_ERROR);
    U4 = P.IZZ * (P.KP_YAW * YAW_ERROR + P.KD_YAW * YAW_RATE_ERROR);

    MIXING_MATRIX = [
        P.KF,              P.KF,              P.KF,              P.KF;
        0,                 P.ARM_LENGTH*P.KF, 0,                -P.ARM_LENGTH*P.KF;
       -P.ARM_LENGTH*P.KF, 0,                 P.ARM_LENGTH*P.KF, 0;
        P.KM,             -P.KM,              P.KM,             -P.KM
    ];

    U_VECTOR = [U1; U2; U3; U4];

    MOTOR_SPEED_SQUARED = MIXING_MATRIX \ U_VECTOR;

    MOTOR_SPEED_SQUARED = max(MOTOR_SPEED_SQUARED, P.MIN_MOTOR_SPEED^2);
    MOTOR_SPEED_SQUARED = min(MOTOR_SPEED_SQUARED, P.MAX_MOTOR_SPEED^2);

    MOTOR_SPEEDS = sqrt(MOTOR_SPEED_SQUARED);

    ACTUAL_U = MIXING_MATRIX * MOTOR_SPEED_SQUARED;

    U1 = ACTUAL_U(1);
    U2 = ACTUAL_U(2);
    U3 = ACTUAL_U(3);
    U4 = ACTUAL_U(4);

    XDD = (U1 / P.MASS) * ...
        (cos(YAW) * sin(PITCH) * cos(ROLL) + sin(YAW) * sin(ROLL));

    YDD = (U1 / P.MASS) * ...
        (sin(YAW) * sin(PITCH) * cos(ROLL) - cos(YAW) * sin(ROLL));

    ZDD = (U1 / P.MASS) * ...
        (cos(PITCH) * cos(ROLL)) - P.GRAVITY;

    ROLL_DD = U2 / P.IXX;
    PITCH_DD = U3 / P.IYY;
    YAW_DD = U4 / P.IZZ;

    STATE_DOT = zeros(12,1);

    STATE_DOT(1) = XD;
    STATE_DOT(2) = YD;
    STATE_DOT(3) = ZD;

    STATE_DOT(4) = XDD;
    STATE_DOT(5) = YDD;
    STATE_DOT(6) = ZDD;

    STATE_DOT(7) = ROLL_RATE;
    STATE_DOT(8) = PITCH_RATE;
    STATE_DOT(9) = YAW_RATE;

    STATE_DOT(10) = ROLL_DD;
    STATE_DOT(11) = PITCH_DD;
    STATE_DOT(12) = YAW_DD;

    DESIRED_OUTPUT = [X_DES; Y_DES; Z_DES; YAW_DES];

end

function plot_all_states(TIME, STATE_HISTORY, DESIRED_HISTORY, MOTOR_HISTORY, TITLE_PREFIX)

    figure('Name', [TITLE_PREFIX ' - Position States']);
    plot(TIME, STATE_HISTORY(:,1), 'LineWidth', 1.5); hold on;
    plot(TIME, STATE_HISTORY(:,2), 'LineWidth', 1.5);
    plot(TIME, STATE_HISTORY(:,3), 'LineWidth', 1.5);
    plot(TIME, DESIRED_HISTORY(:,1), '--', 'LineWidth', 1.2);
    plot(TIME, DESIRED_HISTORY(:,2), '--', 'LineWidth', 1.2);
    plot(TIME, DESIRED_HISTORY(:,3), '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('x', 'y', 'z', 'x desired', 'y desired', 'z desired');
    title([TITLE_PREFIX ' - Position States']);

    figure('Name', [TITLE_PREFIX ' - Velocity States']);
    plot(TIME, STATE_HISTORY(:,4), 'LineWidth', 1.5); hold on;
    plot(TIME, STATE_HISTORY(:,5), 'LineWidth', 1.5);
    plot(TIME, STATE_HISTORY(:,6), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    legend('xdot', 'ydot', 'zdot');
    title([TITLE_PREFIX ' - Velocity States']);

    figure('Name', [TITLE_PREFIX ' - Euler Angle States']);
    plot(TIME, rad2deg(STATE_HISTORY(:,7)), 'LineWidth', 1.5); hold on;
    plot(TIME, rad2deg(STATE_HISTORY(:,8)), 'LineWidth', 1.5);
    plot(TIME, rad2deg(STATE_HISTORY(:,9)), 'LineWidth', 1.5);
    plot(TIME, rad2deg(DESIRED_HISTORY(:,4)), '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    legend('roll phi', 'pitch theta', 'yaw psi', 'yaw desired');
    title([TITLE_PREFIX ' - Euler Angle States']);

    figure('Name', [TITLE_PREFIX ' - Angular Rate States']);
    plot(TIME, rad2deg(STATE_HISTORY(:,10)), 'LineWidth', 1.5); hold on;
    plot(TIME, rad2deg(STATE_HISTORY(:,11)), 'LineWidth', 1.5);
    plot(TIME, rad2deg(STATE_HISTORY(:,12)), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Angular Rate [deg/s]');
    legend('roll rate', 'pitch rate', 'yaw rate');
    title([TITLE_PREFIX ' - Angular Rate States']);

    figure('Name', [TITLE_PREFIX ' - Motor Speeds']);
    plot(TIME, MOTOR_HISTORY(:,1), 'LineWidth', 1.5); hold on;
    plot(TIME, MOTOR_HISTORY(:,2), 'LineWidth', 1.5);
    plot(TIME, MOTOR_HISTORY(:,3), 'LineWidth', 1.5);
    plot(TIME, MOTOR_HISTORY(:,4), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Motor Speed [rad/s]');
    legend('\omega_1', '\omega_2', '\omega_3', '\omega_4');
    title([TITLE_PREFIX ' - Motor Speeds']);

end

function animate_quadrotor(TIME, STATE_HISTORY, DESIRED_HISTORY, P, FIGURE_TITLE)

    figure('Name', FIGURE_TITLE);
    hold on; grid on; axis equal;

    plot3(DESIRED_HISTORY(:,1), DESIRED_HISTORY(:,2), DESIRED_HISTORY(:,3), ...
        '--', 'LineWidth', 1.5);

    plot3(STATE_HISTORY(:,1), STATE_HISTORY(:,2), STATE_HISTORY(:,3), ...
        'LineWidth', 1.5);

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(FIGURE_TITLE);

    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    zlim([0 1.8]);
    view(45, 25);

    ARM_LINE_1 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
    ARM_LINE_2 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
    BODY_POINT = plot3(0, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

    SKIP = max(1, floor(length(TIME)/700));

    for K = 1:SKIP:length(TIME)

        POS = STATE_HISTORY(K,1:3).';
        ROLL = STATE_HISTORY(K,7);
        PITCH = STATE_HISTORY(K,8);
        YAW = STATE_HISTORY(K,9);

        ROTATION = rotation_matrix_zyx(ROLL, PITCH, YAW);

        ARM_X_BODY = [P.ARM_LENGTH; 0; 0];
        ARM_Y_BODY = [0; P.ARM_LENGTH; 0];

        P1 = POS + ROTATION * ARM_X_BODY;
        P2 = POS - ROTATION * ARM_X_BODY;
        P3 = POS + ROTATION * ARM_Y_BODY;
        P4 = POS - ROTATION * ARM_Y_BODY;

        set(ARM_LINE_1, 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)], 'ZData', [P1(3) P2(3)]);
        set(ARM_LINE_2, 'XData', [P3(1) P4(1)], 'YData', [P3(2) P4(2)], 'ZData', [P3(3) P4(3)]);
        set(BODY_POINT, 'XData', POS(1), 'YData', POS(2), 'ZData', POS(3));

        drawnow;

    end

end

function R = rotation_matrix_zyx(ROLL, PITCH, YAW)

    CPHI = cos(ROLL);
    SPHI = sin(ROLL);

    CTHETA = cos(PITCH);
    STHETA = sin(PITCH);

    CPSI = cos(YAW);
    SPSI = sin(YAW);

    R = [
        CPSI*CTHETA, CPSI*STHETA*SPHI - SPSI*CPHI, CPSI*STHETA*CPHI + SPSI*SPHI;
        SPSI*CTHETA, SPSI*STHETA*SPHI + CPSI*CPHI, SPSI*STHETA*CPHI - CPSI*SPHI;
        -STHETA,     CTHETA*SPHI,                     CTHETA*CPHI
    ];

end

function VALUE = saturate(VALUE, LOWER_LIMIT, UPPER_LIMIT)
    VALUE = min(max(VALUE, LOWER_LIMIT), UPPER_LIMIT);
end

function ANGLE = wrap_to_pi(ANGLE)
    ANGLE = mod(ANGLE + pi, 2*pi) - pi;
end