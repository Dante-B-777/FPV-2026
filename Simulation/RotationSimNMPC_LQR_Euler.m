%% NMPC OUTER LOOP + LQR INNER LOOP QUADROTOR SIMULATION
% Optimized version with:
% - smooth takeoff
% - settle phase
% - circle radius ramp
% - NMPC timing report
% - LQR inner-loop attitude control
% - nonlinear Euler-angle dynamics
%
% Required toolboxes:
% - Optimization Toolbox
% - Control System Toolbox

clear; clc; close all;

%% ============================================================
%  USER INPUT PARAMETERS
% =============================================================

MASS = 0.50;                          % kg
GRAVITY = 9.81;                       % m/s^2

IXX = 0.0023;                         % kg*m^2
IYY = 0.0023;                         % kg*m^2
IZZ = 0.0040;                         % kg*m^2
INERTIA = diag([IXX IYY IZZ]);

MAX_TOTAL_THRUST = 2.5*MASS*GRAVITY;  % N
MIN_TOTAL_THRUST = 0.05*MASS*GRAVITY; % N

MAX_ROLL_CMD_DEG = 25;                % deg
MAX_PITCH_CMD_DEG = 25;               % deg
MAX_YAW_CMD_DEG = 180;                % deg

MAX_TORQUE_ROLL = 0.08;               % N*m
MAX_TORQUE_PITCH = 0.08;              % N*m
MAX_TORQUE_YAW = 0.04;                % N*m

SIM_TIME = 14.0;                      % seconds
DT_INNER = 0.005;                     % 200 Hz LQR loop
DT_OUTER = 0.05;                      % 20 Hz NMPC loop
INNER_STEPS_PER_OUTER = round(DT_OUTER/DT_INNER);

CIRCLE_RADIUS = 1.0;                  % m
CIRCLE_ALTITUDE = 1.0;                % m
CIRCLE_PERIOD = 8.0;                  % s
CIRCLE_OMEGA = 2*pi/CIRCLE_PERIOD;    % rad/s

TAKEOFF_TIME = 2.0;                   % s
SETTLE_TIME = 1.0;                    % s
CIRCLE_RAMP_TIME = 2.0;               % s
CIRCLE_START_TIME = TAKEOFF_TIME + SETTLE_TIME;

NMPC_HORIZON = 8;                     % shorter horizon for speed
NMPC_DT = DT_OUTER;

%% ============================================================
%  NMPC WEIGHTS
% =============================================================

Q_POS = diag([20 20 90]);             % x,y,z tracking
Q_VEL = diag([5 5 15]);               % vx,vy,vz tracking
Q_ATT_CMD = diag([0.5 0.5 0.1]);      % predicted attitude command penalty

R_INPUT = diag([0.02 0.9 0.9 0.15]);  % [T, phi_cmd, theta_cmd, psi_cmd]
R_DINPUT = diag([0.25 1.5 1.5 0.2]);  % smoothness penalty

%% ============================================================
%  INNER LQR WEIGHTS
% =============================================================

Q_LQR = diag([250 250 80 8 8 4]);     % [phi theta psi p q r]
R_LQR = diag([1.0 1.0 1.5]);          % [tau_phi tau_theta tau_psi]

%% ============================================================
%  INITIAL STATE
% =============================================================

% x = [X Y Z VX VY VZ PHI THETA PSI P Q R]'
x = zeros(12,1);
x(1) = 0.0;
x(2) = 0.0;
x(3) = 0.05;
x(9) = 0.0;

%% ============================================================
%  INNER LOOP LQR DESIGN
% =============================================================

A_ATT = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0];

B_ATT = [0       0       0;
         0       0       0;
         0       0       0;
         1/IXX   0       0;
         0       1/IYY   0;
         0       0       1/IZZ];

K_LQR = lqr(A_ATT, B_ATT, Q_LQR, R_LQR);

disp('Inner-loop LQR gain K_LQR = ');
disp(K_LQR);

%% ============================================================
%  SIMULATION STORAGE
% =============================================================

N_SIM = floor(SIM_TIME/DT_INNER) + 1;

TIME = zeros(1,N_SIM);
XHIST = zeros(12,N_SIM);
REFHIST = zeros(12,N_SIM);
UHIST = zeros(4,N_SIM);               % [T tau_phi tau_theta tau_psi]
CMDHIST = zeros(4,N_SIM);             % [T phi_cmd theta_cmd psi_cmd]

XHIST(:,1) = x;

NMPC_SOLVE_TIME = [];
NMPC_EXITFLAGS = [];

TOTAL_SIM_TIMER = tic;

%% ============================================================
%  FMINCON OPTIONS
% =============================================================

options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','none', ...
    'MaxIterations',25, ...
    'MaxFunctionEvaluations',3000, ...
    'ConstraintTolerance',1e-3, ...
    'StepTolerance',1e-4, ...
    'OptimalityTolerance',1e-3);

%% ============================================================
%  NMPC INITIALIZATION
% =============================================================

u_outer_prev = [MASS*GRAVITY; 0; 0; 0];
u_outer_sequence_prev = repmat(u_outer_prev, NMPC_HORIZON, 1);
current_outer_cmd = u_outer_prev;

phi_max = deg2rad(MAX_ROLL_CMD_DEG);
theta_max = deg2rad(MAX_PITCH_CMD_DEG);
psi_max = deg2rad(MAX_YAW_CMD_DEG);

LB_STAGE = [MIN_TOTAL_THRUST; -phi_max; -theta_max; -psi_max];
UB_STAGE = [MAX_TOTAL_THRUST;  phi_max;  theta_max;  psi_max];

LB = repmat(LB_STAGE, NMPC_HORIZON, 1);
UB = repmat(UB_STAGE, NMPC_HORIZON, 1);

%% ============================================================
%  MAIN SIMULATION LOOP
% =============================================================

for i = 1:N_SIM-1

    t = (i-1)*DT_INNER;
    TIME(i) = t;

    %% --------------------------------------------------------
    %  OUTER LOOP NMPC UPDATE
    % ---------------------------------------------------------

    if mod(i-1, INNER_STEPS_PER_OUTER) == 0

        REF_NMPC = zeros(9, NMPC_HORIZON+1);

        for k = 1:NMPC_HORIZON+1
            tk = t + (k-1)*NMPC_DT;

            ref12 = smooth_circle_reference( ...
                tk, ...
                CIRCLE_RADIUS, ...
                CIRCLE_ALTITUDE, ...
                CIRCLE_OMEGA, ...
                TAKEOFF_TIME, ...
                CIRCLE_START_TIME, ...
                CIRCLE_RAMP_TIME);

            REF_NMPC(:,k) = ref12(1:9);
        end

        U0 = shift_warm_start(u_outer_sequence_prev, NMPC_HORIZON);

        COSTFUN = @(U) nmpc_cost_outer( ...
            U, x, REF_NMPC, u_outer_prev, ...
            MASS, GRAVITY, NMPC_DT, NMPC_HORIZON, ...
            Q_POS, Q_VEL, Q_ATT_CMD, R_INPUT, R_DINPUT);

        solve_timer = tic;

        try
            [USTAR,~,exitflag] = fmincon(COSTFUN, U0, [], [], [], [], LB, UB, [], options);
        catch
            warning('NMPC failed at t = %.3f s. Using previous command.', t);
            USTAR = U0;
            exitflag = -999;
        end

        solve_time = toc(solve_timer);

        NMPC_SOLVE_TIME(end+1) = solve_time;
        NMPC_EXITFLAGS(end+1) = exitflag;

        if solve_time > DT_OUTER
            fprintf('WARNING: NMPC missed real time at t = %.2f s. Solve %.4f s, budget %.4f s\n', ...
                t, solve_time, DT_OUTER);
        end

        current_outer_cmd = USTAR(1:4);
        u_outer_sequence_prev = USTAR;
        u_outer_prev = current_outer_cmd;
    end

    %% --------------------------------------------------------
    %  INNER LOOP LQR ATTITUDE CONTROL
    % ---------------------------------------------------------

    T_cmd = current_outer_cmd(1);
    phi_cmd = current_outer_cmd(2);
    theta_cmd = current_outer_cmd(3);
    psi_cmd = current_outer_cmd(4);

    attitude_state = x([7 8 9 10 11 12]);
    attitude_ref = [phi_cmd; theta_cmd; psi_cmd; 0; 0; 0];

    attitude_error = attitude_state - attitude_ref;
    attitude_error(3) = wrapToPiLocal(attitude_error(3));

    torque_cmd = -K_LQR * attitude_error;

    torque_cmd(1) = saturate(torque_cmd(1), -MAX_TORQUE_ROLL, MAX_TORQUE_ROLL);
    torque_cmd(2) = saturate(torque_cmd(2), -MAX_TORQUE_PITCH, MAX_TORQUE_PITCH);
    torque_cmd(3) = saturate(torque_cmd(3), -MAX_TORQUE_YAW, MAX_TORQUE_YAW);

    u_actual = [T_cmd; torque_cmd(:)];

    %% --------------------------------------------------------
    %  NONLINEAR EULER-ANGLE QUADROTOR DYNAMICS
    % ---------------------------------------------------------

    x = rk4_step(@(xx) quad_euler_dynamics(xx, u_actual, MASS, GRAVITY, INERTIA), x, DT_INNER);

    x(9) = wrapToPiLocal(x(9));

    %% --------------------------------------------------------
    %  LOG DATA
    % ---------------------------------------------------------

    XHIST(:,i+1) = x;
    UHIST(:,i) = u_actual;
    CMDHIST(:,i) = current_outer_cmd;

    REFHIST(:,i) = smooth_circle_reference( ...
        t, ...
        CIRCLE_RADIUS, ...
        CIRCLE_ALTITUDE, ...
        CIRCLE_OMEGA, ...
        TAKEOFF_TIME, ...
        CIRCLE_START_TIME, ...
        CIRCLE_RAMP_TIME);
end

TIME(end) = SIM_TIME;

REFHIST(:,end) = smooth_circle_reference( ...
    SIM_TIME, ...
    CIRCLE_RADIUS, ...
    CIRCLE_ALTITUDE, ...
    CIRCLE_OMEGA, ...
    TAKEOFF_TIME, ...
    CIRCLE_START_TIME, ...
    CIRCLE_RAMP_TIME);

UHIST(:,end) = UHIST(:,end-1);
CMDHIST(:,end) = CMDHIST(:,end-1);

TOTAL_WALL_TIME = toc(TOTAL_SIM_TIMER);

%% ============================================================
%  PERFORMANCE METRICS
% =============================================================

pos_error = vecnorm(XHIST(1:3,:) - REFHIST(1:3,:), 2, 1);
xy_error = vecnorm(XHIST(1:2,:) - REFHIST(1:2,:), 2, 1);
z_error = abs(XHIST(3,:) - REFHIST(3,:));

SIM_REALTIME_RATIO = TOTAL_WALL_TIME / SIM_TIME;

fprintf('\n================ TRACKING REPORT ================\n');
fprintf('Final position error:        %.4f m\n', pos_error(end));
fprintf('Mean position error:         %.4f m\n', mean(pos_error));
fprintf('Max position error:          %.4f m\n', max(pos_error));
fprintf('Final XY error:              %.4f m\n', xy_error(end));
fprintf('Mean XY error:               %.4f m\n', mean(xy_error));
fprintf('Final Z error:               %.4f m\n', z_error(end));
fprintf('Mean Z error:                %.4f m\n', mean(z_error));
fprintf('Max Z error:                 %.4f m\n', max(z_error));
fprintf('=================================================\n');

fprintf('\n================ NMPC TIMING REPORT ================\n');
fprintf('Simulated time:              %.3f s\n', SIM_TIME);
fprintf('Wall-clock compute time:     %.3f s\n', TOTAL_WALL_TIME);
fprintf('Real-time ratio:             %.3f x real time\n', SIM_REALTIME_RATIO);
fprintf('Outer-loop period budget:    %.4f s\n', DT_OUTER);
fprintf('Mean NMPC solve time:        %.4f s\n', mean(NMPC_SOLVE_TIME));
fprintf('Median NMPC solve time:      %.4f s\n', median(NMPC_SOLVE_TIME));
fprintf('Max NMPC solve time:         %.4f s\n', max(NMPC_SOLVE_TIME));
fprintf('Min NMPC solve time:         %.4f s\n', min(NMPC_SOLVE_TIME));
fprintf('Percent real-time misses:    %.2f %%\n', 100*mean(NMPC_SOLVE_TIME > DT_OUTER));
fprintf('Number of NMPC solves:       %d\n', length(NMPC_SOLVE_TIME));
fprintf('====================================================\n');

%% ============================================================
%  PLOTS
% =============================================================

figure;
plot3(REFHIST(1,:), REFHIST(2,:), REFHIST(3,:), 'k--', 'LineWidth', 2); hold on;
plot3(XHIST(1,:), XHIST(2,:), XHIST(3,:), 'b', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('3D Trajectory: NMPC Outer Loop + LQR Inner Loop');
legend('Reference','Actual');

figure;
subplot(3,1,1);
plot(TIME, REFHIST(1,:), 'k--', TIME, XHIST(1,:), 'b', 'LineWidth', 1.2);
grid on; ylabel('X [m]'); legend('ref','actual');

subplot(3,1,2);
plot(TIME, REFHIST(2,:), 'k--', TIME, XHIST(2,:), 'b', 'LineWidth', 1.2);
grid on; ylabel('Y [m]'); legend('ref','actual');

subplot(3,1,3);
plot(TIME, REFHIST(3,:), 'k--', TIME, XHIST(3,:), 'b', 'LineWidth', 1.2);
grid on; ylabel('Z [m]'); xlabel('Time [s]'); legend('ref','actual');

figure;
subplot(3,1,1);
plot(TIME, rad2deg(CMDHIST(2,:)), 'r--', TIME, rad2deg(XHIST(7,:)), 'b', 'LineWidth', 1.2);
grid on; ylabel('\phi [deg]'); legend('cmd','actual');

subplot(3,1,2);
plot(TIME, rad2deg(CMDHIST(3,:)), 'r--', TIME, rad2deg(XHIST(8,:)), 'b', 'LineWidth', 1.2);
grid on; ylabel('\theta [deg]'); legend('cmd','actual');

subplot(3,1,3);
plot(TIME, rad2deg(CMDHIST(4,:)), 'r--', TIME, rad2deg(XHIST(9,:)), 'b', 'LineWidth', 1.2);
grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); legend('cmd','actual');

figure;
subplot(4,1,1);
plot(TIME, UHIST(1,:), 'LineWidth', 1.2);
grid on; ylabel('T [N]');
title('Control Inputs');

subplot(4,1,2);
plot(TIME, UHIST(2,:), 'LineWidth', 1.2);
grid on; ylabel('\tau_\phi [N*m]');

subplot(4,1,3);
plot(TIME, UHIST(3,:), 'LineWidth', 1.2);
grid on; ylabel('\tau_\theta [N*m]');

subplot(4,1,4);
plot(TIME, UHIST(4,:), 'LineWidth', 1.2);
grid on; ylabel('\tau_\psi [N*m]');
xlabel('Time [s]');

figure;
plot(TIME, pos_error, 'LineWidth', 1.5); hold on;
plot(TIME, xy_error, 'LineWidth', 1.5);
plot(TIME, z_error, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Error [m]');
title('Tracking Error');
legend('3D position error','XY error','Z error');

figure;
plot(NMPC_SOLVE_TIME, 'LineWidth', 1.5); hold on;
yline(DT_OUTER, 'r--', 'Real-time budget');
grid on;
xlabel('NMPC Solve Number');
ylabel('Solve Time [s]');
title('NMPC Computation Time Per Outer-Loop Step');
legend('Solve time','Budget');

%% ============================================================
%  LOCAL FUNCTIONS
% =============================================================

function xdot = quad_euler_dynamics(x, u, m, g, J)

    vx = x(4);
    vy = x(5);
    vz = x(6);

    phi = x(7);
    theta = x(8);
    psi = x(9);

    p = x(10);
    q = x(11);
    r = x(12);

    T = u(1);
    tau = u(2:4);

    R = rot_zyx(phi, theta, psi);

    F_world = R * [0; 0; T];

    accel = F_world/m - [0; 0; g];

    omega = [p; q; r];
    omega_dot = J \ (tau - cross(omega, J*omega));

    euler_dot = euler_rates_from_body_rates(phi, theta, p, q, r);

    xdot = zeros(12,1);

    xdot(1) = vx;
    xdot(2) = vy;
    xdot(3) = vz;

    xdot(4:6) = accel;

    xdot(7:9) = euler_dot;

    xdot(10:12) = omega_dot;
end

function R = rot_zyx(phi, theta, psi)

    cphi = cos(phi);   sphi = sin(phi);
    cth = cos(theta);  sth = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    Rz = [cpsi -spsi 0;
          spsi  cpsi 0;
          0     0    1];

    Ry = [cth 0 sth;
          0   1 0;
         -sth 0 cth];

    Rx = [1 0     0;
          0 cphi -sphi;
          0 sphi  cphi];

    R = Rz * Ry * Rx;
end

function eulerdot = euler_rates_from_body_rates(phi, theta, p, q, r)

    ctheta = cos(theta);

    if abs(ctheta) < 1e-4
        ctheta = sign(ctheta + 1e-12)*1e-4;
    end

    W = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/ctheta,      cos(phi)/ctheta];

    eulerdot = W * [p; q; r];
end

function xnext = rk4_step(f, x, dt)

    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);

    xnext = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

function Jcost = nmpc_cost_outer(U, x0, REF, u_prev, m, g, dt, N, Qp, Qv, Qa, Rinput, Rdinput)

    xp = x0(1:9);

    Jcost = 0;
    last_u = u_prev;

    hover_input = [m*g; 0; 0; 0];

    for k = 1:N

        uk = U(4*(k-1)+1 : 4*k);

        xref = REF(:,k);

        pos_err = xp(1:3) - xref(1:3);
        vel_err = xp(4:6) - xref(4:6);

        att_err = xp(7:9) - xref(7:9);
        att_err(3) = wrapToPiLocal(att_err(3));

        du = uk - last_u;

        input_err = uk - hover_input;

        Jcost = Jcost ...
            + pos_err' * Qp * pos_err ...
            + vel_err' * Qv * vel_err ...
            + att_err' * Qa * att_err ...
            + input_err' * Rinput * input_err ...
            + du' * Rdinput * du;

        xp = outer_prediction_step(xp, uk, m, g, dt);

        last_u = uk;
    end

    xrefN = REF(:,N+1);

    pos_errN = xp(1:3) - xrefN(1:3);
    vel_errN = xp(4:6) - xrefN(4:6);

    Jcost = Jcost ...
        + 3.0*(pos_errN' * Qp * pos_errN) ...
        + 2.0*(vel_errN' * Qv * vel_errN);
end

function xp_next = outer_prediction_step(xp, u, m, g, dt)

    T = u(1);
    phi_cmd = u(2);
    theta_cmd = u(3);
    psi_cmd = u(4);

    phi = xp(7);
    theta = xp(8);
    psi = xp(9);

    % Approximate inner-loop closed-loop attitude response.
    % Smaller value means faster attitude tracking.
    ATT_TAU = 0.10;

    phi_dot = (phi_cmd - phi)/ATT_TAU;
    theta_dot = (theta_cmd - theta)/ATT_TAU;

    psi_err = wrapToPiLocal(psi_cmd - psi);
    psi_dot = psi_err/ATT_TAU;

    R = rot_zyx(phi, theta, psi);
    accel = R * [0; 0; T]/m - [0; 0; g];

    xp_next = xp;

    xp_next(1:3) = xp(1:3) + dt*xp(4:6);
    xp_next(4:6) = xp(4:6) + dt*accel;

    xp_next(7) = xp(7) + dt*phi_dot;
    xp_next(8) = xp(8) + dt*theta_dot;
    xp_next(9) = wrapToPiLocal(xp(9) + dt*psi_dot);
end

function ref = smooth_circle_reference(t, radius, altitude, omega, takeoff_time, circle_start_time, ramp_time)

    ref = zeros(12,1);

    smoothstep = @(s) max(0,min(1,s)).^2 .* (3 - 2*max(0,min(1,s)));

    % Smooth altitude
    sz = smoothstep(t/takeoff_time);
    Z = altitude * sz;

    if t < takeoff_time
        s = max(0,min(1,t/takeoff_time));
        dsz_dt = (6*s - 6*s^2)/takeoff_time;
        VZ = altitude * dsz_dt;
    else
        VZ = 0;
    end

    % Smooth circle radius
    if t < circle_start_time
        r_now = 0;
        r_dot = 0;
        tau = 0;
    else
        tau = t - circle_start_time;
        sr = smoothstep(tau/ramp_time);
        r_now = radius * sr;

        if tau < ramp_time
            s = tau/ramp_time;
            dsdt = (6*s - 6*s^2)/ramp_time;
            r_dot = radius * dsdt;
        else
            r_dot = 0;
        end
    end

    angle = omega * max(0, t - circle_start_time);

    X = r_now*cos(angle);
    Y = r_now*sin(angle);

    VX = r_dot*cos(angle) - r_now*omega*sin(angle);
    VY = r_dot*sin(angle) + r_now*omega*cos(angle);

    PSI = wrapToPiLocal(angle + pi/2);

    ref(1) = X;
    ref(2) = Y;
    ref(3) = Z;

    ref(4) = VX;
    ref(5) = VY;
    ref(6) = VZ;

    ref(7) = 0;
    ref(8) = 0;
    ref(9) = PSI;
end

function U0 = shift_warm_start(Uprev, N)

    Umat = reshape(Uprev, 4, N);
    Umat_shift = [Umat(:,2:end), Umat(:,end)];
    U0 = Umat_shift(:);
end

function y = saturate(x, xmin, xmax)

    y = min(max(x, xmin), xmax);
end

function a = wrapToPiLocal(a)

    a = mod(a + pi, 2*pi) - pi;
end