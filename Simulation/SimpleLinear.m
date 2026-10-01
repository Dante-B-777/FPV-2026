%% simple_quadrotor_vertical_takeoff_PI_autoheight.m
% ------------------------------------------------------------
% Simple vertical-only quadrotor takeoff simulation
%
% Goal:
%   Command a 500 g quadrotor to take off to a chosen z height,
%   then hover there.
%
% Model:
%   Fi = kf * omega_i^2
%
%   m*z_ddot = F1 + F2 + F3 + F4 - m*g - c*z_dot
%
% z is positive upward.
% This ignores x/y motion, roll, pitch, yaw, and motor dynamics.

clear; clc; close all;

%% ------------------------------------------------------------
% 1. User-adjustable command
% ------------------------------------------------------------

zRefFinal = 1.0;        % commanded hover height [m]
takeoffRampTime = 1.0;  % seconds to ramp command from 0 to zRefFinal

%% ------------------------------------------------------------
% 2. Physical parameters
% ------------------------------------------------------------

m = 0.500;              % drone mass [kg]
g = 9.81;               % gravity [m/s^2]

Nrotors = 4;            % quadrotor
omegaMax = 850;         % max rotor speed [rad/s], about 8117 RPM

% Choose kf for about 2:1 thrust-to-weight at omegaMax.
% Total max thrust = 2*m*g
% Per-rotor max thrust = 2*m*g/4
kf = (2*m*g/Nrotors)/(omegaMax^2);   % [N/(rad/s)^2]

% Hover thrust and hover rotor speed
Fhover_total = m*g;
Fhover_rotor = Fhover_total/Nrotors;
omegaHover = sqrt(Fhover_rotor/kf);

fprintf('Chosen kf = %.4e N/(rad/s)^2\n', kf);
fprintf('Hover thrust per rotor = %.4f N\n', Fhover_rotor);
fprintf('Hover omega = %.2f rad/s = %.0f RPM\n', ...
    omegaHover, omegaHover*60/(2*pi));

%% ------------------------------------------------------------
% 3. Controller parameters
% ------------------------------------------------------------

% Each rotor has its own PI controller.
% All rotors see the same altitude error in this simplified vertical model.
Kp = [0.25; 0.25; 0.25; 0.25];       % [N/m] per rotor
Ki = [0.0001; 0.0001; 0.0001; 0.0001];   % [N/(m*s)] per rotor

intError = zeros(Nrotors,1);
intLimit = 8.0;

% Simple vertical damping
cDrag = 2.0;            % [N/(m/s)]

%% ------------------------------------------------------------
% 4. Simulation setup
% ------------------------------------------------------------

dt = 0.01;
tFinal = max(15, 8 + 3*zRefFinal);   % automatically longer for taller climbs
t = 0:dt:tFinal;
Nt = length(t);

% State
z = 0;
zdot = 0;

% Storage
zHist = zeros(Nt,1);
zRefHist = zeros(Nt,1);
zdotHist = zeros(Nt,1);
aHist = zeros(Nt,1);
omegaHist = zeros(Nt,Nrotors);
FHist = zeros(Nt,Nrotors);
THist = zeros(Nt,1);
errHist = zeros(Nt,1);

%% ------------------------------------------------------------
% 5. Main simulation loop
% ------------------------------------------------------------

for k = 1:Nt

    tk = t(k);

    % Smooth takeoff command.
    % Instead of instantly commanding zRefFinal, we ramp it up.
    if tk < takeoffRampTime
        zRef = zRefFinal * (tk/takeoffRampTime);
    else
        zRef = zRefFinal;
    end

    % Altitude error
    err = zRef - z;

    % PI integrator for each rotor
    intError = intError + err*dt;
    intError = max(min(intError, intLimit), -intLimit);

    % Rotor thrust commands
    Fcmd = Fhover_rotor + Kp.*err + Ki.*intError;

    % Saturate commanded thrust
    Fcmd = max(Fcmd, 0);
    Fcmd = min(Fcmd, kf*omegaMax^2);

    % Convert thrust command to rotor speed
    omega = sqrt(Fcmd/kf);

    % Actual rotor thrust from model
    F = kf*(omega.^2);

    % Total thrust
    Ttotal = sum(F);

    % Vertical acceleration
    zddot = (Ttotal - m*g - cDrag*zdot)/m;

    % Integrate vertical motion
    zdot = zdot + zddot*dt;
    z = z + zdot*dt;

    % Ground constraint
    if z < 0
        z = 0;
        if zdot < 0
            zdot = 0;
        end
    end

    % Save data
    zHist(k) = z;
    zRefHist(k) = zRef;
    zdotHist(k) = zdot;
    aHist(k) = zddot;
    omegaHist(k,:) = omega.';
    FHist(k,:) = F.';
    THist(k) = Ttotal;
    errHist(k) = err;
end

%% ------------------------------------------------------------
% 6. Auto-scaling values for plots and animation
% ------------------------------------------------------------

zMaxData = max([zHist; zRefHist; zRefFinal]);
zPlotMax = max(1.2, 1.25*zMaxData);

omegaPlotMax = 1.15*max(omegaHist(:));
FPlotMax = 1.15*max(FHist(:));
TPlotMax = 1.15*max(THist);

%% ------------------------------------------------------------
% 7. Print simulation summary
% ------------------------------------------------------------

fprintf('\nSimulation results:\n');
fprintf('Commanded hover height = %.3f m\n', zRefFinal);
fprintf('Final altitude = %.3f m\n', zHist(end));
fprintf('Maximum altitude = %.3f m\n', max(zHist));
fprintf('Maximum rotor speed = %.2f rad/s = %.0f RPM\n', ...
    max(omegaHist(:)), max(omegaHist(:))*60/(2*pi));
fprintf('Final rotor speed = %.2f rad/s = %.0f RPM\n', ...
    omegaHist(end,1), omegaHist(end,1)*60/(2*pi));

%% ------------------------------------------------------------
% 8. Plots with automatic axis scaling
% ------------------------------------------------------------

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
plot(t, zdotHist, 'LineWidth', 1.8);
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
plot(t, omegaHist, 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Rotor Speed \omega_i [rad/s]');
title('Rotor Angular Speeds');
legend('\omega_1', '\omega_2', '\omega_3', '\omega_4');
ylim([0 omegaPlotMax]);

figure;
plot(t, FHist, 'LineWidth', 1.3); hold on;
yline(Fhover_rotor, '--', 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Rotor Thrust F_i [N]');
title('Individual Rotor Thrusts');
legend('F_1', 'F_2', 'F_3', 'F_4', 'Hover thrust per rotor');
ylim([0 FPlotMax]);

figure;
plot(t, THist, 'LineWidth', 1.8); hold on;
yline(m*g, '--', 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Total Thrust [N]');
title('Total Upward Thrust');
legend('Total thrust', 'Weight mg');
ylim([0 TPlotMax]);

%% ------------------------------------------------------------
% 9. 3D animation with automatic height scaling
% ------------------------------------------------------------

armLength = 0.20;       % visual arm length [m]

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

% Actual vertical path
plot3(zeros(size(zHist)), zeros(size(zHist)), zHist, 'k--');

% Ground plane
[Xg, Yg] = meshgrid(-0.6:0.2:0.6, -0.6:0.2:0.6);
Zg = zeros(size(Xg));
mesh(Xg, Yg, Zg, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2);

% Reference height plane
[Xr, Yr] = meshgrid(-0.4:0.2:0.4, -0.4:0.2:0.4);
Zr = zRefFinal*ones(size(Xr));
mesh(Xr, Yr, Zr, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.2);

% Drone graphics objects
hArm1 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
hArm2 = plot3([0 0], [0 0], [0 0], 'LineWidth', 3);
hCenter = plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);

hRotors = gobjects(Nrotors,1);
for i = 1:Nrotors
    hRotors(i) = plot3(0, 0, 0, 'o', 'MarkerSize', 12, 'LineWidth', 2);
end

skip = 8;

for k = 1:skip:Nt

    center = [0, 0, zHist(k)];
    rotorWorld = rotorPosBody + center;

    % Arm from rotor 1 to rotor 3
    set(hArm1, ...
        'XData', [rotorWorld(1,1), rotorWorld(3,1)], ...
        'YData', [rotorWorld(1,2), rotorWorld(3,2)], ...
        'ZData', [rotorWorld(1,3), rotorWorld(3,3)]);

    % Arm from rotor 2 to rotor 4
    set(hArm2, ...
        'XData', [rotorWorld(2,1), rotorWorld(4,1)], ...
        'YData', [rotorWorld(2,2), rotorWorld(4,2)], ...
        'ZData', [rotorWorld(2,3), rotorWorld(4,3)]);

    set(hCenter, ...
        'XData', center(1), ...
        'YData', center(2), ...
        'ZData', center(3));

    for i = 1:Nrotors
        set(hRotors(i), ...
            'XData', rotorWorld(i,1), ...
            'YData', rotorWorld(i,2), ...
            'ZData', rotorWorld(i,3));
    end

    drawnow;
end