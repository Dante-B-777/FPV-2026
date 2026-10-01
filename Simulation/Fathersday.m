clear; clc; close all;

%% ================= USER PARAMETERS =================

MASS = 0.50;                 % kg
GRAVITY = 9.81;              % m/s^2

ARM_LENGTH = 0.12;           % m
IXX = 0.0023;                % kg*m^2
IYY = 0.0023;                % kg*m^2
IZZ = 0.0040;                % kg*m^2
INERTIA = diag([IXX IYY IZZ]);

MAX_THRUST_TOTAL = 2.5 * MASS * GRAVITY;
MIN_THRUST_TOTAL = 0.05 * MASS * GRAVITY;

MAX_ROLL = deg2rad(25);
MAX_PITCH = deg2rad(25);
MAX_YAW_RATE = deg2rad(80);

MAX_XY_SPEED = 1.5;          % m/s
MAX_Z_SPEED = 1.0;           % m/s

SIM_DT = 0.01;               % simulation step, seconds
DRAW_EVERY = 2;              % draw every N simulation steps

WORLD_X_LIMIT = [-2 8];
WORLD_Y_LIMIT = [-4 4];
WORLD_Z_LIMIT = [0 4];

DRONE_ARM_DRAW = 0.25;

HOOP_RADIUS = 0.65;          % m
HOOP_TUBE_RADIUS = 0.035;    % visual tube radius only
HOOP_PASS_RADIUS = 0.50;     % drone must be inside this radius
HOOP_PLANE_TOL = 0.20;       % distance through hoop plane

HOOPS = [ ...
    2.0,  0.0, 1.2;
    4.0,  1.0, 1.5;
    6.0, -0.8, 1.1];

%% ================= CONTROLLER GAINS =================

KP_POS_XY = 1.4;
KD_POS_XY = 1.8;

KP_Z = 8.0;
KD_Z = 5.0;

KP_ANGLE = [12; 12; 6];
KD_RATE  = [3.0; 3.0; 1.8];

%% ================= INITIAL STATE =================
% State:
% [x y z vx vy vz phi theta psi p q r]'

x = zeros(12,1);
x(3) = 1.0;

desired_z = 1.0;
desired_yaw = 0.0;

passed_hoops = false(3,1);
current_hoop = 1;

sim_time = 0;
step_count = 0;

%% ================= FIGURE SETUP =================

fig = figure( ...
    'Name','Keyboard Drone Hoop Simulation', ...
    'Color','w', ...
    'NumberTitle','off', ...
    'KeyPressFcn',@keyDown, ...
    'KeyReleaseFcn',@keyUp);

setappdata(fig,'keys',struct());

ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
xlim(ax,WORLD_X_LIMIT);
ylim(ax,WORLD_Y_LIMIT);
zlim(ax,WORLD_Z_LIMIT);
xlabel(ax,'X forward [m]');
ylabel(ax,'Y left/right [m]');
zlabel(ax,'Z up [m]');
view(ax,35,22);

title(ax,'Click the figure, then use keyboard controls');

% Ground
[Xg,Yg] = meshgrid(WORLD_X_LIMIT(1):1:WORLD_X_LIMIT(2), WORLD_Y_LIMIT(1):1:WORLD_Y_LIMIT(2));
Zg = zeros(size(Xg));
surf(ax,Xg,Yg,Zg,'FaceAlpha',0.08,'EdgeAlpha',0.15);

% Draw hoops
hoop_handles = gobjects(3,1);
for i = 1:3
    hoop_handles(i) = drawHoop(ax,HOOPS(i,:),HOOP_RADIUS,HOOP_TUBE_RADIUS,i);
end

% Drone graphics
drone_body = plot3(ax,0,0,1,'ko','MarkerFaceColor','k','MarkerSize',7);
arm1 = plot3(ax,[0 0],[0 0],[0 0],'b-','LineWidth',3);
arm2 = plot3(ax,[0 0],[0 0],[0 0],'r-','LineWidth',3);
motor_pts = plot3(ax,zeros(1,4),zeros(1,4),zeros(1,4),'ko','MarkerFaceColor','k','MarkerSize',5);

trail = animatedline(ax,'LineWidth',1.5);

info_text = text(ax,WORLD_X_LIMIT(1)+0.2,WORLD_Y_LIMIT(1)+0.2,WORLD_Z_LIMIT(2)-0.3, ...
    '', ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'Color','k');

final_text = text(ax,mean(WORLD_X_LIMIT),0,WORLD_Z_LIMIT(2)-0.7, ...
    '', ...
    'FontSize',26, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center', ...
    'Color',[0.85 0.1 0.1]);

control_text = text(ax,WORLD_X_LIMIT(1)+0.2,WORLD_Y_LIMIT(2)-0.3,WORLD_Z_LIMIT(2)-0.3, ...
    sprintf(['Controls:\n', ...
    'W/S: forward/back\n', ...
    'A/D: left/right\n', ...
    'Q/E: yaw left/right\n', ...
    'Up/Down: altitude\n', ...
    'Space: hover\n', ...
    'R: reset\n', ...
    'Esc: stop']), ...
    'FontSize',9, ...
    'Color',[0.1 0.1 0.1]);

disp('Simulation started.');
disp('Click the figure window before pressing keys.');
disp('Controls: W/S forward/back, A/D left/right, Q/E yaw, Up/Down altitude, Space hover, R reset, Esc stop.');

%% ================= MAIN SIMULATION LOOP =================

running = true;

while running && ishandle(fig)

    step_count = step_count + 1;
    sim_time = sim_time + SIM_DT;

    keys = getappdata(fig,'keys');

    if isKeyDown(keys,'escape')
        running = false;
        break;
    end

    if isKeyDown(keys,'r')
        x = zeros(12,1);
        x(3) = 1.0;
        desired_z = 1.0;
        desired_yaw = 0.0;
        passed_hoops(:) = false;
        current_hoop = 1;
        clearpoints(trail);
        set(final_text,'String','');
        for i = 1:3
            set(hoop_handles(i),'Color',[0.1 0.4 1.0]);
        end
        setappdata(fig,'keys',struct());
    end

    if isKeyDown(keys,'space')
        desired_body_vx = 0;
        desired_body_vy = 0;
        desired_z_rate = 0;
        desired_yaw_rate = 0;
    else
        desired_body_vx = 0;
        desired_body_vy = 0;
        desired_z_rate = 0;
        desired_yaw_rate = 0;

        if isKeyDown(keys,'w')
            desired_body_vx = desired_body_vx + MAX_XY_SPEED;
        end
        if isKeyDown(keys,'s')
            desired_body_vx = desired_body_vx - MAX_XY_SPEED;
        end
        if isKeyDown(keys,'a')
            desired_body_vy = desired_body_vy + MAX_XY_SPEED;
        end
        if isKeyDown(keys,'d')
            desired_body_vy = desired_body_vy - MAX_XY_SPEED;
        end
        if isKeyDown(keys,'uparrow')
            desired_z_rate = desired_z_rate + MAX_Z_SPEED;
        end
        if isKeyDown(keys,'downarrow')
            desired_z_rate = desired_z_rate - MAX_Z_SPEED;
        end
        if isKeyDown(keys,'q')
            desired_yaw_rate = desired_yaw_rate + MAX_YAW_RATE;
        end
        if isKeyDown(keys,'e')
            desired_yaw_rate = desired_yaw_rate - MAX_YAW_RATE;
        end
    end

    desired_z = desired_z + desired_z_rate * SIM_DT;
    desired_z = min(max(desired_z,0.4),3.4);

    desired_yaw = desired_yaw + desired_yaw_rate * SIM_DT;
    desired_yaw = wrapToPiLocal(desired_yaw);

    u = controller(x,desired_body_vx,desired_body_vy,desired_z,desired_yaw, ...
        MASS,GRAVITY,INERTIA, ...
        KP_POS_XY,KD_POS_XY,KP_Z,KD_Z,KP_ANGLE,KD_RATE, ...
        MAX_ROLL,MAX_PITCH,MIN_THRUST_TOTAL,MAX_THRUST_TOTAL);

    x = rk4Step(@(xx) droneDynamics(xx,u,MASS,GRAVITY,INERTIA),x,SIM_DT);

    x(3) = max(x(3),0.02);
    if x(3) <= 0.021 && x(6) < 0
        x(6) = 0;
    end

    % Hoop pass check
    pos = x(1:3).';

    if current_hoop <= 3
        hoop_center = HOOPS(current_hoop,:);
        dx_plane = abs(pos(1) - hoop_center(1));
        radial_error = norm(pos(2:3) - hoop_center(2:3));

        if dx_plane < HOOP_PLANE_TOL && radial_error < HOOP_PASS_RADIUS
            passed_hoops(current_hoop) = true;
            set(hoop_handles(current_hoop),'Color',[0.1 0.8 0.1]);
            current_hoop = current_hoop + 1;

            if current_hoop > 3
                set(final_text,'String','happy fathers day!');
            end
        end
    end

    % Draw
    if mod(step_count,DRAW_EVERY) == 0
        R = eulToRotmLocal(x(7),x(8),x(9));
        p0 = x(1:3);

        motor_body = [ ...
             DRONE_ARM_DRAW, 0, 0;
            -DRONE_ARM_DRAW, 0, 0;
             0, DRONE_ARM_DRAW, 0;
             0,-DRONE_ARM_DRAW, 0]';

        motor_world = R * motor_body + p0;

        set(drone_body,'XData',p0(1),'YData',p0(2),'ZData',p0(3));

        set(arm1, ...
            'XData',[motor_world(1,1), motor_world(1,2)], ...
            'YData',[motor_world(2,1), motor_world(2,2)], ...
            'ZData',[motor_world(3,1), motor_world(3,2)]);

        set(arm2, ...
            'XData',[motor_world(1,3), motor_world(1,4)], ...
            'YData',[motor_world(2,3), motor_world(2,4)], ...
            'ZData',[motor_world(3,3), motor_world(3,4)]);

        set(motor_pts, ...
            'XData',motor_world(1,:), ...
            'YData',motor_world(2,:), ...
            'ZData',motor_world(3,:));

        addpoints(trail,p0(1),p0(2),p0(3));

        if current_hoop <= 3
            target_string = sprintf('Current target: Hoop %d of 3',current_hoop);
        else
            target_string = 'All hoops complete!';
        end

        info_string = sprintf(['Time: %.2f s\n', ...
            'Position: x %.2f, y %.2f, z %.2f\n', ...
            'Passed hoops: %d / 3\n', ...
            '%s'], ...
            sim_time,p0(1),p0(2),p0(3),sum(passed_hoops),target_string);

        set(info_text,'String',info_string);

        drawnow limitrate;
    end
end

disp('Simulation stopped.');

%% ================= LOCAL FUNCTIONS =================

function u = controller(x,desired_body_vx,desired_body_vy,desired_z,desired_yaw, ...
    m,g,J, ...
    kp_xy,kd_xy,kp_z,kd_z,kp_ang,kd_rate, ...
    max_roll,max_pitch,min_T,max_T)

    pos = x(1:3);
    vel = x(4:6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    omega = x(10:12);

    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
          0         0        1];

    desired_world_vel = Rz * [desired_body_vx; desired_body_vy; 0];

    ax_cmd = kp_xy * (desired_world_vel(1) - vel(1));
    ay_cmd = kp_xy * (desired_world_vel(2) - vel(2));

    z_error = desired_z - pos(3);
    az_cmd = kp_z * z_error + kd_z * (0 - vel(3));

    T = m * (g + az_cmd);
    T = min(max(T,min_T),max_T);

    desired_phi = (1/g) * (ax_cmd * sin(psi) - ay_cmd * cos(psi));
    desired_theta = (1/g) * (ax_cmd * cos(psi) + ay_cmd * sin(psi));

    desired_phi = min(max(desired_phi,-max_roll),max_roll);
    desired_theta = min(max(desired_theta,-max_pitch),max_pitch);

    yaw_error = wrapToPiLocal(desired_yaw - psi);

    desired_angles = [desired_phi; desired_theta; psi + yaw_error];
    current_angles = [phi; theta; psi];

    angle_error = desired_angles - current_angles;
    angle_error(3) = wrapToPiLocal(angle_error(3));

    desired_omega = [0; 0; 0];

    tau = kp_ang .* angle_error + kd_rate .* (desired_omega - omega);

    u = [T; tau];
end

function dx = droneDynamics(x,u,m,g,J)

    vel = x(4:6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    omega = x(10:12);

    T = u(1);
    tau = u(2:4);

    R = eulToRotmLocal(phi,theta,psi);

    force_world = R * [0;0;T] + [0;0;-m*g];
    accel = force_world / m;

    E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    eul_dot = E * omega;

    omega_dot = J \ (tau - cross(omega,J*omega));

    dx = zeros(12,1);
    dx(1:3) = vel;
    dx(4:6) = accel;
    dx(7:9) = eul_dot;
    dx(10:12) = omega_dot;
end

function x_next = rk4Step(f,x,dt)

    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);

    x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function R = eulToRotmLocal(phi,theta,psi)

    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
          0         0        1];

    Ry = [cos(theta) 0 sin(theta);
          0          1 0;
         -sin(theta) 0 cos(theta)];

    Rx = [1 0        0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];

    R = Rz * Ry * Rx;
end

function h = drawHoop(ax,center,radius,tube_radius,index_number)

    t = linspace(0,2*pi,160);

    x = center(1) * ones(size(t));
    y = center(2) + radius*cos(t);
    z = center(3) + radius*sin(t);

    h = plot3(ax,x,y,z,'LineWidth',5,'Color',[0.1 0.4 1.0]);

    text(ax,center(1),center(2),center(3)+radius+0.25, ...
        sprintf('Hoop %d',index_number), ...
        'FontSize',11, ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center');

    % Small visual center marker
    plot3(ax,center(1),center(2),center(3),'kx','LineWidth',1.5,'MarkerSize',8);

    % Tube radius is included as a parameter in case you want to extend this
    % into a surface hoop later.
    unused = tube_radius;
end

function keyDown(src,event)

    keys = getappdata(src,'keys');
    keys.(matlab.lang.makeValidName(lower(event.Key))) = true;
    setappdata(src,'keys',keys);
end

function keyUp(src,event)

    keys = getappdata(src,'keys');
    key_name = matlab.lang.makeValidName(lower(event.Key));

    if isfield(keys,key_name)
        keys.(key_name) = false;
    end

    setappdata(src,'keys',keys);
end

function tf = isKeyDown(keys,key_name)

    key_name = matlab.lang.makeValidName(lower(key_name));

    if isfield(keys,key_name)
        tf = keys.(key_name);
    else
        tf = false;
    end
end

function ang = wrapToPiLocal(ang)

    ang = mod(ang + pi,2*pi) - pi;
end