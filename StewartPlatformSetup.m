%% Simulation of a Stewart platform

%  Instructions
%  ------------
%
%  This file is the setup file to simulate a Stewart platform, which
%  is designed for project using in IRP, TU Braunschweig. The thesis 
%  "Design und Dynamikanalyse der Stewart-Plattform" by Yunqi Gu provides
%  the theoretical background of this simulation. The detail of the
%  simulation process is written in thesis "Simulation einer Stewart-
%  Plattform mit MATLAB" by Chijiang Duan.
%  The files needed in this simulation
%  
%     Base.step
%     Control_loop.slx
%     holder.step
%     inverse_kinematic.m
%     leg_d.step
%     leg_u.step
%     platform.step
%     StewartPlatformSetup.m
%
%  All datas and files can be downloaded from
%  https://github.com/chijiang/Simulation-StewartPlatform.git

%% Initialization
clc; clear; close all; format;
%% Define basic angular unit conversions and axes
deg2rad = pi / 180;
x_axis = [1 0 0];
y_axis = [0 1 0];
z_axis = [0 0 1];

%% Connection points on the base and top platform w.r.t the world frame
pos_base = zeros(6, 3);
pos_top = zeros(6, 3);
radius_b = 0.25145;
radius_t = 0.2169;
alpha_b = (60 - 11.47) * deg2rad;
alpha_t = 12.86 * deg2rad;
height = 0.5;

for i = 1:3 
    % base points
    angle_m_b = (2 * pi/3) * (i-1) - alpha_b - pi/2;
    angle_p_b = (2 * pi/3) * (i-1) + alpha_b - pi/2;
    pos_base(2 * i - 1,:) = [radius_b * cos(angle_m_b), ...
        radius_b * sin(angle_m_b), 0.05];
    pos_base(2 * i, :) = [radius_b * cos(angle_p_b), ...
        radius_b* sin(angle_p_b), 0.05];
    % top points
    angle_m_t = (2 * pi / 3)* (i-1) - alpha_t - pi/2; 
    angle_p_t = (2 * pi / 3)* (i-1) + alpha_t - pi/2;
    pos_top(2 * i - 1, :) = [radius_t * cos(angle_m_t), ...
        radius_t * sin(angle_m_t), height + 0.05];
    pos_top(2 * i,:) = [radius_t * cos(angle_p_t), ...
        radius_t * sin(angle_p_t), height + 0.05];
end

%% Leg lengths and unit vectors
legs = pos_top - pos_base;
leg_length = zeros(1, 6);
leg_vectors = zeros(6, 3);
for i = 1:6
    leg_length(i) = norm(legs(i, :));
    leg_vectors(i, :) = legs(i, :) / leg_length(i);
end 

%% Important translations, density and center of masses (Not all needed for this version of simulation)
% %% revolute and cylindrical axes for joints
% rev1 = zeros(6,3);
% rev2 = zeros(6,3);
% rev3 = zeros(6,3);
% rev4 = zeros(6,3);
% cyl1 = zeros(6,3);
% for i = 1:6
%     % spherical joints
%     rev1(i,:) = cross(leg_vectors(i,:), z_axis);
%     rev1(i,:) = rev1(i,:) / norm(rev1(i,:));
%     rev2(i,:) = - cross(rev1(i,:), leg_vectors(i,:));
%     rev2(i,:) = rev2(i,:) / norm(rev2(i,:));
%     % cylinder joints
%     cyl1(i,:) = leg_vectors(i,:);
%     % universal joints
%     rev3(i,:) = rev1(i,:);
%     rev4(i,:) = rev2(i,:);
% end
% % The Translation from platform to upper legs
top2leg = pos_top' - (height + 0.05 + 0.02) * [zeros(2,6);ones(1,6)];
% lower_leg = struct('origin', [0 0 0], 'rotation', eye(3), 'end_point', [0 0 0]);
% upper_leg = struct('origin', [0 0 0], 'rotation', eye(3), 'end_point', [0 0 0]);
% for i = 1:6
%     lower_leg(i).origin = pos_base(i,:) + (3/8)*legs(i,:);
%     lower_leg(i).end_point = pos_base(i,:) + (3/4)*legs(i,:);
%     lower_leg(i).rotation = [rev1(i,:)', rev2(i,:)', cyl1(i,:)'];
%     upper_leg(i).origin = pos_base(i,:) + (1-3/8)*legs(i,:);
%     upper_leg(i).end_point = pos_base(i,:) + (1/4)*legs(i,:); 
%     upper_leg(i).rotation = [rev1(i,:)', rev2(i,:)', cyl1(i,:)'];
% end

% %% Inertia and mass of the top plate, bottom plate, and the legs
% top_thickness = 0.04;
% base_thickness = 0.05;
% inner_radius = 0.02; % legs
% outer_radius = 0.035; % legs
density = 2712; % Kg/m^3 --- Aluminum    2712;
% [lower_leg_mass, lower_leg_inertia] = inertiaCylinder(density, 0.350,outer_radius, inner_radius);
% [upper_leg_mass, upper_leg_inertia] = inertiaCylinder(density, 0.350,inner_radius, 0);
% [top_mass, top_inertia] = inertiaCylinder(density, top_thickness, radius_t, 0.15);
% [base_mass, base_inertia] = inertiaCylinder(density, base_thickness, radius_b, 0.175);

%% PID controller gains
% Kp = 2e6; Ki = 1e4; Kd = 4.5e4;
Kp = 3e6; Ki = 0; Kd = 2.5e4;

%% Motion control
% Amplitude of sine signals
xang_a = 0; yang_a = 20*deg2rad; zang_a = 0;
xpos_a = 0.15; ypos_a = 0; zpos_a = 0;
% Frequency of sine signals
xang_f = 2; yang_f = 2; zang_f = 0;
xpos_f = 2; ypos_f = 0; zpos_f = 2;
% Phase of sine signals
xang_p = 0; yang_p = 0; zang_p = 0;
xpos_p = 0; ypos_p = pi/2; zpos_p = 0;

%% Initial posture
alpha = xang_a*sin(0 + xang_p); 
beta = yang_a*sin(0 + yang_p); 
gamma = zang_a*sin(0 + zang_p);

top_position = [xpos_a*sin(0 + xpos_p); 
    ypos_a*sin(0 + ypos_p); 
    zpos_a*sin(0 + zpos_p) + height+0.07];
speed_top = zeros(6, 1);
[length, speed] = inverse_kinematic(alpha, beta, gamma, top_position, speed_top, pos_base', top2leg);
length = length - leg_length;

%% Perform the simulation
% i = 1;
% for Ki = [0, 1000, 1e4]
%     sim('Control_loop.slx') 
%     subplot(3,2,i)
%     plot(error, 'LineWidth', 2)
%     title(['Error by K_i = ', num2str(Ki)])
%     legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
%     ylabel('Error [m]')
%     xlabel('Time [s]')
%     set(gca,'FontSize',15)
%     subplot(3,2,i+1)
%     plot(force, 'LineWidth', 2)
%     % axis([0,5,50,120]);
%     title(['Force by K_i = ', num2str(Ki)])
%     ylabel('Force [N]')
%     xlabel('Time [s]')
%     legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
%     set(gca,'FontSize',15)
%     i = i + 2;
% end
% --------------------------------------------------
% Pendulum
% figure;
% sim('Control_loop.slx')
% subplot(3,2,1)
% plot(Platform_pos, 'LineWidth', 2)
% legend('x','y','z')
% title('Platform position for pendulum')
% xlabel('Time [s]')
% ylabel('Position [m]')
% set(gca,'FontSize',15)
% subplot(3,2,3)
% plot(error, 'LineWidth', 2)
% legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
% title('Error for pendulum')
% xlabel('Time [s]')
% ylabel('Error [m]')
% set(gca,'FontSize',15)
% subplot(3,2,5)
% plot(force, 'LineWidth', 2)
% legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
% title('Leg force for pendulum')
% xlabel('Time [s]')
% ylabel('Force [N]')
% set(gca,'FontSize',15)
% % -------------------------------------------------
% % Lifting movement
% yang_a = 0; xpos_a = 0; zpos_a = 0.2;
% zpos_f = 2; zpos_p = 0;
% alpha = xang_a*sin(0 + xang_p); 
% beta = yang_a*sin(0 + yang_p); 
% gamma = zang_a*sin(0 + zang_p);
% 
% top_position = [xpos_a*sin(0 + xpos_p); 
%     ypos_a*sin(0 + ypos_p); 
%     zpos_a*sin(0 + zpos_p) + height+0.07];
% speed_top = zeros(6, 1);
% [length, speed] = inverse_kinematic(alpha, beta, gamma, top_position, speed_top, pos_base', top2leg);
% length = length - leg_length;
% sim('Control_loop.slx')
% subplot(3,2,2)
% plot(Platform_pos, 'LineWidth', 2)
% legend('x','y','z')
% title('Platform position for lifting movement')
% xlabel('Time [s]')
% ylabel('Position [m]')
% set(gca,'FontSize',15)
% subplot(3,2,4)
% plot(error, 'LineWidth', 2)
% legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
% title('Error for lifting movement')
% xlabel('Time [s]')
% ylabel('Error [m]')
% set(gca,'FontSize',15)
% subplot(3,2,6)
% plot(force, 'LineWidth', 2)
% legend('Leg 1','Leg 2','Leg 3','Leg 4','Leg 5','Leg 6')
% title('Leg force for lifting movement')
% xlabel('Time [s]')
% ylabel('Force [N]')
% set(gca,'FontSize',15)

% saveas(gcf, '~/Documents/StewartPlatform/variFig.png')

sim('Control_loop.slx')