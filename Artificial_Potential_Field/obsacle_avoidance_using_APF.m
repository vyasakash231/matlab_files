%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

% call genpath inside of addpath to add all subfolders of Utilities to the search path
addpath(genpath('Utilities'))  

X_g = 0.1; Y_g = -0.2; Z_g = 0.158934;
eta = 0.004; % force multiplication factor

% Define sphere parameters (obstacle)
obstacle = [0.195;-0.12;0.158934]; % center
radius = 0.02;
q_angle = linspace(0, 2*pi, 40); % azimuthal angle
psi = linspace(0, pi, 40); % polar angle

xs = obstacle(1) + radius * sin(psi') * cos(q_angle);
ys = obstacle(2) + radius * sin(psi') * sin(q_angle);
zs = obstacle(3) + radius * cos(psi') * ones(size(q_angle));

[U,V,W] = surfnorm(xs,ys,zs);

% Define Influence_Radius sphere
Influence_radius = 0.015; % if the distance btw manipulator and obstacle is less then this, apply force
q_bound = linspace(0, 2*pi, 30); % azimuthal angle
psi_bound = linspace(0, pi, 30); % polar angle
[Theta, Phi] = meshgrid(q_bound, psi_bound);

x_bound = obstacle(1) + (radius + Influence_radius) * sin(psi_bound') .* cos(q_bound);
y_bound = obstacle(2) + (radius + Influence_radius) * sin(psi_bound') .* sin(q_bound);
z_bound = obstacle(3) + (radius + Influence_radius) * cos(psi_bound') * ones(size(q_bound));

% Define plane vertices
x = linspace(-10, 10, 100); % x coordinates
y = linspace(-10, 10, 100); % y coordinates
[xp, yp] = meshgrid(x, y); % create a grid of x and y coordinates
zp = zeros(size(xp)); % z coordinates are all zeros (plane at z=0)

n = 4; % No of Joint
p = 0.128; q = 0.024;

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = [0,90,-79.38,-45];  %[90,79.38,-79.38,0]; % In degree
Le = 0.126; % End-effector length

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(1,:) = X_cord; 
Y(1,:) = Y_cord;
Z(1,:) = Z_cord;

X_start = X_cord(end);
Y_start = Y_cord(end);
Z_start = Z_cord(end);

theta_range = [-180 180;-117 90;-90 87.5;-103 114.5]; % range of motion of each joint

R = diag([0.1,0.1,0.15,0.15]);
Q = diag([100000,50000,50000]);
Joint_torque_weight = diag([0.75,0.75,0.9,0.9]);

d_t = 0.01;
t = 0:d_t:5; 

plot_avg_D = [];

alp = 0.005; % Smoothing factor, adjust as necessary
smoothed_D_Factor = 0;

del_X = ones(3,1);

for i=1:length(t)
    prev_D_min = 100;
    D_factor = 1000;
    [D, dD_dx, O_m] = Repulsive_Potential_Field(radius, obstacle, X_cord, Y_cord, Z_cord);
    [norm_tau,norm_F] = Repulsive_Torque(D, dD_dx, O_m, Influence_radius, alpha, a, d, theta(i,:), eta);  % if obstacle is far from object apply zero torque

    if any(D <= 2*Influence_radius)
        if min(D) < prev_D_min  % If two obstacle are close enough and arm is within the Influence radius of both, then we need min out of those
            prev_D_min = min(D);
            D_Factor = 100*prev_D_min; % Factor to reduce Joint Velocity Input

%             smoothed_D_Factor = alp * D_Factor + (1 - alp) * smoothed_D_Factor;
        end
%         D_factor = smoothed_D_Factor;
    end

    % Store CLOSEST Point
    closest_point(:,:,i) = O_m;

    % Store Force on Closest Point
    Force(:,:,i) = norm_F;

    final_D = D_factor / (1 + D_factor); % when D_Factor = 1000 (all nearest point are outside the 2*influence_radius), then final_D = 0.999 almost equal to 1 (does not reduce HJB control input)
    plot_avg_D(end+1) = final_D;

    X_c = X(i,n+2);
    Y_c = Y(i,n+2);
    Z_c = Z(i,n+2);
    
    dX = [(X_c - X_g); (Y_c - Y_g); (Z_c - Z_g)];
    del_X(:,i) = dX;  % for plotting

    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));
    
    phi = (Jv' * dX) / norm(Jv' * dX); % (4,1)

    d_theta = - sqrt(dX' * Q * dX) * (inv(R).^0.5 * phi); 

    control_input(:,i+1) = (1-exp(-i*0.01)) * d_theta * (pi/180);

    torque(:,i) = norm_tau;
    
    % Solving using Euler-forward method
    theta_new = theta(i,:) + final_D * d_theta' * d_t + (Joint_torque_weight*norm_tau)'; % In degrees
    
    theta(i+1,:) = theta_new; % In degrees  (for plotting)
    
    % Next Step performed
    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta_new,Le);

    X(i+1,:) = X_cord; % for plotting
    Y(i+1,:) = Y_cord; % for plotting
    Z(i+1,:) = Z_cord; % for plotting
end

figure
for k = 1:i-250
    set(gca,'Projection','perspective')
    plot3(X_start, Y_start, Z_start, 'o', 'LineWidth', 4) % strart state
    hold on
    plot3(X_g, Y_g, Z_g, 'o', 'LineWidth', 4) % goal state
    hold on

    plot3(X(k,1:2),Y(k,1:2),Z(k,1:2),'-','LineWidth',8);
    hold on
    plot3(X(k,2:3),Y(k,2:3),Z(k,2:3),'-','LineWidth',8);
    hold on
    plot3(X(k,3:4),Y(k,3:4),Z(k,3:4),'-','LineWidth',7);
    hold on
    plot3(X(k,4:5),Y(k,4:5),Z(k,4:5),'-','LineWidth',6);
    hold on
    plot3(X(k,5:6),Y(k,5:6),Z(k,5:6),'-','LineWidth',5);
    hold on
    
    plot3(closest_point(1,:,k),closest_point(2,:,k),closest_point(3,:,k),'*','LineWidth',4,'Color','k'); % plot clossest point to obstacle
    hold on  

    plot3(xs, ys, zs, 'k'); % plot a sphere
    hold on
    
    surf(x_bound, y_bound, z_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.2); % plot infulence field
    hold on

    surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5); % plot the plane
    hold on
    plot3(X(1:k,6),Y(1:k,6),Z(1:k,6),'LineWidth',1,'Color','m'); % plot trajectory
    hold on
    view(0,90);
    axis([-0.25 0.25 -0.25 0.25 0 0.5])
    axis square
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    grid on
    hold on
    drawnow
    hold off
    pause(0.1)
end

% figure
% subplot(2,2,1)
% plot(t(1:i),del_X(1:3,:),'--','LineWidth',2)
% hold on 
% plot(t(1:i),rms(del_X(1:3,:)),'LineWidth',2)
% axis square
% xlabel('time (s)')
% ylabel('Position error(mm)')
% legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})','RMS Error')
% grid on
% grid minor
% 
% subplot(2,2,2)
% plot(t(1:i),control_input(:,1:i),'LineWidth',2)
% hold on 
% axis square
% xlabel('time (s)')
% ylabel('Control Input (rad/s)')
% legend('J_{1}','J_{2}','J_{3}','J_{4}')
% grid on
% grid minor
% 
% subplot(2,2,3)
% plot(t(1:i),torque(:,1:i),'LineWidth',2)
% hold on 
% axis square
% xlabel('time (s)')
% ylabel('Normaized_Torque')
% legend('J_{1}','J_{2}','J_{3}','J_{4}')
% grid on
% grid minor
% 
% 
% figure
% subplot(2,2,1)
% plot(1:i+1,theta(:,1),'LineWidth',2)
% hold on
% plot(1:i,-180*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:i,180*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{1}')
% grid on
% grid minor
% 
% subplot(2,2,2)
% plot(1:i+1,theta(:,2),'LineWidth',2)
% hold on
% plot(1:i,-117*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:i,90*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{2}')
% grid on
% grid minor
% 
% subplot(2,2,3)
% plot(1:i+1,theta(:,3),'LineWidth',2)
% hold on
% plot(1:i,-90*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:i,87.5*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{3}')
% grid on
% grid minor
% 
% subplot(2,2,4)
% plot(1:i+1,theta(:,4),'LineWidth',2)
% hold on
% plot(1:i,-103*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{4}')
% grid on
% grid minor