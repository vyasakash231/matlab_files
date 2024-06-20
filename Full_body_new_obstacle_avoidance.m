%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

%X_g = -0.05; Y_g = -0.2; Z_g = 0.075;  % original_goal state
X_g = 0.08; Y_g = -0.15; Z_g = 0.16;  % original_goal state

% Define sphere parameters (obstacle center)
%obstacle = [0.075; -0.075; 0.22]; 
%obstacle = [0.085; -0.085; 0.22];
obstacle = [0.09; -0.09; 0.185];
%obstacle = [0.05; -0.075; 0.2];
%obstacle = [0.09; -0.085; 0.2];  % failed case

radius = 0.02;
q_angle = linspace(0, 2*pi, 40);  % azimuthal angle
psi = linspace(0, pi, 40);  % polar angle

xs = obstacle(1) + radius * sin(psi') .* cos(q_angle);
ys = obstacle(2) + radius * sin(psi') .* sin(q_angle);
zs = obstacle(3) + radius * cos(psi') * ones(size(q_angle));

% Define Influence_Radius sphere
scale = 2;
Influence_radius = 0.014;  % if the distance btw manipulator and obstacle is less then this, apply force
D2 = radius + Influence_radius;  % inner Influence sphere
D1 = radius + scale*Influence_radius;  % outer Influence sphere

q_bound = linspace(0, 2*pi, 30);  % azimuthal angle
psi_bound = linspace(0, pi, 30);  % polar angle
[Theta, Phi] = meshgrid(q_bound, psi_bound);

x1_bound = obstacle(1) + D2 * sin(psi_bound') .* cos(q_bound);
y1_bound = obstacle(2) + D2 * sin(psi_bound') .* sin(q_bound);
z1_bound = obstacle(3) + D2 * cos(psi_bound') * ones(size(q_bound));

% Define Influence_Radius sphere
x2_bound = obstacle(1) + D1 * sin(psi_bound') .* cos(q_bound);
y2_bound = obstacle(2) + D1 * sin(psi_bound') .* sin(q_bound);
z2_bound = obstacle(3) + D1 * cos(psi_bound') * ones(size(q_bound));

% Define plane vertices
x = linspace(-10, 10, 100);  % x coordinates
y = linspace(-10, 10, 100);  % y coordinates
[xp, yp] = meshgrid(x, y);  % create a grid of x and y coordinates
zp = zeros(size(xp));  % z coordinates are all zeros (plane at z=0)

n = 4;  % No of Joint
p = 0.128; q = 0.024;

% DH parameters
alpha = [0,90,0,0];  % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
%theta = [90,90,-79.38,-45];
theta = [0,90,-79.38,-45];
Le = 0.126;  % End-effector length

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(1,:) = X_cord; 
Y(1,:) = Y_cord;
Z(1,:) = Z_cord;

X_start = X_cord(end);
Y_start = Y_cord(end);
Z_start = Z_cord(end);

R = diag([0.1,0.1,0.15,0.15]);
Q = diag([70000,70000,50000]);
beta = diag([0.25,0.25,0.25,0.25]);

d_t = 0.01;
t = 0:d_t:5; 

critic_vel_norm = zeros(3,n);
coord_of_critical_vel = zeros(3,n);
unit_vec_links_to_obstacle = [];
links_unit_vec = [];
new_critical_vel_norm = [];

del_X = ones(3,1);
for i=1:50 %length(t)
    [D, O_m] = Critical_Point(radius, obstacle, X_cord, Y_cord, Z_cord);

    % Store CLOSEST Point
    closest_point(:,:,i) = O_m;

    % End-effector coord
    X_e = X(i,n+2);
    Y_e = Y(i,n+2);
    Z_e = Z(i,n+2);

    dX = [(X_g - X_e); (Y_g - Y_e); (Z_g - Z_e)];
    del_X(:,i) = dX;  % for plotting
    
    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));
    
    phi = (Jv' * dX) / norm(Jv' * dX); % (4,1)
    d_theta = (1-exp(-i*0.05)) * sqrt(dX' * Q * dX) * (inv(R).^0.5 * phi);  % HJB Control Input
    
    if any(D <= 2*Influence_radius)  % check if any of the close point is within Influence sphere
        idx = [find(D <= 2*Influence_radius)];  % store all close point (within Influence sphere)
        [critical_point_velocity, point] = Joint_Velocity_to_Link_Velocity(D, O_m, Influence_radius, d_theta, alpha, a, d, theta(i,:));  % (3,4), (3,4)       
        if length(idx) > 1  % If multiple point on the manipulator are close to the obstacle
            for  w=1:length(idx)  % If the point is too close apply force on that point
                critic_vel_norm(:,idx(w),i) =  critical_point_velocity(:,idx(w)) / norm(critical_point_velocity(:,idx(w)));
                coord_of_critical_vel(:,idx(w),i) = point(:,idx(w));
            end
        else  % If single point on the manipulator are close to the obstacle
            critic_vel_norm(:,idx,i) =  critical_point_velocity(:, idx) / norm(critical_point_velocity(:, idx));
            coord_of_critical_vel(:,idx,i) = point(:, idx);            
        end

        for u=1:size(critical_point_velocity, 2)  % for each link closest_point
            unit_vec_links_to_obstacle(:,u,i) = (obstacle - coord_of_critical_vel(:,u,i)) / norm(obstacle - coord_of_critical_vel(:,u,i));
            links_unit_vec(:,u,i) = cross(unit_vec_links_to_obstacle(:,u,i), critic_vel_norm(:,u,i));

            gamma(u) = acos(dot(unit_vec_links_to_obstacle(:,u,i), critic_vel_norm(:,u,i)));  % in radians

            z = min(D);

            if (gamma(u) < pi/2)             
                lambda = (pi/2 - gamma(u)) * ((norm(obstacle - coord_of_critical_vel(:,u,i)) - D1) / (D2 - D1));  % in radians
                new_critical_vel(:,u) = Linear_Velocity_transform(critical_point_velocity(:,u), coord_of_critical_vel(:,u,i), links_unit_vec(:,u,i), lambda);
                new_critical_vel_norm(:,u,i) = new_critical_vel(:,u) / norm(new_critical_vel(:,u));
                d_theta_ext(:,u) = exp(-100*(D(u) - z)) * Link_Velocity_to_Joint_Velocity(u, O_m(:,u), new_critical_vel(:,u), alpha, a, d, theta(i,:));
            end
        end

        if any(gamma(idx) < pi/2)
            d_theta = beta * d_theta + sum(d_theta_ext,2);
        %    d_theta = (eye(size(beta)) - beta) * d_theta + beta * sum(Link_Velocity_to_Joint_Velocity(D, O_m, Influence_radius, new_critical_vel, alpha, a, d, theta(i,:)), 2);
        end
    end
    
    control_input(:,i+1) = d_theta * (pi/180);
    
    % Solving using Euler-forward method
    theta_new = theta(i,:) + d_theta' * d_t;  % here theta is in degrees
    
    theta(i+1,:) = theta_new; % In degrees  (for plotting)
    
    % Next Step performed
    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta_new,Le);

    X(i+1,:) = X_cord; % for plotting
    Y(i+1,:) = Y_cord; % for plotting
    Z(i+1,:) = Z_cord; % for plotting

    if norm(dX) < 0.005  % if distance btw goal and ee is less then 1mm
        break
    end

end

% Initialize the plots
plotHandles = initializePlots(X_start, Y_start, Z_start, X_g, Y_g, Z_g, xs, ys, zs, x1_bound, y1_bound, z1_bound, x2_bound, y2_bound, z2_bound, xp, yp, zp);
const = 0.225; % scale unit vector for display
for k = 1:i
    updatePlots(plotHandles, X, Y, Z, k, closest_point, X_g, Y_g, Z_g, obstacle, [], [], [], [], const, [], critic_vel_norm, coord_of_critical_vel, unit_vec_links_to_obstacle, links_unit_vec, new_critical_vel_norm);
    pause(0.12); % Adjust the pause duration as needed
end

figure
subplot(1,2,1)
plot(t(1:i),del_X(1:3,:),'--','LineWidth',2)
hold on 
plot(t(1:i),rms(del_X(1:3,:)),'LineWidth',2)
axis square
xlabel('time (s)')
ylabel('Position error(mm)')
legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})','RMS Error')
grid on
grid minor

subplot(1,2,2)
plot(t(1:i),control_input(:,1:i),'LineWidth',2)
hold on 
axis square
xlabel('time (s)')
ylabel('Control Input (rad/s)')
legend('J_{1}','J_{2}','J_{3}','J_{4}')
grid on
grid minor


%figure
%subplot(2,2,1)
%plot(1:i+1,theta(:,1),'LineWidth',2)
%hold on
%plot(1:i,-180*ones(1,i),'--','LineWidth',2)
%hold on
%plot(1:i,180*ones(1,i),'--','LineWidth',2)
%hold off
%xlabel('No of Iteration')
%ylabel('\theta_{1}')
%grid on
%grid minor
%
%subplot(2,2,2)
%plot(1:i+1,theta(:,2),'LineWidth',2)
%hold on
%plot(1:i,-117*ones(1,i),'--','LineWidth',2)
%hold on
%plot(1:i,90*ones(1,i),'--','LineWidth',2)
%hold off
%xlabel('No of Iteration')
%ylabel('\theta_{2}')
%grid on
%grid minor
%
%subplot(2,2,3)
%plot(1:i+1,theta(:,3),'LineWidth',2)
%hold on
%plot(1:i,-90*ones(1,i),'--','LineWidth',2)
%hold on
%plot(1:i,87.5*ones(1,i),'--','LineWidth',2)
%hold off
%xlabel('No of Iteration')
%ylabel('\theta_{3}')
%grid on
%grid minor
%
%subplot(2,2,4)
%plot(1:i+1,theta(:,4),'LineWidth',2)
%hold on
%plot(1:i,-103*ones(1,i),'--','LineWidth',2)
%hold on
%plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
%hold off
%xlabel('No of Iteration')
%ylabel('\theta_{4}')
%grid on
%grid minor