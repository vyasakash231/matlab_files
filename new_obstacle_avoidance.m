%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

%addpath E:\Matlab_Files\Robotics\Lab_Assignment % To access function from other folders

eta = 0.004; % force multiplication factor

% Define sphere parameters (obstacle)
obstacle = [0.19;-0.1;0.158934]; % center
radius = 0.02;
q_angle = linspace(0, 2*pi, 15); % azimuthal angle
psi = linspace(0, pi, 15); % polar angle

xs = obstacle(1) + radius * sin(psi') * cos(q_angle);
ys = obstacle(2) + radius * sin(psi') * sin(q_angle);
zs = obstacle(3) + radius * cos(psi') * ones(size(q_angle));

[U,V,W] = surfnorm(xs,ys,zs);

% Define Influence_Radius sphere
scale = 2;
Influence_radius = 0.015; % if the distance btw manipulator and obstacle is less then this, apply force
D2 = radius + Influence_radius; % inner Influence sphere
D1 = radius + scale*Influence_radius; % outer Influence sphere

q_bound = linspace(0, 2*pi, 30); % azimuthal angle
psi_bound = linspace(0, pi, 30); % polar angle
[Theta, Phi] = meshgrid(q_bound, psi_bound);

x1_bound = obstacle(1) + D2 * sin(psi_bound') .* cos(q_bound);
y1_bound = obstacle(2) + D2 * sin(psi_bound') .* sin(q_bound);
z1_bound = obstacle(3) + D2 * cos(psi_bound') * ones(size(q_bound));

% Define Influence_Radius sphere
x2_bound = obstacle(1) + D1 * sin(psi_bound') .* cos(q_bound);
y2_bound = obstacle(2) + D1 * sin(psi_bound') .* sin(q_bound);
z2_bound = obstacle(3) + D1 * cos(psi_bound') * ones(size(q_bound));

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

R = diag([0.1,0.1,0.15,0.15]);
Q = diag([80000,50000,50000]);

d_t = 0.01;
t = 0:d_t:5; 

del_X = ones(3,1);
for i=1:length(t)
    X_g = 0.06; Y_g = -0.2; Z_g = 0.158934;  % original_goal state

    [D, dD_dx, O_m] = Repulsive_Potential_Field(radius, obstacle, X_cord, Y_cord, Z_cord);

    % Store CLOSEST Point
    closest_point(:,:,i) = O_m;

    % End-effector coord
    X_e = X(i,n+2);
    Y_e = Y(i,n+2);
    Z_e = Z(i,n+2);

    % Vector calculation
    A = [obstacle(1) - X_e; obstacle(2) - Y_e; obstacle(3) - Z_e];  % vector from ee to obstacle center
    B = [(X_g - X_e); (Y_g - Y_e); (Z_g - Z_e)];  % vector from ee to original_goal

    n_vec(:,i) = cross(A, B) / norm(cross(A, B));  % normal vector to the plane at ee 

    % data for unit vector plot
    A_unit(:,i) = A / norm(A); % unit vector from ee to obstacle center
    B_unit(:,i) = B / norm(B); % unit vector from ee to goal

    gamma(i) = rad2deg(acos(dot(A_unit(:,i),B_unit(:,i))));

    if (gamma(i) < 90) & (norm(A) <= D1)  % if ee distance from obstacle is less the equal to D1 
        %lambda = (90 - gamma(i))*((2*D2 - norm(A))/D2)^2;
        lambda = (90 - gamma(i)) * ((D1 - norm(A)) / Influence_radius);
        new_X_g(:,i) = Goal_shifting([X_g;Y_g;Z_g], [X_e;Y_e;Z_e], n_vec(:,i), -deg2rad(lambda));
        C = [new_X_g(1,i) - X_e; new_X_g(2,i) - Y_e; new_X_g(3,i) - Z_e];  % vector from ee to new_goal
        C_unit(:,i) = C / norm(C);
        X_g = new_X_g(1,i); Y_g = new_X_g(2,i); Z_g = new_X_g(3,i);
    end

    dX = [(X_g - X_e); (Y_g - Y_e); (Z_g - Z_e)];
    del_X(:,i) = dX;  % for plotting
    
    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));
    
    phi = (Jv' * dX) / norm(Jv' * dX); % (4,1)

    d_theta = (1-exp(-i*0.05)) * sqrt(dX' * Q * dX) * (inv(R).^0.5 * phi);

    control_input(:,i+1) = d_theta * (pi/180);
    
    % Solving using Euler-forward method
    theta_new = theta(i,:) + d_theta' * d_t;
    
    theta(i+1,:) = theta_new; % In degrees  (for plotting)
    
    % Next Step performed
    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta_new,Le);

    X(i+1,:) = X_cord; % for plotting
    Y(i+1,:) = Y_cord; % for plotting
    Z(i+1,:) = Z_cord; % for plotting
end

% Initialize the plots
plotHandles = initializePlots(X_start, Y_start, Z_start, X_g, Y_g, Z_g, xs, ys, zs, x1_bound, y1_bound, z1_bound, x2_bound, y2_bound, z2_bound, xp, yp, zp);
const = 0.3; % scale unit vector for display
for k = 1:i-300
    updatePlots(plotHandles, X, Y, Z, k, closest_point, X_g, Y_g, Z_g, obstacle, A_unit, B_unit, C_unit, n_vec, const, new_X_g, [], [], [], []);
    pause(0.08); % Adjust the pause duration as needed
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