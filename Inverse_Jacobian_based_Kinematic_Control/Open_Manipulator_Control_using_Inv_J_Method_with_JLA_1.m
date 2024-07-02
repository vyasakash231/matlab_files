%% Configuration control Method with Joint Limit avoidance using Inequality Constraints
%{
The limit of the joints are defined by part-time constraints as aditional
tasks. These part-time additional task are active for a joint, when the
joint position is close to the joint limit, when a joint position is far
from the joint limit, the JLA additional task becomes inactive for that
joint.
A JLA additional task is activated and deactivated by wisely selecting its
corresponding weight matrix-Wc in the configuration control formulation.
%}
clc; clear all; close all;
syms theta1 theta2 theta3 theta4 

% Goal Location in Task Space
X_g = 0.1; Y_g = -0.2; Z_g = 0.12;
%X_g = -0.247; Y_g = -0.005; Z_g = 0.211;

% Define sphere parameters (obstacle)
obstacle = [0.1;-0.12;0.15];
radius = 0.02;
q_angle = linspace(0, 2*pi, 30);  % azimuthal angle
psi = linspace(0, pi, 30);  % polar angle

xs = obstacle(1) + radius * sin(psi') * cos(q_angle);
ys = obstacle(2) + radius * sin(psi') * sin(q_angle);
zs = obstacle(3) + radius * cos(psi') * ones(size(q_angle));

% Define Influence_Radius sphere
Influence_radius = 0.02;  % if the distance btw manipulator and obstacle is less then this, apply force
q_bound = linspace(0, 2*pi, 30);  % azimuthal angle
psi_bound = linspace(0, pi, 30);  % polar angle
[Theta, Phi] = meshgrid(q_bound, psi_bound);

x_bound = obstacle(1) + (radius + Influence_radius) * sin(psi_bound') .* cos(q_bound);
y_bound = obstacle(2) + (radius + Influence_radius) * sin(psi_bound') .* sin(q_bound);
z_bound = obstacle(3) + (radius + Influence_radius) * cos(psi_bound') * ones(size(q_bound));

% Define plane vertices
x = linspace(-10, 10, 50); % x coordinates
y = linspace(-10, 10, 50); % y coordinates
[xp, yp] = meshgrid(x, y); % create a grid of x and y coordinates
zp = zeros(size(xp)); % z coordinates are all zeros (plane at z=0)

n = 4; % No of Joint
p = 0.128; q = 0.024;

Q = [0,0,0,0]; % Initia angle in hardware
offset = [0,79.38,-79.38,0]; % Joint angle offset in hardware

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = Q + offset; % In degree 
Le = 0.126; % End-effector length

i = 1;

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(i,:) = X_cord;
Y(i,:) = Y_cord;
Z(i,:) = Z_cord;

% Addition Constraints
zc = [theta1; theta2; theta3; theta4];
theta_vector = [theta1; theta2; theta3; theta4]; % Joint variable vector
Jc = double(jacobian(zc,theta_vector)); % Jacobian matrix of additional task (4x4)

Q_range = [-180 180;-117 90;-90 87.5;-103 114.5]; % range of motion of each joint

%{
When the joint position enters this region(activation buffer), the weight of the JLA Wc 
task is increased from zero to a maximum at the lower limit (qimin) or upper limit (qi max).
%}
buffer_angle = 0.25*ones(size(Q)); % activation buffer

del_X = ones(3,i);
while norm(del_X(1:3,i)) > 0.01
    X_c = X(i,n+2); 
    Y_c = Y(i,n+2); 
    Z_c = Z(i,n+2);

    [We,Wc,Wv] = Weight_matrix(3, n, deg2rad(Q_range), buffer_angle, deg2rad(Q(i,:))); % weight matrix is calculated based on Q of hardware
    
    del_X(:,i+1) = [(X_g - X_c); (Y_g - Y_c); (Z_g - Z_c)];
    
    [~,Je,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:)); % Jacobain matrix of Main task (3x4)

    J = (Je'*We*Je + Jc'*Wc*Jc + Wv)\(Je'*We);
    
    % Solving using Newton-Raphson Method
    theta_new = theta(i,:) + (J*del_X(:,i+1))'; % In degree --> [Q_new + offset] = [Q + offset] + (J*del_X(:,i+1))'
    Q(i+1,:) = theta_new - offset; % Q_new = [Q + offset] + (J*del_X(:,i+1))' - offset
    theta(i+1,:) = theta_new; % In degree

    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta(i+1,:),Le);

    i = i + 1;
    
    X(i,:) = X_cord;
    Y(i,:) = Y_cord;
    Z(i,:) = Z_cord;

    if i > 750
        break
    end
end

figure('units','normalized','outerposition',[0 0 1 1]);  % to make full-screen view
for k = 1:i
    plot3(X(k,1:2),Y(k,1:2),Z(k,1:2),'-','LineWidth',8);
    hold on
    plot3(X(k,2:3),Y(k,2:3),Z(k,2:3),'-','LineWidth',7);
    hold on
    plot3(X(k,3:4),Y(k,3:4),Z(k,3:4),'-','LineWidth',6);
    hold on
    plot3(X(k,4:5),Y(k,4:5),Z(k,4:5),'-','LineWidth',5);
    hold on
    plot3(X(k,5:6),Y(k,5:6),Z(k,5:6),'-','LineWidth',4);
    hold on
    plot3(X(1:k,6),Y(1:k,6),Z(1:k,6),'LineWidth',1.5,'Color','m');
    hold on
    % plot3(xs, ys, zs, 'k'); % plot a sphere
    hold on
    plot3(X_g, Y_g, Z_g, 'or', 'LineWidth', 4); % plot goal
    hold on
    % surf(x_bound, y_bound, z_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.2);
    hold on
    surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5); % plot the plane
    hold on
    view(190,25);
    axis([-0.26 0.26 -0.26 0.26 0 0.52])
    axis square
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    grid on
    hold on
    drawnow
    hold off
end

figure
plot(2:i,del_X(1:3,2:end),'LineWidth',2)
xlabel('No of Iteration')
ylabel('Position error(mm)')
legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})')
grid on
grid minor

figure
subplot(2,2,1)
plot(1:i,Q(:,1),'LineWidth',2)
hold on
plot(1:i,-180*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,180*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{1}')
grid on
grid minor

subplot(2,2,2)
plot(1:i,Q(:,2),'LineWidth',2)
hold on
plot(1:i,-117*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,90*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{2}')
grid on
grid minor

subplot(2,2,3)
plot(1:i,Q(:,3),'LineWidth',2)
hold on
plot(1:i,-90*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,87.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{3}')
grid on
grid minor

subplot(2,2,4)
plot(1:i,Q(:,4),'LineWidth',2)
hold on
plot(1:i,-103*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{4}')
grid on
grid minor
