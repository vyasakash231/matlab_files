clc; clear all; close all;

addpath(genpath('Utilities'))

% Goal Location in Task Space
% X_g = -0.15; Y_g = -0.2; Z_g = 0.3;
X_g = 0.00; Y_g = -0.15; Z_g = 0.16;  % original_goal state

%% Define 3D obstacle
a1 = 0.03; a2 = 0.03; a3 = 0.03; m = 4;  % m is always even
% obstacle_center = [0.1, -0.1, 0.25];  % zeta_0
obstacle_center = [0.125; -0.1; 0.18];
[xo, yo, zo] = meshgrid(linspace(-0.5, 0.5, 100));
F = ((xo-obstacle_center(1))/a1).^m + ((yo-obstacle_center(2))/a2).^m + ((zo-obstacle_center(3))/a3).^m - 1;

% Define plane vertices
x = linspace(-10, 10, 50); % x coordinates
y = linspace(-10, 10, 50); % y coordinates
[xp, yp] = meshgrid(x, y); % create a grid of x and y coordinates
zp = zeros(size(xp)); % z coordinates are all zeros (plane at z=0)

n = 4; % No of Joint
p = 0.128; q = 0.024;

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = [0,90,-79.38,-45]; % In degree 
Le = 0.126; % End-effector length

R = diag([0.1,0.1,0.15,0.15]);
Q = diag([70000,70000,50000]);

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(1,:) = X_cord;  Y(1,:) = Y_cord;  Z(1,:) = Z_cord;

dt = 0.01;
t = 0:dt:10;

del_X = ones(3,1)/dt;
for i=1:length(t)
    Xe = X(i,n+2);  Ye = Y(i,n+2);  Ze = Z(i,n+2);  % zeta = [Xe; Ye; Ze]
    
    dX = [(X_g - Xe); (Y_g - Ye); (Z_g - Ze)];
    del_X(:,i+1) = dX/dt;
    
    [~,Je,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));  % Jacobain matrix of Main task (3x4)

    % Modulation
    M = modulation_matrix_3D([Xe; Ye; Ze], a1, a2, a3, m, obstacle_center);

    J_inv = inv(Je'*Q*Je + R) * (Je' * Q);

    dtheta =  (1-exp(-i*0.05)) * J_inv * M * del_X(:,i+1);

    control_input(:,i) = deg2rad(dtheta);

    cost(i) = transpose(dX) * Q * (dX) + transpose(dtheta) * R * dtheta;
    
    % Solving using Newton-Raphson Method
    theta(i+1,:) = theta(i,:) + dtheta' * dt; % In degree

    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta(i+1,:),Le);

    X(i+1,:) = X_cord;  Y(i+1,:) = Y_cord;  Z(i+1,:) = Z_cord;
    
    if norm(dX) < 0.002  % if distance btw goal and ee is less then 1mm
        break
    end
end

% figure('units','normalized','outerposition',[0 0 1 1]);  % to make full-screen view
for k = 1:i-100
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
    plot3(X(1:k,6),Y(1:k,6),Z(1:k,6),'LineWidth',2,'Color','m');
    hold on
    plot3(X(1,6),Y(1,6),Z(1,6), 'og', 'LineWidth', 4); % plot initial state
    hold on
    plot3(X_g, Y_g, Z_g, 'or', 'LineWidth', 4); % plot goal
    hold on
    p = patch(isosurface(xo, yo, zo, F, 0));  % Plot obstacle
    set(p, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    hold on
    surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5); % plot the plane
    
    view(240,20);
    axis([-0.25 0.25 -0.25 0.25 0 0.5])
    axis square
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    % grid on
    hold off
    drawnow
end

figure
subplot(1,3,1)
plot(1:size(del_X,2)-1,del_X(1:3,2:end),'LineWidth',2)
xlabel('No of Iteration')
ylabel('Position error(mm)')
legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})')
grid on
grid minor
axis square

subplot(1,3,2)
plot(1:i,control_input(:,1:i),'LineWidth',2)
hold on 
axis square
xlabel('time (s)')
ylabel('Control Input (rad/s)')
legend('J_{1}','J_{2}','J_{3}','J_{4}')
grid on
grid minor

subplot(1,3,3)
plot(1:length(cost),cost,'LineWidth',2)
xlabel('No of Iteration')
ylabel('Tajectory Cost')
grid on
grid minor
axis square


% Modulation function for 3D
function M = modulation_matrix_3D(zeta, a1, a2, a3, m, obstacle_center)
    % Compute relative position
    x = zeta(1) - obstacle_center(1);
    y = zeta(2) - obstacle_center(2);
    z = zeta(3) - obstacle_center(3);
    
    % Compute gradient analytically
    gx = (m/a1) * (x/a1)^(m-1);
    gy = (m/a2) * (y/a2)^(m-1);
    gz = (m/a3) * (z/a3)^(m-1);
    grad = [gx; gy; gz];

    % Normalize gradient to get normal vector
    n_vec = grad / norm(grad);  % Added eps to avoid division by zero

    % Compute two orthogonal vectors to n_vec (one way is Gram-Schmidt process orthonormalizing method but Q'll use a simple trick)
    t1 = cross(n_vec, [1; 0; 0]);  % compute the cross product of n_vec and standard basis along x-axis (this will give a vector perpendicular to n_vec and x-axis) 
    if norm(t1) < 1e-6  % if n_vec is aligned with the x-axis, there cross-product will be very small
        t1 = cross(n_vec, [0; 1; 0]);  % compute the cross product of n_vec and standard basis along y-axis
        if norm(t1) < 1e-6  % if n_vec is aligned with the x-axis, there cross-product will be very small
            t1 = cross(n_vec, [0; 0; 1]);  % compute the cross product of n_vec and standard basis along z-axis
        end
    end
    t1 = t1 / norm(t1);

    t2 = cross(n_vec, t1);  % to find another vector perpendicular to n_vec take cross product of n_vec and t1
    
    E = [n_vec, t1, t2];
    
    % Compute distance to the surface
    d = ((abs(x)/a1)^m + (abs(y)/a2)^m + (abs(z)/a3)^m)^(1/m);
    
    % Compute eigenvalues
    % The larger the reactivity ρ, the larger the amplitude of the deflection, and consequently the earlier the robot responds to the presence of an obstacle
    rho = 0.5;  % where ρ > 0 is the reactivity parameter
    lambda_1 = 1 - 1/d^(1/rho);
    lambda_2 = 1 + 1/d^(1/rho); 
    lambda_3 = 1 + 1/d^(1/rho);
    
    D = diag([lambda_1, lambda_2, lambda_3]);
    M = E * D * E';
end

