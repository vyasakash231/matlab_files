clc; clear all; close all;

% Time settings
dt = 0.01;
T = 0:dt:10;

% Define 2D obstacle
obstacle = @(x, y) ((x-0.5)/0.4)^2 + ((y-0.5)/0.3)^2 - 1;
obstacle_grad = @(x, y) [2*(x-0.5)/0.4^2; 2*(y-0.5)/0.3^2];

% Original dynamical system
f = @(x, y) [-x + y; -x - y];

% Initial conditions
[X, Y] = meshgrid(linspace(-1, 1.5, 10), linspace(-1, 1.5, 10));
initial_conditions = [X(:)'; Y(:)'];

% Simulation
Zeta_vec = zeros(2, length(T)+1, size(initial_conditions, 2));
for j = 1:size(initial_conditions, 2)
    Zeta_vec(:,1,j) = initial_conditions(:,j);
    for i = 1:length(T)
        Zeta = Zeta_vec(:,i,j);
        % Modulation
        M = modulation_matrix_2D(Zeta, obstacle, obstacle_grad);
        % Modified dynamical system
        Zeta_dot = M * f(Zeta(1), Zeta(2));
        % Update state
        Zeta_vec(:,i+1,j) = Zeta + Zeta_dot * dt;
    end
end

% Plotting
figure;
hold on;

% Plot trajectories
for j = 1:size(initial_conditions, 2)
    plot(Zeta_vec(1,:,j), Zeta_vec(2,:,j), 'LineWidth', 1.5)
end

% Plot vector field
[X, Y] = meshgrid(linspace(-1, 2, 20), linspace(-1, 2, 20));
U = zeros(size(X));
V = zeros(size(Y));
for i = 1:numel(X)
    Zeta = [X(i); Y(i)];
    M = modulation_matrix_2D(Zeta, obstacle, obstacle_grad);
    Zeta_dot = M * f(Zeta(1), Zeta(2));
    U(i) = Zeta_dot(1);
    V(i) = Zeta_dot(2);
end
quiver(X, Y, U, V, 'k');
hold on

% Create and plot obstacle
theta = linspace(0, 2*pi, 100);
x_ellipse = 0.4 * cos(theta) + 0.5;
y_ellipse = 0.3 * sin(theta) + 0.5;
fill(x_ellipse, y_ellipse, 'g', 'FaceAlpha', 1);
plot(x_ellipse, y_ellipse, 'k', 'LineWidth', 2);

xlabel('x'); ylabel('y');
title('2D Dynamical System with Obstacle Avoidance');
axis equal;
xlim([-1 2]); ylim([-1 2]);
grid on;
legend('Obstacle', 'Obstacle Boundary', 'Trajectories', 'Vector Field');

% Modulation function for 2D
function M = modulation_matrix_2D(zeta, obstacle, obstacle_grad)
    grad = obstacle_grad(zeta(1), zeta(2));
    n = grad / (norm(grad) + eps);
    % Compute orthogonal vector to n
    e = [-n(2); n(1)];
    E = [n, e];
    % Compute distance to the surface
    d = max(abs(obstacle(zeta(1), zeta(2))), eps);
    % Compute eigenvalues
    lambda_1 = 1 - exp(-d);
    lambda_2 = 1 + exp(-d);
    D = diag([lambda_1, lambda_2]);
    M = E * D * E';
end