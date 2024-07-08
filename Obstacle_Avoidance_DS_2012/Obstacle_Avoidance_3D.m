clc; clear all; close all;

% Time settings
dt = 0.01;
T = 0:dt:10;

%% Define 3D obstacle
a1 = 0.4; a2 = 0.2; a3 = 0.2; m = 4;  % Changed to m = 4
obstacle_center = [0.1, 0, 0];
[x, y, z] = meshgrid(linspace(-1, 1, 50));
F = ((x-obstacle_center(1))/a1).^m + ((y-obstacle_center(2))/a2).^m + ((z-obstacle_center(3))/a3).^m - 1;

% Initial conditions (3D grid)
[X, Y, Z] = meshgrid(-1*ones(4), linspace(-0.75, 0.75, 4), linspace(-0.75, 0.75, 4));  
initial_conditions = [X(:)'; Y(:)'; Z(:)'];

% Simulation
Zeta_vec = zeros(3, length(T)+1, size(initial_conditions, 2));
for j = 1:size(initial_conditions, 2)
    Zeta_vec(:,1,j) = initial_conditions(:,j);
    for i = 1:length(T)
        Zeta = Zeta_vec(:,i,j);
        
        % Original 3D dynamical system (converging to [1,1,1])
        f = -[Zeta(1)-1.5; Zeta(2)-0; Zeta(3)-0];
        
        % Modulation
        M = modulation_matrix_3D(Zeta, a1, a2, a3, n, obstacle_center);
        
        % Modified dynamical system
        Zeta_dot_vec = M * f;
        
        % Update state
        Zeta_vec(:,i+1,j) = Zeta + Zeta_dot_vec * dt;
    end
end

% Plotting
figure;
hold on;

% Plot obstacle
p = patch(isosurface(x, y, z, F, 0));
set(p, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Plot trajectories
for j = 1:size(initial_conditions, 2)
    plot3(Zeta_vec(1,:,j), Zeta_vec(2,:,j), Zeta_vec(3,:,j), 'LineWidth', 1.5)
end

% Plot attractor point
plot3(1.5, 0, 0, 'k*', 'MarkerSize', 10)

xlabel('x'); ylabel('y'); zlabel('z');
title('3D Dynamical System with Obstacle Avoidance (n=4)');
view(3); grid on; axis equal;
xlim([-1 2]); ylim([-1 1.5]); zlim([-1 1.5]);


% Modulation function for 3D
function M = modulation_matrix_3D(zeta, a1, a2, a3, m, obstacle_center)
    % relative position
    x = zeta(1) - obstacle_center(1);
    y = zeta(2) - obstacle_center(2);
    z = zeta(3) - obstacle_center(3);
    
    % gradient wrt x,y,z
    gx = (n/a1) * (x/a1)^(n-1);
    gy = (n/a2) * (y/a2)^(n-1);
    gz = (n/a3) * (z/a3)^(n-1);
    grad = [gx; gy; gz];

    % Normalize gradient to get normal vector
    n_vec = grad / norm(grad); 
    
    % Compute two orthogonal vectors to n_vec
    t1 = cross(n_vec, [0; 0; 1]);
    if norm(t1) < 1e-6
        t1 = cross(n_vec, [0; 1; 0]);
    end
    t1 = t1 / norm(t1);
    t2 = cross(n_vec, t1);
    
    E = [n_vec, t1, t2];
    
    % Compute distance to the surface
    d = ((abs(x)/a1)^m + (abs(y)/a2)^m + (abs(z)/a3)^m)^(1/m);
    
    % Compute eigenvalues
    lambda_1 = 1 - 1/d^(1/0.75);
    lambda_2 = 1 + 1/d^(1/0.75);
    lambda_3 = 1 + 1/d^(1/0.75);
    
    D = diag([lambda_1, lambda_2, lambda_3]);
    M = E * D * E';
end
