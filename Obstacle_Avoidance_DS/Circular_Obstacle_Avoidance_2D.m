clc; clear all; close all;

% Time settings
dt = 0.05;
T = 0:dt:75;

% Define obstacle (for simplicity, let's use a circular obstacle)
obstacle_center = [9.2; 7.5];
obstacle_radius = 1.8;

% Initial conditions
zeta_initial = -1;
for j = 1:50
    Zeta_vec(:,1,j) = [0; zeta_initial];
    zeta_initial = zeta_initial + 0.4;
end

% Simulation
for j = 1:50
    for i = 1:length(T)
        Zeta = Zeta_vec(:,i,j);
        
        % Original dynamical system
        f = [1; -sin(Zeta(1))];
        
        % Modulation
        M = modulation_matrix(Zeta, struct('center', obstacle_center, 'radius', obstacle_radius));
        
        % Modified dynamical system
        Zeta_dot_vec = M * f;
        
        % Update state
        Zeta_vec(:,i+1,j) = Zeta + Zeta_dot_vec * dt;
    end
end

% Plotting
figure;
for j = 1:50
    plot(Zeta_vec(1,:,j), Zeta_vec(2,:,j), "LineWidth", 1.5, "Color", 'red')
    hold on
end
viscircles(obstacle_center', obstacle_radius, 'Color', 'b');
axis([3 16 0 13])
axis square
xlabel('\zeta_{1}','FontSize',15)
ylabel('\zeta_{2}','FontSize',15)
title('Dynamical System with Obstacle Avoidance')

% Modulation function
function M = modulation_matrix(zeta, obstacle)
    % Implement the modulation matrix M as per equation (5) in the paper, This is a simplified version
    q = zeta - obstacle.center;
    d = norm(q);
    n = q / d;
    E = [n, [-n(2); n(1)]];
    lambda_1 = 1 - (obstacle.radius / d)^2;
    lambda_2 = 1 + (obstacle.radius / d)^2;
    D = diag([lambda_1, lambda_2]);
    M = E * D * E';
end
