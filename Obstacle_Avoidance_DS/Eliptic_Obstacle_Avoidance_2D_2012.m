clc; clear all; close all;

% Time settings
dt = 0.01;
T = 0:dt:75;

% Goal state
x_g = 3.8; y_g = 1.5;

% Define obstacle (for simplicity, let's use a circular obstacle)
r1 = 0.3; r2 = 0.55; m = 4;
obstacle_center = [2; 1.5];
[x, y] = meshgrid(linspace(-5, 5, 150));
F = ((x-obstacle_center(1)) / r1).^m + ((y-obstacle_center(2)) / r2).^m - 1;

contour_level = 0; % The level at which you want to create the "isocurve"
C = contourc(x(1,:), y(:,1), F, [contour_level contour_level]);

% Initial conditions
no_of_robots = 20;
zeta_initial = 0;
for j = 1:no_of_robots
    Zeta_vec(:,j,1) = [0; zeta_initial];  % initial condition [zeta_1, zeta_2] for J no of robots
    zeta_initial = zeta_initial + 0.175;
end

% Simulation
for i = 1:length(T)
    Zeta = Zeta_vec(:,:,i); % shape [2, 25x25]s
    
    % Original dynamical system
    f = -[Zeta(1,:) - x_g; Zeta(2,:) - y_g]; % shape [2, 25x25]
    [~,c] = size(f);
    
    % Modulation
    M = modulation_matrix(Zeta, r1, r2, m, obstacle_center, c); % shape [2,2,25x25]
    
    % Modified dynamical system (vectorized)
    Zeta_dot_vec = zeros(size(f));
    for j = 1:2
        for k = 1:2
            Zeta_dot_vec(j,:) = Zeta_dot_vec(j,:) + squeeze(M(j,k,:)).' .* f(k,:);
        end
    end
    
    % Update state
    Zeta_vec(:,:,i+1) = Zeta + Zeta_dot_vec * dt;
end


% Plotting
figure;
% Extract the coordinates from the contour matrix
for j = 1:c  % no of robots
    x_plot = reshape(Zeta_vec(1,j,:),[1,length(T)+1]);
    y_plot = reshape(Zeta_vec(2,j,:),[1,length(T)+1]);
    plot(x_plot, y_plot, "LineWidth", 1.5, "Color", 'red')
    hold on
end

hold on

k = 1;
while k < size(C, 2)
    n = C(2, k);
    x_contour = C(1, k+1:k+n);
    y_contour = C(2, k+1:k+n);
    k = k + n + 1;

    % Plot the patch
    hold on;
    patch(x_contour, y_contour, 'b', 'FaceAlpha', 0.5); % 'r' specifies the color red
end

hold on
plot(obstacle_center(1), obstacle_center(2), "o", 'MarkerSize', 8, 'MarkerEdgeColor','g', 'MarkerFaceColor','g')
hold on
plot(x_g, y_g, "pentagram", 'MarkerSize', 10, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')


axis([0 4 -0.5 3.5])
axis square
grid on;
xlabel('\zeta_{1}','FontSize',15)
ylabel('\zeta_{2}','FontSize',15)
title('Dynamical System with Obstacle Avoidance')

% Modulation function
function M = modulation_matrix(zeta, r1, r2, m, obstacle, c)
    % relative position
    x = zeta(1,:) - obstacle(1);  % shape [1, 25x25]
    y = zeta(2,:) - obstacle(2);  % shape [1, 25x25]

    % gradient wrt x,y,z
    gx = (m/(r1^m)) * (x.^(m-1));  % shape [1, 25x25]
    gy = (m/(r2^m)) * (y.^(m-1));  % shape [1, 25x25]
    grad = [gx; gy];  % shape [2, 25x25]

    % Normalize gradient to get normal vector
    n_vec = grad ./ vecnorm(grad);  % shape [2, 25x25]
    
    n_vec_3D = [n_vec; zeros(1,c)];
    z_basis_3D = repmat([0; 0; 1],1,c);

    e = cross(n_vec_3D, z_basis_3D);
    e = e ./ vecnorm(e);  % shape [2, 25x25]
    
    E(:,:,1) = n_vec; 
    E(:,:,2) = e(1:2,:);
    
    % Permute the dimensions
    E = permute(E, [1, 3, 2]);  % shape [2, 1, 25x25]

    % Compute distance to the surface
    d = ((x/r1).^m + (y/r2).^m).^(1/m);
    
    % Compute eigenvalues
    lambda_1 = 1 - 1./abs(d);
    lambda_2 = 1 + 1./abs(d);
    
    for i=1:length(lambda_1)
        D = diag([lambda_1(i), lambda_2(i)]);
        M(:,:,i) = E(:,:,i) * D * E(:,:,i)';  % shape [2,2,25x25]
    end
end