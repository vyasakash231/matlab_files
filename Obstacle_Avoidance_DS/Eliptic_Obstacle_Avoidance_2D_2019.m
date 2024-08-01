clc; clear all; close all;

% % Time settings
dt = 0.01;
T = 0:dt:75;

% Goal state
x_g = 3.8; y_g = 1.5;

% Define obstacle (for simplicity, let's use a circular obstacle)
r1 = 0.3; r2 = 0.55; m = 4;
obstacle_center = [2; 1.5];  % [zeta1_c; zeta2_c]
[x, y] = meshgrid(linspace(-5, 5, 150));
F = ((x-obstacle_center(1)) / r1).^m + ((y-obstacle_center(2)) / r2).^m - 1;

contour_level = 0; % The level at which we want to create the "isocurve"
C = contourc(x(1,:), y(:,1), F, [contour_level contour_level]);

% reference point
reference_point = [2; 1.5];

% Initial conditions
zeta_initial = 0;
for j = 1:20
    Zeta_vec(:,j,1) = [0; zeta_initial];  % initial condition [zeta_1, zeta_2] for J no of robots
    zeta_initial = zeta_initial + 0.175;
end

for i = 1:length(T)
    Zeta = Zeta_vec(:,:,i); % shape [2, 25x25]
    
    % Original dynamical system
    f = -[Zeta(1,:) - x_g; Zeta(2,:) - y_g]; % shape [2, 25x25]
    [~,c] = size(f);
    
    % Modulation
    M = modulation_matrix(Zeta, r1, r2, m, obstacle_center, reference_point, c); % shape [2,2,25x25]
    
    % Modified dynamical system (vectorized)
    Zeta_dot_vec = zeros(size(f));
    for j = 1:2
        for k = 1:2
            Zeta_dot_vec(j,:) = Zeta_dot_vec(j,:) + squeeze(M(j,k,:)).' .* f(k,:);
        end
    end

    Zeta_dot_plot(:,:,i+1) = Zeta_dot_vec;
    
    % Update state
    Zeta_vec(:,:,i+1) = Zeta + Zeta_dot_vec * dt;
end

% Plotting
figure;
% Extract the coordinates from the contour matrix
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

for j = 1:c
    x_plot = reshape(Zeta_vec(1,j,:),[1,length(T)+1]);
    y_plot = reshape(Zeta_vec(2,j,:),[1,length(T)+1]);
    plot(x_plot, y_plot, "LineWidth", 1.5, "Color", 'red')
    hold on
end

hold on
plot(obstacle_center(1), obstacle_center(2), "o", 'MarkerSize', 8, 'MarkerEdgeColor','g', 'MarkerFaceColor','g')
hold on
plot(reference_point(1), reference_point(2), "*", 'MarkerSize', 8, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')
hold on
plot(x_g, y_g, "pentagram", 'MarkerSize', 10, 'MarkerEdgeColor','k', 'MarkerFaceColor','k')

axis([0 4 -0.5 3.5])
axis square
grid on;
xlabel('\zeta_{1}','FontSize',15)
ylabel('\zeta_{2}','FontSize',15)
title('Dynamical System with Obstacle Avoidance')


% % Modulation function
function M = modulation_matrix(zeta, r1, r2, m, obstacle, reference_point, c)
    p = 1;
    x = zeta(1,:) - reference_point(1);
    y = zeta(2,:) - reference_point(2);

    r = [x;y];
    r_norm = r ./ vecnorm(r);  % shape [2, 25x25]

    % Define the equation as a function
    for h=1:c
        equation = @(b) ((b*r(1,h) + reference_point(1) - obstacle(1))/r1)^m + ((b*r(2,h) + reference_point(2) - obstacle(2))/r2)^m - 1;

        % Initial guess for b
        b0 = 1.5; % example initial guess

        % Solve for k using fsolve
        options = optimoptions('fsolve', 'Display', 'off'); % Display iteration output
        [b_, ~, exitflag] = fsolve(equation, b0, options);

        % Display the result
        if exitflag > 0
            b(h) = b_ ;
        else
            fprintf('fsolve did not converge. Try different initial guess or check the equation.\n');
        end
    end

    % ref_point_vec = repmat(reference_point,1,c);
    zeta_b = reference_point + b .* r_norm;  % shape [2, 25x25]
    R = vecnorm(zeta_b - reference_point);  % shape [1, 25x25]

    % Dd_Dxi
    gx = (2*p) * (1 ./ (R.^(2*p))) .* (vecnorm(zeta - obstacle).^(2*p-2)) .* (zeta(1,:) - obstacle(1));
    gy = (2*p) * (1 ./ (R.^(2*p))) .* (vecnorm(zeta - obstacle).^(2*p-2)) .* (zeta(2,:) - obstacle(2));

    n_vec = [gx;gy];

    n_vec = n_vec ./ vecnorm(n_vec);
    z_basis_3D = repmat([0; 0; 1],1,c);

    e = cross([n_vec; zeros(1,c)], z_basis_3D);
    e = e ./ vecnorm(e);
    
    E(:,:,1) = r_norm; 
    E(:,:,2) = e(1:2,:);

    % Permute the dimensions
    E = permute(E, [1, 3, 2]);  % shape [2, 2, 25x25]
    
    % Compute distance to the surface
    d = (vecnorm(zeta - obstacle) ./ R).^2*p;
    
    % Compute eigenvalues (Apply constraints)
    lambda_r = 1 - 1./(d.^(1/0.5));
    lambda_r = min(1, lambda_r); % lambda_r <= 1
    lambda_r(d == 1) = 0;  % lambda_r = 0, at boundary (d == 1)

    lambda_t = 1 + 1./(d.^(1/0.5));
    lambda_t = max(1, lambda_t); % lambda_t >= 1

    % Special case: far from the obstacle
    lambda_r(d > 3) = 1;
    lambda_t(d > 3) = 1;

    D = zeros(2,2,c); % shape [2,2,400]
    D(1,1,:) = lambda_r;
    D(2,2,:) = lambda_t; 

    for i=1:c
        M(:,:,i) = E(:,:,i) * D(:,:,i) * inv(E(:,:,i));  % shape [2,2,400]
    end
end