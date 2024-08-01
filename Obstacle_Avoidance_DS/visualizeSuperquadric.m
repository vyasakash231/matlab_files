function visualizeSuperquadric()
    % Parameters for superquadric
    a1 = 0.75; a2 = 0.3; a3 = 0.3;  % Scale parameters
    e1 = 0.7; e2 = 1;  % Shape parameters
    
    % Generate superquadric in local frame
    [X, Y, Z] = generateSuperquadric(a1, a2, a3, e1, e2);
    
    % Define transformation
    translation = [2; 1; 1];
    rotation = eul2rotm([pi/4, pi/6, pi/3]);  % Euler angles to rotation matrix
    T = [rotation, translation; 0 0 0 1];  % Homogeneous transformation matrix
    
    % Apply transformation
    [X_global, Y_global, Z_global] = applyTransformation(X, Y, Z, T);
    
    % Plotting
    figure;
    surf(X_global, Y_global, Z_global);
    hold on;
    plotCoordinateSystem(eye(4), 'r', 'g', 'b');  % World coordinate system
    plotCoordinateSystem(T, 'r', 'g', 'b');  % Local coordinate system
    title('Superquadric in Global Frame');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis square;
    axis([0 3 0 3 0 3])
    
    
    % Display transformation matrix
    disp('Transformation Matrix:');
    disp(T);
end

function [X, Y, Z] = generateSuperquadric(a1, a2, a3, e1, e2)
    [eta, omega] = meshgrid(linspace(-pi/2, pi/2, 50), linspace(-pi, pi, 50));
    
    X = a1 * sign(cos(eta)) .* abs(cos(eta)).^e1 .* sign(cos(omega)) .* abs(cos(omega)).^e2;
    Y = a2 * sign(cos(eta)) .* abs(cos(eta)).^e1 .* sign(sin(omega)) .* abs(sin(omega)).^e2;
    Z = a3 * sign(sin(eta)) .* abs(sin(eta)).^e1;
end

function [X_new, Y_new, Z_new] = applyTransformation(X, Y, Z, T)
    % Homogeneous coordinates
    points = [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
    
    % Apply transformation
    new_points = T * points;
    
    % Extract new coordinates
    X_new = reshape(new_points(1, :), size(X));
    Y_new = reshape(new_points(2, :), size(Y));
    Z_new = reshape(new_points(3, :), size(Z));
end

function plotCoordinateSystem(T, xColor, yColor, zColor)
    origin = T(1:3, 4);
    xAxis = origin + T(1:3, 1);
    yAxis = origin + T(1:3, 2);
    zAxis = origin + T(1:3, 3);
    
    plot3([origin(1), xAxis(1)], [origin(2), xAxis(2)], [origin(3), xAxis(3)], xColor, 'LineWidth', 3);
    plot3([origin(1), yAxis(1)], [origin(2), yAxis(2)], [origin(3), yAxis(3)], yColor, 'LineWidth', 3);
    plot3([origin(1), zAxis(1)], [origin(2), zAxis(2)], [origin(3), zAxis(3)], zColor, 'LineWidth', 3);
end