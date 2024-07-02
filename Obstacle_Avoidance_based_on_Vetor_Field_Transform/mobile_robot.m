% Inverse Jacobian Based Control for Multiple Point Robots in 2D Plane
clc; clear all; close all;

% Initialize the starting positions of the robots
% robotPos = [1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25; 
%             2.05, 2.15, 2.25, 2.35, 2.45, 2.55, 2.65, 2.75, 2.85, 2.95, 3.05, 3.15, 3.25, 3.35, 3.45, 3.55, 3.65, 3.75, 3.85, 3.95]; % Each column represents a robot's starting position
robotPos = [1.5; 
            2.75];  % Each column represents a robot's starting position
% robotPos = [1.5; 3.25];  % Each column represents a robot's starting position
% robotPos = [1.25, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25, 1.35, 1.25, 1.5, 1.25; 
%            2.25, 2.35, 2.45, 2.55, 2.65, 2.75, 2.85, 2.95, 3.05, 3.15, 3.25, 3.35, 3.45, 3.55, 3.65, 3.75]; % Each column represents a robot's starting position

initial = robotPos;

obstacle = [3;3];
radius = 0.2;
buffer = 0.1;
D1 = radius + 2*buffer;
D2 = radius + buffer;

% Dimensions of the robot (rectangle)
robotWidth = 0.15;
robotHeight = 0.075;

% Define the four corners of the rectangle before rotation
corners = [-robotWidth/2, -robotHeight/2;
            robotWidth/2, -robotHeight/2;
            robotWidth/2,  robotHeight/2;
           -robotWidth/2,  robotHeight/2]';

% Desired target position
targetPos = [3.65; 3];

% Control gain (affects the speed of convergence)
K = 0.01;

% Tolerance for stopping criteria
tolerance = 0.01;

% Maximum number of iterations
maxIterations = 500;

% Number of robots
numRobots = size(robotPos, 2);

% Store the robots' paths for plotting
robotPaths = cell(1, numRobots);
for r = 1:numRobots
    robotPaths{r} = robotPos(:, r)';
end

% Iterative control loop
for i = 1:maxIterations
    allRobotsReached = true;
    lambda = zeros(1,numRobots);

    for r = 1:numRobots
        % Compute the error between current position and target position
        error = targetPos - robotPos(:, r);
        e_norm(:, r, i) = error / norm(error);
        obs_norm(:, r, i) = (obstacle - robotPos(:, r)) / norm(obstacle - robotPos(:, r));
        
        normal_vec(:, r, i) = obs_norm(1,r,i) * e_norm(2,r,i) - e_norm(1,r,i) * obs_norm(2,r,i);  % k_{cap}

        % Calculate the angle in radians tan(gamma) = |a x b| / a.b
        gamma(r) = atan2(normal_vec(:, r, i), dot(obs_norm(:, r, i), e_norm(:, r, i)));  % this gamma can be +ve and -ve (+pi, -pi)

        % Check if the robot is within the tolerance of the target position
        if norm(error) >= tolerance
            allRobotsReached = false;
            
            % Compute the Jacobian (In 2D, the Jacobian is the identity matrix)
            J = eye(2);
            
            % Compute the inverse Jacobian
            J_inv = inv(J);
        
            % To handle saddle points on th obstacle boundary apply a small perturbation about Z-axis
            if gamma(r) == 0
                gamma(r) = (pi/2) * rand(1);  % In general, you can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1)"
            end

            D = norm(obstacle - robotPos(:, r));
            if D <= D1
                if gamma(r) > 0 && gamma(r) < pi/2  % anti-clock wise rotation
                    lambda(r) = (pi/2 - gamma(r)) * ((D - D1) / (D2 - D1));  % in radians
                end
                if gamma(r) < 0 && gamma(r) > -pi/2  % clock wise rotation
                    lambda(r) = ((-pi/2) - gamma(r)) * ((D - D1) / (D2 - D1));  % in radians
                end

                T = [1, 0, -robotPos(1, r);
                     0, 1, -robotPos(2, r);
                     0, 0,       1        ];
                
                Rz = [cos(lambda(r)), -sin(lambda(r)), 0;
                      sin(lambda(r)),  cos(lambda(r)), 0;
                            0       ,        0       , 1];

                new_targetPos = inv(T) * Rz * T * [targetPos;1];
                error = new_targetPos(1:2) - robotPos(:, r);
                new_e_norm(:,r,i) = error / norm(error);
            end

            % Compute the control input using the inverse Jacobian
            controlInput = K * J_inv * error;
            
            % Update the robot's position
            robotPos(:, r) = robotPos(:, r) + controlInput;
                    
            % Append the new position to the path
            robotPaths{r} = [robotPaths{r}; robotPos(:, r)'];
        end
    end
    
    % If all robots have reached the target position, stop the loop
    if allRobotsReached
        disp('All robots reached the target position.');
        break;
    end
    
    % % % Plot the robots' paths =======================================================================================================================
    figure(1);
    clf;
    color_dict = {[0 0.4470 0.7410], [0.6350 0.0780 0.1840], [0.4940 0.1840 0.3560], [0.7660 0.640 0.50], [0.8500 0.3250 0.0980], ...
                    [0.6940 0.440 0.260], [0.4940 0.1840 0.5560], [0.7660 0.640 0.50], [0.8500 0.3250 0.0980], [0.6940 0.440 0.260], ...
                    [0.7500 0.2250 0.380], [0.5940 0.740 0.150], [0.4940 0.1840 0.5560],[0.3500 0.7250 0.0980], [0.6940 0.740 0.560], ...
                    [0.8500 0.5250 0.2980], [0.4940 0.840 0.260], [0 0.4470 0.7410], [0.6940 0.140 0.560], [0.7940 0.3840 0.5560]};
    hold on;
    rectangle('Position', [obstacle(1)-radius, obstacle(2)-radius, 2*radius, 2*radius], 'Curvature', [1, 1], 'FaceColor', [0.8940 0.7560 0.2840]);
    hold on;
    plot(targetPos(1), targetPos(2), 'k*', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'goal position');
    hold on
    viscircles([obstacle(1), obstacle(2)], D2, "Color", [0.8 0.8 0.8], 'LineWidth', 3);
    hold on;
    viscircles([obstacle(1), obstacle(2)], D1, "Color", [0.8 0.8 0.8], 'LineWidth', 3);
    hold on;
    %for r = 1:numRobots
        %plot(initial(1, r), initial(2, r), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'initial position');
    %end
    for r = 1:numRobots
        plot(robotPaths{r}(:, 1), robotPaths{r}(:, 2), '-.', 'Color', color_dict{r}, 'LineWidth', 3);
        plot(robotPos(1, r), robotPos(2, r), "o", 'LineWidth', 3, 'Color', color_dict{r});
        hold on
        quiver(robotPos(1, r), robotPos(2, r), e_norm(1, r, i), e_norm(2, r, i), 0.175, 'Color','blue', 'LineWidth', 1.5, 'MaxHeadSize', 1);
        try
           quiver(robotPos(1, r), robotPos(2, r), new_e_norm(1, r, i), new_e_norm(2, r, i), 0.175, 'Color','red', 'LineWidth', 1.5, 'MaxHeadSize', 1);
        catch
        end
        quiver(robotPos(1, r), robotPos(2, r), obs_norm(1, r, i), obs_norm(2, r, i), 0.175, 'Color', [0.3660 0.6740 0.30], 'LineWidth', 1.5, 'MaxHeadSize', 1);
        hold on
        % rectangle('Position', [robotPos(1, r) - robotWidth / 2, robotPos(2, r) - robotHeight / 2, robotWidth, robotHeight],'FaceColor', 'blue');
    end
    text(obstacle(1)-0.075, obstacle(2), 'obstacle', 'FontSize', 14, 'Color', 'k', 'FontWeight', 'bold');
    ax = gca;
    ax.FontWeight = 'bold';
    ax.LineWidth = 2;
    ax.FontSize = 15;
    ax.YTick = 2:0.5:4; ax.XTick = 2:0.5:4;
    xlabel('X Position','fontsize',15,'fontweight','bold');
    ylabel('Y Position','fontsize',15,'fontweight','bold');
    % legend('goal position', 'start position','','','','','','','','','','','','','','','','','','','','',...
    % 'robot 1','','robot 2','','robot 3','','robot 4','','robot 5','','robot 6','','robot 7','','robot 8','','robot 9',...
    % '','robot 10','','robot 11','','robot 12','','robot 13','','robot 14','','robot 15');
    % legend('goal state','fontsize',15,'fontweight','bold')
    axis([2 4 2 4]);
    axis square;
    box on
    hold off;
end


% % Plot the robots' paths =======================================================================================================================

% dict = {i-300, 125, 85, 50};
% angle = {25, 0, -45, 5};
% for d=1:4
%     % Rotate the corners
%     theta = deg2rad(angle{d}); % Convert angle to radians
%     R = [cos(theta), -sin(theta);
%         sin(theta),  cos(theta)];

%     rotatedCorners = R * corners;
%     % Translate the corners to the desired position
%     rot_Corners(1,:,d) = rotatedCorners(1,:) + robotPaths{r}(dict{d}, 1);
%     rot_Corners(2,:,d) = rotatedCorners(2,:) + robotPaths{r}(dict{d}, 2);
% end

% figure(2);
% color_dict = {[0.6350 0.0780 0.1840], [0 0.4470 0.7410], [0.4940 0.1840 0.3560], [0.7660 0.640 0.50], [0.8500 0.3250 0.0980], ...
%                 [0.6940 0.440 0.260], [0.4940 0.1840 0.5560], [0.7660 0.640 0.50], [0.8500 0.3250 0.0980], [0.6940 0.440 0.260], ...
%                 [0.7500 0.2250 0.380], [0.5940 0.740 0.150], [0.4940 0.1840 0.5560],[0.3500 0.7250 0.0980], [0.6940 0.740 0.560], ...
%                 [0.8500 0.5250 0.2980], [0.4940 0.840 0.260], [0 0.4470 0.7410], [0.6940 0.140 0.560], [0.7940 0.3840 0.5560]};
% hold on;
% rectangle('Position', [obstacle(1)-radius, obstacle(2)-radius, 2*radius, 2*radius], 'Curvature', [1, 1], 'FaceColor', [0.8940 0.7560 0.2840]);
% hold on;
% plot(targetPos(1), targetPos(2), 'k*', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'goal position');
% hold on
% viscircles([obstacle(1), obstacle(2)], D2, "Color", [0.8 0.8 0.8], 'LineWidth', 3);
% hold on;
% viscircles([obstacle(1), obstacle(2)], D1, "Color", [0.8 0.8 0.8], 'LineWidth', 3);
% hold on;
% %for r = 1:numRobots
%     %plot(initial(1, r), initial(2, r), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'initial position');
% %end
% for r = 1:numRobots
%     plot(robotPaths{r}(:, 1), robotPaths{r}(:, 2), '-.', 'Color', color_dict{r}, 'LineWidth', 3);
%     hold on
%     %plot(robotPos(1, r), robotPos(2, r), "o", 'LineWidth', 3, 'Color', color_dict{r});
%     hold on
%     patch('Faces',[1,2,3,4],'Vertices',rot_Corners(:,:,1)','FaceColor',[0.4940 0.1840 0.5560],'FaceAlpha',.75);
%     hold on
%     patch('Faces',[1,2,3,4],'Vertices',rot_Corners(:,:,2)','FaceColor',[0.4940 0.1840 0.5560],'FaceAlpha',.75);
%     hold on 
%     patch('Faces',[1,2,3,4],'Vertices',rot_Corners(:,:,3)','FaceColor',[0.4940 0.1840 0.5560],'FaceAlpha',.75);
%     hold on 
%     patch('Faces',[1,2,3,4],'Vertices',rot_Corners(:,:,4)','FaceColor',[0.4940 0.1840 0.5560],'FaceAlpha',.75);

%     hold on
%     quiver(robotPaths{r}(end-300, 1), robotPaths{r}(end-300, 2), e_norm(1, r, end-300), e_norm(2, r, end-300), 0.125, 'Color','blue', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(end-300, 1), robotPaths{r}(end-300, 2), obs_norm(1, r, end-300), obs_norm(2, r, end-300), 0.125, 'Color', [0.3660 0.6740 0.30], 'LineWidth', 3, 'MaxHeadSize', 0.75);
    
%     hold on
%     quiver(robotPaths{r}(125, 1), robotPaths{r}(125, 2), e_norm(1, r, 125), e_norm(2, r, 125), 0.125, 'Color','blue', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(125, 1), robotPaths{r}(125, 2), new_e_norm(1, r, 125), new_e_norm(2, r, 125), 0.125, 'Color','red', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(125, 1), robotPaths{r}(125, 2), obs_norm(1, r, 125), obs_norm(2, r, 125), 0.125, 'Color', [0.3660 0.6740 0.30], 'LineWidth', 3, 'MaxHeadSize', 0.75);

%     hold on
%     quiver(robotPaths{r}(85, 1), robotPaths{r}(85, 2), e_norm(1, r, 85), e_norm(2, r, 85), 0.125, 'Color','blue', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(85, 1), robotPaths{r}(85, 2), new_e_norm(1, r, 85), new_e_norm(2, r, 85), 0.125, 'Color','red', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(85, 1), robotPaths{r}(85, 2), obs_norm(1, r, 85), obs_norm(2, r, 85), 0.125, 'Color', [0.3660 0.6740 0.30], 'LineWidth', 3, 'MaxHeadSize', 0.75);
    
%     hold on
%     quiver(robotPaths{r}(50, 1), robotPaths{r}(50, 2), e_norm(1, r, 50), e_norm(2, r, 50), 0.125, 'Color','blue', 'LineWidth', 3, 'MaxHeadSize', 0.75);
%     quiver(robotPaths{r}(50, 1), robotPaths{r}(50, 2), obs_norm(1, r, 50), obs_norm(2, r, 50), 0.125, 'Color', [0.3660 0.6740 0.30], 'LineWidth', 3, 'MaxHeadSize', 0.75);
% end
% % Add text at a specific point (e.g., x = 5, y = sin(5))
% text(obstacle(1)-0.075, obstacle(2), 'obstacle', 'FontSize', 14, 'Color', 'k', 'FontWeight', 'bold');

% ax = gca;
% ax.FontWeight = 'bold';
% ax.LineWidth = 2;
% ax.FontSize = 15;
% ax.YTick = 2:0.5:4; ax.XTick = 2:0.5:4;
% % Add invisible points for the legend
% h1 = plot(nan, nan, '-b', 'LineWidth', 1.5);
% h2 = plot(nan, nan, '-r', 'LineWidth', 1.5);
% h3 = plot(nan, nan, '-g', 'LineWidth', 1.5);

% % Add legend
% legend([h1, h2, h3], {'Actual velocity', 'Rotated velocity', 'Obstacle vector'}, 'Location', 'best');
% xlabel('X Position','fontsize',15,'fontweight','bold');
% ylabel('Y Position','fontsize',15,'fontweight','bold');
% axis([2 4 2 4]);
% axis square;
% box on
% hold off;
