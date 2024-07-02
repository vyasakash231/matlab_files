%function plotHandles = initializePlots(X_start, Y_start, Z_start, X_g, Y_g, Z_g, xs, ys, zs, x1_bound, y1_bound, z1_bound, x2_bound, y2_bound, z2_bound, xp, yp, zp)
%    % Initialize subplots for different views
%    plotHandles = struct();
%
%    figure('units','normalized','outerposition',[0 0 1 1]);  % to make full-screen view
%
%    % Subplot 1
%    subplot(1, 3, 1);
%    plotHandles.h1_start = plot3(X_start, Y_start, Z_start, 'o', 'LineWidth', 4); % start state
%    hold on;
%    plotHandles.h1_goal = plot3(X_g, Y_g, Z_g, 'o', 'LineWidth', 4); % goal state
%    plotHandles.h1_line1 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0 0.4470 0.7410]);
%    plotHandles.h1_line2 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0.6350 0.0780 0.1840]);
%    plotHandles.h1_line3 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 7, 'Color', [0.4940 0.1840 0.5560]);
%    plotHandles.h1_line4 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 6, 'Color', [0.4660 0.6740 0.1880]);
%    plotHandles.h1_line5 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 5, 'Color', [0.8500 0.3250 0.0980]);
%    plotHandles.h1_closest_point = plot3(NaN, NaN, NaN, '*', 'LineWidth', 5, 'Color', 'k');
%
%    plotHandles.h1_plane = fill3(NaN, NaN, NaN, 'r', 'LineStyle', 'none', 'FaceAlpha', 0.5);  % traingular plane btw ee, obstacle and goal
%    plotHandles.h1_moving_goal = plot3(NaN, NaN, NaN, 'ok', 'LineWidth', 4); % new moving goal state
%    plotHandles.h1_unit_vec_ee_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_ee_to_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_ee_to_moving_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_perpendicular_vector = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    
%    plotHandles.h1_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h1_new_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_new_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_new_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_new_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h1_unit_vec_closest_point_vel_1_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_closest_point_vel_2_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_closest_point_vel_3_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_closest_point_vel_4_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h1_unit_vec_perp_to_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_perp_to_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_perp_to_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h1_unit_vec_perp_to_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h1_sphere = plot3(xs, ys, zs, 'k');  % spherical obstacle bounndary points
%    plotHandles.h1_influence_field_1 = surf(x1_bound, y1_bound, z1_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.1); % infulence sphere 1
%    plotHandles.h1_influence_field_2 = surf(x2_bound, y2_bound, z2_bound, 'FaceColor', 'black', 'EdgeColor', 'none', 'FaceAlpha', 0.1); % infulence sphere 2
%    plotHandles.h1_surface_plane = surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5); % base plane surface
%    plotHandles.h1_trajectory = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'm');
%
%    plotHandles.h1_trajectory_closest_point_4 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%    plotHandles.h1_trajectory_closest_point_3 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%
%    xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
%    title('View 1');
%    view(0, 90); % View from the x-axis
%    grid on;
%    axis([-0.25 0.25 -0.25 0.25 0 0.5])
%    axis square
%
%    % Subplot 2
%    subplot(1, 3, 2);
%    plotHandles.h2_start = plot3(X_start, Y_start, Z_start, 'o', 'LineWidth', 4); % start state
%    hold on;
%    plotHandles.h2_goal = plot3(X_g, Y_g, Z_g, 'o', 'LineWidth', 4); % goal state
%    plotHandles.h2_line1 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0 0.4470 0.7410]);
%    plotHandles.h2_line2 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0.6350 0.0780 0.1840]);
%    plotHandles.h2_line3 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 7, 'Color', [0.4940 0.1840 0.5560]);
%    plotHandles.h2_line4 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 6, 'Color', [0.4660 0.6740 0.1880]);
%    plotHandles.h2_line5 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 5, 'Color', [0.8500 0.3250 0.0980]);
%    plotHandles.h2_closest_point = plot3(NaN, NaN, NaN, '*', 'LineWidth', 5, 'Color', 'k');
%
%    plotHandles.h2_plane = fill3(NaN, NaN, NaN, 'r', 'LineStyle', 'none', 'FaceAlpha', 0.5);  % traingular plane
%    plotHandles.h2_moving_goal = plot3(NaN, NaN, NaN, 'ok', 'LineWidth', 4); % new moving goal state
%    plotHandles.h2_unit_vec_ee_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_ee_to_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_ee_to_moving_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_perpendicular_vector = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h2_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h2_new_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_new_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_new_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_new_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h2_unit_vec_closest_point_vel_1_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_closest_point_vel_2_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_closest_point_vel_3_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_closest_point_vel_4_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h2_unit_vec_perp_to_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_perp_to_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_perp_to_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h2_unit_vec_perp_to_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h2_sphere = plot3(xs, ys, zs, 'k');  % spherical obstacle bounndary points
%    plotHandles.h2_influence_field_1 = surf(x1_bound, y1_bound, z1_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
%    plotHandles.h2_influence_field_2 = surf(x2_bound, y2_bound, z2_bound, 'FaceColor', 'black', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
%    plotHandles.h2_surface_plane = surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5);
%    plotHandles.h2_trajectory = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'm');
%
%    plotHandles.h2_trajectory_closest_point_3 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%    plotHandles.h2_trajectory_closest_point_4 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%
%    xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
%    title('View 2');
%    view(240, 20); % View from the y-axis
%    grid on;
%    axis([-0.25 0.25 -0.25 0.25 0 0.5])
%    axis square
%
%    % Subplot 3
%    subplot(1, 3, 3);
%    plotHandles.h3_start = plot3(X_start, Y_start, Z_start, 'o', 'LineWidth', 4); % start state
%    hold on;
%    plotHandles.h3_goal = plot3(X_g, Y_g, Z_g, 'o', 'LineWidth', 4); % goal state
%    plotHandles.h3_line1 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0 0.4470 0.7410]);
%    plotHandles.h3_line2 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', [0.6350 0.0780 0.1840]);
%    plotHandles.h3_line3 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 7, 'Color', [0.4940 0.1840 0.5560]);
%    plotHandles.h3_line4 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 6, 'Color', [0.4660 0.6740 0.1880]);
%    plotHandles.h3_line5 = plot3(NaN, NaN, NaN, '-', 'LineWidth', 5, 'Color', [0.8500 0.3250 0.0980]);
%    plotHandles.h3_closest_point = plot3(NaN, NaN, NaN, '*', 'LineWidth', 5, 'Color', 'k');
%
%    plotHandles.h3_plane = fill3(NaN, NaN, NaN, 'r', 'LineStyle', 'none', 'FaceAlpha', 0.5);
%    plotHandles.h3_moving_goal = plot3(NaN, NaN, NaN, 'ok', 'LineWidth', 4); % new moving goal state
%    plotHandles.h3_unit_vec_ee_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_ee_to_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_ee_to_moving_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_perpendicular_vector = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    
%    plotHandles.h3_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h3_new_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_new_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_new_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_new_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h3_unit_vec_closest_point_vel_1_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_closest_point_vel_2_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_closest_point_vel_3_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_closest_point_vel_4_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h3_unit_vec_perp_to_closest_point_vel_1 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_perp_to_closest_point_vel_2 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_perp_to_closest_point_vel_3 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%    plotHandles.h3_unit_vec_perp_to_closest_point_vel_4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
%
%    plotHandles.h3_sphere = plot3(xs, ys, zs, 'k');  % spherical obstacle bounndary points
%    plotHandles.h3_influence_field_1 = surf(x1_bound, y1_bound, z1_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
%    plotHandles.h3_influence_field_2 = surf(x2_bound, y2_bound, z2_bound, 'FaceColor', 'black', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
%    plotHandles.h3_surface_plane = surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5);
%    plotHandles.h3_trajectory = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'm');
%
%    plotHandles.h3_trajectory_closest_point_3 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%    plotHandles.h3_trajectory_closest_point_4 = plot3(NaN, NaN, NaN, 'LineWidth', 1, 'Color', 'k');
%
%    xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
%    title('View 3');
%    %view(120, 25); % View from the y-axis
%    view(120, 0); % View from the y-axis
%    grid on;
%    %axis([-0.25 0.25 -0.25 0.25 0 0.5])
%    axis([-0.1 0.25 -0.25 0.1 0 0.35])
%    axis square
%end

%========================================================================================================================================================================================================================================================%


function plotHandles = initializePlots(X_start, Y_start, Z_start, X_g, Y_g, Z_g, xs, ys, zs, x1_bound, y1_bound, z1_bound, x2_bound, y2_bound, z2_bound, xp, yp, zp)
    % Initialize subplots for different views
    plotHandles = struct();

    plotHandles.h1 = struct('lines', {cell(1, 5)}, 'closest_point', [], 'plane', [], 'moving_goal', [], 'unit_vec_ee_to_obstacle', [],...
                    'unit_vec_ee_to_goal',[], 'unit_vec_ee_to_moving_goal', [], 'perpendicular_vector', [], 'closest_point_vel', {cell(1, 4)},...
                    'new_closest_point_vel', {cell(1,4)}, 'unit_vec_closest_point_vel_to_obstacle', [], 'unit_vec_perp_to_closest_point_vel', [],...
                    'sphere', [], 'influence_field_1', [], 'influence_field_2', [], 'surface_plane', [], 'trajectory', [], 'trajectory_closest_point', {cell(1,4)});

    plotHandles.h2 = struct('lines', {cell(1, 5)}, 'closest_point', [], 'plane', [], 'moving_goal', [], 'unit_vec_ee_to_obstacle', [],...
        'unit_vec_ee_to_goal',[], 'unit_vec_ee_to_moving_goal', [], 'perpendicular_vector', [], 'closest_point_vel', {cell(1, 4)},...
        'new_closest_point_vel', {cell(1,4)}, 'unit_vec_closest_point_vel_to_obstacle', [], 'unit_vec_perp_to_closest_point_vel', [],...
        'sphere', [], 'influence_field_1', [], 'influence_field_2', [], 'surface_plane', [], 'trajectory', [], 'trajectory_closest_point', {cell(1,4)});

    plotHandles.h3 = struct('lines', {cell(1, 5)}, 'closest_point', [], 'plane', [], 'moving_goal', [], 'unit_vec_ee_to_obstacle', [],...
        'unit_vec_ee_to_goal',[], 'unit_vec_ee_to_moving_goal', [], 'perpendicular_vector', [], 'closest_point_vel', {cell(1, 4)},...
        'new_closest_point_vel', {cell(1,4)}, 'unit_vec_closest_point_vel_to_obstacle', [], 'unit_vec_perp_to_closest_point_vel', [],...
        'sphere', [], 'influence_field_1', [], 'influence_field_2', [], 'surface_plane', [], 'trajectory', [], 'trajectory_closest_point', {cell(1,4)});

    figure('units','normalized','outerposition',[0 0 1 1]);  % to make full-screen view

    % view angles
    azimuth = [0,240,50];
    elevation_angles = [90,20,10];

    % axis values
    plot_axis = [-0.25, 0.25, -0.25, 0.25, 0, 0.5; 
                -0.25, 0.25, -0.25, 0.25, 0, 0.5; 
                %-0.1, 0.25, -0.25, 0.1, 0, 0.35];
                -0.05, 0.25, -0.25, 0.05, 0.0, 0.3];

    subplots = {'h1', 'h2', 'h3'};

    for i=1:length(subplots)

        subplot_handle = subplots{i};
        color_set = {[0 0.4470 0.7410], [0.6350 0.0780 0.1840], [0.4940 0.1840 0.5560], [0.4660 0.6740 0.1880], [0.8500 0.3250 0.0980]};
        
        subplot(1, 3, i);

        plotHandles.(subplot_handle).start = plot3(X_start, Y_start, Z_start, 'o', 'LineWidth', 4); % start state
        hold on;
        plotHandles.(subplot_handle).goal = plot3(X_g, Y_g, Z_g, 'o', 'LineWidth', 4); % goal state
        
        for j=1:5
            plotHandles.(subplot_handle).lines{j} = plot3(NaN, NaN, NaN, '-', 'LineWidth', 10-j, 'Color', color_set{j});
        end

        plotHandles.(subplot_handle).closest_point = plot3(NaN, NaN, NaN, '*', 'LineWidth', 5, 'Color', 'k');
        plotHandles.(subplot_handle).plane = fill3(NaN, NaN, NaN, 'r', 'LineStyle', 'none', 'FaceAlpha', 0.5);  % traingular plane btw ee, obstacle and goal
        plotHandles.(subplot_handle).moving_goal = plot3(NaN, NaN, NaN, 'ok', 'LineWidth', 4); % new moving goal state
        plotHandles.(subplot_handle).unit_vec_ee_to_obstacle = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2, 'MaxHeadSize', 1.2);
        plotHandles.(subplot_handle).unit_vec_ee_to_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2, 'MaxHeadSize', 1.2);
        plotHandles.(subplot_handle).unit_vec_ee_to_moving_goal = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'k', 'LineWidth', 2, 'MaxHeadSize', 1.2);
        plotHandles.(subplot_handle).perpendicular_vector = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2, 'MaxHeadSize', 1.2);

        for k=1:4
            plotHandles.(subplot_handle).closest_point_vel{k} = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 1.25);
            plotHandles.(subplot_handle).new_closest_point_vel{k} = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'g', 'LineWidth', 2.5, 'MaxHeadSize', 1.25);
            plotHandles.(subplot_handle).unit_vec_closest_point_vel_to_obstacle{k} = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.15, 'b', 'LineWidth', 2.5, 'MaxHeadSize', 1.25);
            plotHandles.(subplot_handle).unit_vec_perp_to_closest_point_vel{k} = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 0.2, 'k', 'LineWidth', 2.5, 'MaxHeadSize', 1.25);
        end

        plotHandles.(subplot_handle).sphere = surf(xs, ys, zs, 'FaceColor', 'black', 'EdgeColor', 'none', 'FaceAlpha', 0.5); %plot3(xs, ys, zs, 'k');  % spherical obstacle bounndary points
        plotHandles.(subplot_handle).influence_field_1 = surf(x1_bound, y1_bound, z1_bound, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.1); % infulence sphere 1
        plotHandles.(subplot_handle).influence_field_2 = surf(x2_bound, y2_bound, z2_bound, 'FaceColor', 'black', 'EdgeColor', 'none', 'FaceAlpha', 0.1); % infulence sphere 2
        plotHandles.(subplot_handle).surface_plane = surf(xp, yp, zp, 'FaceColor', 'yellow', 'FaceAlpha', 0.5); % base plane surface
        plotHandles.(subplot_handle).trajectory = plot3(NaN, NaN, NaN, 'LineWidth', 1.5, 'Color', 'm');

        for m=1:4
            plotHandles.(subplot_handle).trajectory_closest_point{m} = plot3(NaN, NaN, NaN, 'LineWidth', 1.5, 'Color', [0.6 0.6 0.6]);
        end

        xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
        title(append('View ',string(i)));
        view(azimuth(i),elevation_angles(i)); % View from the x-axis
        %grid on;
        axis([plot_axis(i,1) plot_axis(i,2) plot_axis(i,3) plot_axis(i,4) plot_axis(i,5) plot_axis(i,6)])
        axis square

    end
end


