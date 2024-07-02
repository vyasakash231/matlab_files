%function [] = updatePlots(plotHandles, X, Y, Z, k, closest_point, X_g, Y_g, Z_g, obstacle, varargin) 
%    P = inputParser;
%    
%   % Optional Input
%    P.addOptional('A_unit', [], @isnumeric); 
%    P.addOptional('B_unit', [], @isnumeric); 
%    P.addOptional('C_unit', [], @isnumeric); 
%    P.addOptional('n_vec', [], @isnumeric); 
%    P.addOptional('const', [], @isnumeric); 
%    P.addOptional('new_X_g', [], @isnumeric); 
%    
%    P.addOptional('critic_vel_norm', [], @isnumeric);
%    P.addOptional('coord_of_critical_vel', [], @isnumeric);
%    P.addOptional('unit_vec_links_to_obstacle', [], @isnumeric);
%    P.addOptional('unit_vec_of_links', [], @isnumeric);
%    P.addOptional('new_critical_vel_norm', [], @isnumeric);
%
%    P.parse(varargin{:})
%
%    % Assign value from "inputParser" 
%    const = P.Results.const;
%    A_unit = P.Results.A_unit; 
%    B_unit = P.Results.B_unit; 
%    C_unit = P.Results.C_unit; 
%    n_vec = P.Results.n_vec;
%    new_X_g = P.Results.new_X_g;
%    critic_vel_norm = P.Results.critic_vel_norm;
%    coord_of_critical_vel = P.Results.coord_of_critical_vel;
%    unit_vec_links_to_obstacle = P.Results.unit_vec_links_to_obstacle;
%    unit_vec_of_links = P.Results.unit_vec_of_links;
%    new_critical_vel_norm = P.Results.new_critical_vel_norm;
%
%% ========================================================================================================== Ploting Section ======================================================================================================================== %
%
%    % Subplot 1: View from X-axis
%    set(plotHandles.h1_line1, 'XData', X(k,1:2), 'YData', Y(k,1:2), 'ZData', Z(k,1:2));
%    set(plotHandles.h1_line2, 'XData', X(k,2:3), 'YData', Y(k,2:3), 'ZData', Z(k,2:3));
%    set(plotHandles.h1_line3, 'XData', X(k,3:4), 'YData', Y(k,3:4), 'ZData', Z(k,3:4));
%    set(plotHandles.h1_line4, 'XData', X(k,4:5), 'YData', Y(k,4:5), 'ZData', Z(k,4:5));
%    set(plotHandles.h1_line5, 'XData', X(k,5:6), 'YData', Y(k,5:6), 'ZData', Z(k,5:6));
%    set(plotHandles.h1_closest_point, 'XData', closest_point(1,:,k), 'YData', closest_point(2,:,k), 'ZData', closest_point(3,:,k));
%    set(plotHandles.h1_trajectory, 'XData', X(1:k,6), 'YData', Y(1:k,6), 'ZData', Z(1:k,6));
%    %set(plotHandles.h1_trajectory_closest_point_3, 'XData', closest_point(1,3,1:k), 'YData', closest_point(2,3,1:k), 'ZData', closest_point(3,3,1:k));    
%    %set(plotHandles.h1_trajectory_closest_point_4, 'XData', closest_point(1,4,1:k), 'YData', closest_point(2,4,1:k), 'ZData', closest_point(3,4,1:k));   
%
%    % ==================================================== for goal shifting ========================================================= %
%    try
%        set(plotHandles.h1_unit_vec_ee_to_obstacle, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*A_unit(1,k), 'VData', const*A_unit(2,k), 'WData', const*A_unit(3,k));
%        set(plotHandles.h1_unit_vec_ee_to_goal, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*B_unit(1,k), 'VData', const*B_unit(2,k), 'WData', const*B_unit(3,k));
%        set(plotHandles.h1_plane, 'XData', [X_g, X(k,6), obstacle(1)], 'YData', [Y_g, Y(k,6), obstacle(2)], 'ZData', [Z_g, Z(k,6), obstacle(3)]);
%        set(plotHandles.h1_perpendicular_vector, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*n_vec(1,k), 'VData', const*n_vec(2,k), 'WData', const*n_vec(3,k));
%    catch
%    end
%
%    try
%        set(plotHandles.h1_moving_goal, 'XData',new_X_g(1,k), 'YData', new_X_g(2,k), 'ZData', new_X_g(3,k)); % new moving goal state
%        set(plotHandles.h1_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*C_unit(1,k), 'VData', const*C_unit(2,k), 'WData', const*C_unit(3,k));
%    catch
%        set(plotHandles.h1_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', 0, 'VData', 0, 'WData', 0);
%    end
%
%    % =================================================== for velocity rotation ====================================================== %
%    if ~isempty(critic_vel_norm)
%        try
%            set(plotHandles.h1_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*critic_vel_norm(1,1,k), 'VData', const*critic_vel_norm(2,1,k), 'WData', const*critic_vel_norm(3,1,k));
%            if all(critic_vel_norm(:,1,k) ~= 0) 
%                set(plotHandles.h1_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_links_to_obstacle(1,1,k), 'VData', const*unit_vec_links_to_obstacle(2,1,k), 'WData', const*unit_vec_links_to_obstacle(3,1,k));
%                set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_of_links(1,1,k), 'VData', const*unit_vec_of_links(2,1,k), 'WData', const*unit_vec_of_links(3,1,k));
%                set(plotHandles.h1_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*new_critical_vel_norm(1,1,k), 'VData', const*new_critical_vel_norm(2,1,k), 'WData', const*new_critical_vel_norm(3,1,k));
%            end
%        catch
%            set(plotHandles.h1_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%
%        end
%
%        try
%            set(plotHandles.h1_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*critic_vel_norm(1,2,k), 'VData', const*critic_vel_norm(2,2,k), 'WData', const*critic_vel_norm(3,2,k));
%            if all(critic_vel_norm(:,2,k) ~= 0) 
%                set(plotHandles.h1_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_links_to_obstacle(1,2,k), 'VData', const*unit_vec_links_to_obstacle(2,2,k), 'WData', const*unit_vec_links_to_obstacle(3,2,k));
%                set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_of_links(1,2,k), 'VData', const*unit_vec_of_links(2,2,k), 'WData', const*unit_vec_of_links(3,2,k));
%                set(plotHandles.h1_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*new_critical_vel_norm(1,2,k), 'VData', const*new_critical_vel_norm(2,2,k), 'WData', const*new_critical_vel_norm(3,2,k));
%            end
%        catch
%            set(plotHandles.h1_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h1_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*critic_vel_norm(1,3,k), 'VData', const*critic_vel_norm(2,3,k), 'WData', const*critic_vel_norm(3,3,k));
%            if all(critic_vel_norm(:,3,k) ~= 0) 
%                set(plotHandles.h1_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_links_to_obstacle(1,3,k), 'VData', const*unit_vec_links_to_obstacle(2,3,k), 'WData', const*unit_vec_links_to_obstacle(3,3,k));
%                set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_of_links(1,3,k), 'VData', const*unit_vec_of_links(2,3,k), 'WData', const*unit_vec_of_links(3,3,k));
%                set(plotHandles.h1_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*new_critical_vel_norm(1,3,k), 'VData', const*new_critical_vel_norm(2,3,k), 'WData', const*new_critical_vel_norm(3,3,k));
%            end
%        catch
%            set(plotHandles.h1_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h1_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*critic_vel_norm(1,4,k), 'VData', const*critic_vel_norm(2,4,k), 'WData', const*critic_vel_norm(3,4,k));
%            if all(critic_vel_norm(:,4,k) ~= 0) 
%                set(plotHandles.h1_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_links_to_obstacle(1,4,k), 'VData', const*unit_vec_links_to_obstacle(2,4,k), 'WData', const*unit_vec_links_to_obstacle(3,4,k));
%                set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_of_links(1,4,k), 'VData', const*unit_vec_of_links(2,4,k), 'WData', const*unit_vec_of_links(3,4,k));
%                set(plotHandles.h1_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*new_critical_vel_norm(1,4,k), 'VData', const*new_critical_vel_norm(2,4,k), 'WData', const*new_critical_vel_norm(3,4,k));
%            end
%        catch
%            set(plotHandles.h1_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h1_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%    end
%
%% ======================================================================================================================================================================================================================================================== %
%
%    % Subplot 2: View from Y-axis
%    set(plotHandles.h2_line1, 'XData', X(k,1:2), 'YData', Y(k,1:2), 'ZData', Z(k,1:2));
%    set(plotHandles.h2_line2, 'XData', X(k,2:3), 'YData', Y(k,2:3), 'ZData', Z(k,2:3));
%    set(plotHandles.h2_line3, 'XData', X(k,3:4), 'YData', Y(k,3:4), 'ZData', Z(k,3:4));
%    set(plotHandles.h2_line4, 'XData', X(k,4:5), 'YData', Y(k,4:5), 'ZData', Z(k,4:5));
%    set(plotHandles.h2_line5, 'XData', X(k,5:6), 'YData', Y(k,5:6), 'ZData', Z(k,5:6));
%    set(plotHandles.h2_closest_point, 'XData', closest_point(1,:,k), 'YData', closest_point(2,:,k), 'ZData', closest_point(3,:,k));
%    set(plotHandles.h2_trajectory, 'XData', X(1:k,6), 'YData', Y(1:k,6), 'ZData', Z(1:k,6));
%    %set(plotHandles.h2_trajectory_closest_point_3, 'XData', closest_point(1,3,1:k), 'YData', closest_point(2,3,1:k), 'ZData', closest_point(3,3,1:k));    
%    %set(plotHandles.h2_trajectory_closest_point_4, 'XData', closest_point(1,4,1:k), 'YData', closest_point(2,4,1:k), 'ZData', closest_point(3,4,1:k));    
%
%    % ==================================================== for goal shifting ========================================================= %
%    try
%        set(plotHandles.h2_unit_vec_ee_to_obstacle, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*A_unit(1,k), 'VData', const*A_unit(2,k), 'WData', const*A_unit(3,k));
%        set(plotHandles.h2_unit_vec_ee_to_goal, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*B_unit(1,k), 'VData', const*B_unit(2,k), 'WData', const*B_unit(3,k));
%        set(plotHandles.h2_plane, 'XData', [X_g, X(k,6), obstacle(1)], 'YData', [Y_g, Y(k,6), obstacle(2)], 'ZData', [Z_g, Z(k,6), obstacle(3)]);
%        set(plotHandles.h2_perpendicular_vector, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*n_vec(1,k), 'VData', const*n_vec(2,k), 'WData', const*n_vec(3,k));
%    catch
%    end
%
%    try
%        set(plotHandles.h2_moving_goal, 'XData',new_X_g(1,k), 'YData', new_X_g(2,k), 'ZData', new_X_g(3,k)); % new moving goal state
%        set(plotHandles.h2_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*C_unit(1,k), 'VData', const*C_unit(2,k), 'WData', const*C_unit(3,k));
%    catch
%        set(plotHandles.h2_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', 0, 'VData', 0, 'WData', 0);
%    end
%
%    % =================================================== for velocity rotation ====================================================== %
%    if ~isempty(critic_vel_norm)
%        try
%            set(plotHandles.h2_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*critic_vel_norm(1,1,k), 'VData', const*critic_vel_norm(2,1,k), 'WData', const*critic_vel_norm(3,1,k));
%            if all(critic_vel_norm(:,1,k) ~= 0) 
%                set(plotHandles.h2_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_links_to_obstacle(1,1,k), 'VData', const*unit_vec_links_to_obstacle(2,1,k), 'WData', const*unit_vec_links_to_obstacle(3,1,k));
%                set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_of_links(1,1,k), 'VData', const*unit_vec_of_links(2,1,k), 'WData', const*unit_vec_of_links(3,1,k));
%                set(plotHandles.h2_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*new_critical_vel_norm(1,1,k), 'VData', const*new_critical_vel_norm(2,1,k), 'WData', const*new_critical_vel_norm(3,1,k));
%            end
%        catch
%            set(plotHandles.h2_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h2_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*critic_vel_norm(1,2,k), 'VData', const*critic_vel_norm(2,2,k), 'WData', const*critic_vel_norm(3,2,k));
%            if all(critic_vel_norm(:,2,k) ~= 0) 
%                set(plotHandles.h2_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_links_to_obstacle(1,2,k), 'VData', const*unit_vec_links_to_obstacle(2,2,k), 'WData', const*unit_vec_links_to_obstacle(3,2,k));
%                set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_of_links(1,2,k), 'VData', const*unit_vec_of_links(2,2,k), 'WData', const*unit_vec_of_links(3,2,k));
%                set(plotHandles.h2_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*new_critical_vel_norm(1,2,k), 'VData', const*new_critical_vel_norm(2,2,k), 'WData', const*new_critical_vel_norm(3,2,k));
%            end
%        catch
%            set(plotHandles.h2_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h2_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*critic_vel_norm(1,3,k), 'VData', const*critic_vel_norm(2,3,k), 'WData', const*critic_vel_norm(3,3,k));
%            if all(critic_vel_norm(:,3,k) ~= 0) 
%                set(plotHandles.h2_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_links_to_obstacle(1,3,k), 'VData', const*unit_vec_links_to_obstacle(2,3,k), 'WData', const*unit_vec_links_to_obstacle(3,3,k));
%                set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_of_links(1,3,k), 'VData', const*unit_vec_of_links(2,3,k), 'WData', const*unit_vec_of_links(3,3,k));
%                set(plotHandles.h2_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*new_critical_vel_norm(1,3,k), 'VData', const*new_critical_vel_norm(2,3,k), 'WData', const*new_critical_vel_norm(3,3,k));
%            end
%        catch
%            set(plotHandles.h2_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h2_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*critic_vel_norm(1,4,k), 'VData', const*critic_vel_norm(2,4,k), 'WData', const*critic_vel_norm(3,4,k));
%            if all(critic_vel_norm(:,4,k) ~= 0) 
%                set(plotHandles.h2_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_links_to_obstacle(1,4,k), 'VData', const*unit_vec_links_to_obstacle(2,4,k), 'WData', const*unit_vec_links_to_obstacle(3,4,k));
%                set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_of_links(1,4,k), 'VData', const*unit_vec_of_links(2,4,k), 'WData', const*unit_vec_of_links(3,4,k));
%                set(plotHandles.h2_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*new_critical_vel_norm(1,4,k), 'VData', const*new_critical_vel_norm(2,4,k), 'WData', const*new_critical_vel_norm(3,4,k));
%            end
%        catch
%            set(plotHandles.h2_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h2_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%    end
%
%% ======================================================================================================================================================================================================================================================== %
%
%    % Subplot 3: View from Z-axis
%    set(plotHandles.h3_line1, 'XData', X(k,1:2), 'YData', Y(k,1:2), 'ZData', Z(k,1:2));
%    set(plotHandles.h3_line2, 'XData', X(k,2:3), 'YData', Y(k,2:3), 'ZData', Z(k,2:3));
%    set(plotHandles.h3_line3, 'XData', X(k,3:4), 'YData', Y(k,3:4), 'ZData', Z(k,3:4));
%    set(plotHandles.h3_line4, 'XData', X(k,4:5), 'YData', Y(k,4:5), 'ZData', Z(k,4:5));
%    set(plotHandles.h3_line5, 'XData', X(k,5:6), 'YData', Y(k,5:6), 'ZData', Z(k,5:6));
%    set(plotHandles.h3_closest_point, 'XData', closest_point(1,:,k), 'YData', closest_point(2,:,k), 'ZData', closest_point(3,:,k));
%    set(plotHandles.h3_trajectory, 'XData', X(1:k,6), 'YData', Y(1:k,6), 'ZData', Z(1:k,6));
%    %set(plotHandles.h3_trajectory_closest_point_3, 'XData', closest_point(1,3,1:k), 'YData', closest_point(2,3,1:k), 'ZData', closest_point(3,3,1:k));    
%    %set(plotHandles.h3_trajectory_closest_point_4, 'XData', closest_point(1,4,1:k), 'YData', closest_point(2,4,1:k), 'ZData', closest_point(3,4,1:k));    
%
%    % ==================================================== for goal shifting ========================================================= %
%    try
%        set(plotHandles.h3_unit_vec_ee_to_obstacle, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*A_unit(1,k), 'VData', const*A_unit(2,k), 'WData', const*A_unit(3,k));
%        set(plotHandles.h3_unit_vec_ee_to_goal, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*B_unit(1,k), 'VData', const*B_unit(2,k), 'WData', const*B_unit(3,k));
%        set(plotHandles.h3_plane, 'XData', [X_g, X(k,6), obstacle(1)], 'YData', [Y_g, Y(k,6), obstacle(2)], 'ZData', [Z_g, Z(k,6), obstacle(3)]);
%        set(plotHandles.h3_perpendicular_vector, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*n_vec(1,k), 'VData', const*n_vec(2,k), 'WData', const*n_vec(3,k));
%    catch
%    end
%
%    try
%        set(plotHandles.h3_moving_goal, 'XData',new_X_g(1,k), 'YData', new_X_g(2,k), 'ZData', new_X_g(3,k)); % new moving goal state
%        set(plotHandles.h3_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*C_unit(1,k), 'VData', const*C_unit(2,k), 'WData', const*C_unit(3,k));
%    catch
%        set(plotHandles.h3_unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', 0, 'VData', 0, 'WData', 0);
%    end
%
%    % =================================================== for velocity rotation ====================================================== %
%    if ~isempty(critic_vel_norm)
%        try
%            set(plotHandles.h3_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*critic_vel_norm(1,1,k), 'VData', const*critic_vel_norm(2,1,k), 'WData', const*critic_vel_norm(3,1,k));
%            if all(critic_vel_norm(:,1,k) ~= 0) 
%                set(plotHandles.h3_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_links_to_obstacle(1,1,k), 'VData', const*unit_vec_links_to_obstacle(2,1,k), 'WData', const*unit_vec_links_to_obstacle(3,1,k));
%                set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*unit_vec_of_links(1,1,k), 'VData', const*unit_vec_of_links(2,1,k), 'WData', const*unit_vec_of_links(3,1,k));
%                set(plotHandles.h3_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,k), 'YData', coord_of_critical_vel(2,1,k), 'ZData', coord_of_critical_vel(3,1,k), 'UData', const*new_critical_vel_norm(1,1,k), 'VData', const*new_critical_vel_norm(2,1,k), 'WData', const*new_critical_vel_norm(3,1,k));
%            end
%        catch
%            set(plotHandles.h3_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_closest_point_vel_1_to_obstacle, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_new_closest_point_vel_1, 'XData', coord_of_critical_vel(1,1,end), 'YData', coord_of_critical_vel(2,1,end), 'ZData', coord_of_critical_vel(3,1,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h3_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*critic_vel_norm(1,2,k), 'VData', const*critic_vel_norm(2,2,k), 'WData', const*critic_vel_norm(3,2,k));
%            if all(critic_vel_norm(:,2,k) ~= 0) 
%                set(plotHandles.h3_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_links_to_obstacle(1,2,k), 'VData', const*unit_vec_links_to_obstacle(2,2,k), 'WData', const*unit_vec_links_to_obstacle(3,2,k));
%                set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*unit_vec_of_links(1,2,k), 'VData', const*unit_vec_of_links(2,2,k), 'WData', const*unit_vec_of_links(3,2,k));
%                set(plotHandles.h3_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,k), 'YData', coord_of_critical_vel(2,2,k), 'ZData', coord_of_critical_vel(3,2,k), 'UData', const*new_critical_vel_norm(1,2,k), 'VData', const*new_critical_vel_norm(2,2,k), 'WData', const*new_critical_vel_norm(3,2,k));
%            end
%        catch
%            set(plotHandles.h3_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_closest_point_vel_2_to_obstacle, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_new_closest_point_vel_2, 'XData', coord_of_critical_vel(1,2,end), 'YData', coord_of_critical_vel(2,2,end), 'ZData', coord_of_critical_vel(3,2,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h3_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*critic_vel_norm(1,3,k), 'VData', const*critic_vel_norm(2,3,k), 'WData', const*critic_vel_norm(3,3,k));
%            if all(critic_vel_norm(:,3,k) ~= 0) 
%                set(plotHandles.h3_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_links_to_obstacle(1,3,k), 'VData', const*unit_vec_links_to_obstacle(2,3,k), 'WData', const*unit_vec_links_to_obstacle(3,3,k));
%                set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*unit_vec_of_links(1,3,k), 'VData', const*unit_vec_of_links(2,3,k), 'WData', const*unit_vec_of_links(3,3,k));
%                set(plotHandles.h3_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,k), 'YData', coord_of_critical_vel(2,3,k), 'ZData', coord_of_critical_vel(3,3,k), 'UData', const*new_critical_vel_norm(1,3,k), 'VData', const*new_critical_vel_norm(2,3,k), 'WData', const*new_critical_vel_norm(3,3,k));
%            end
%        catch
%            set(plotHandles.h3_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_closest_point_vel_3_to_obstacle, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_new_closest_point_vel_3, 'XData', coord_of_critical_vel(1,3,end), 'YData', coord_of_critical_vel(2,3,end), 'ZData', coord_of_critical_vel(3,3,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%
%        try
%            set(plotHandles.h3_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*critic_vel_norm(1,4,k), 'VData', const*critic_vel_norm(2,4,k), 'WData', const*critic_vel_norm(3,4,k));
%            if all(critic_vel_norm(:,4,k) ~= 0) 
%                set(plotHandles.h3_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_links_to_obstacle(1,4,k), 'VData', const*unit_vec_links_to_obstacle(2,4,k), 'WData', const*unit_vec_links_to_obstacle(3,4,k));
%                set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*unit_vec_of_links(1,4,k), 'VData', const*unit_vec_of_links(2,4,k), 'WData', const*unit_vec_of_links(3,4,k));
%                set(plotHandles.h3_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,k), 'YData', coord_of_critical_vel(2,4,k), 'ZData', coord_of_critical_vel(3,4,k), 'UData', const*new_critical_vel_norm(1,4,k), 'VData', const*new_critical_vel_norm(2,4,k), 'WData', const*new_critical_vel_norm(3,4,k));
%            end
%        catch
%            set(plotHandles.h3_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_closest_point_vel_4_to_obstacle, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_unit_vec_perp_to_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%            set(plotHandles.h3_new_closest_point_vel_4, 'XData', coord_of_critical_vel(1,4,end), 'YData', coord_of_critical_vel(2,4,end), 'ZData', coord_of_critical_vel(3,4,end), 'UData', 0, 'VData', 0, 'WData', 0);
%        end
%    end
%
%% ======================================================================================================================================================================================================================================================== %


function [] = updatePlots(plotHandles, X, Y, Z, k, closest_point, X_g, Y_g, Z_g, obstacle, varargin) 
    P = inputParser;
    
   % Optional Input
    P.addOptional('A_unit', [], @isnumeric); 
    P.addOptional('B_unit', [], @isnumeric); 
    P.addOptional('C_unit', [], @isnumeric); 
    P.addOptional('n_vec', [], @isnumeric); 
    P.addOptional('const', [], @isnumeric); 
    P.addOptional('new_X_g', [], @isnumeric); 
    
    P.addOptional('critic_vel_norm', [], @isnumeric);
    P.addOptional('coord_of_critical_vel', [], @isnumeric);
    P.addOptional('unit_vec_links_to_obstacle', [], @isnumeric);
    P.addOptional('unit_vec_of_links', [], @isnumeric);
    P.addOptional('new_critical_vel_norm', [], @isnumeric);

    P.parse(varargin{:})

    % Assign value from "inputParser" 
    const = P.Results.const;
    A_unit = P.Results.A_unit; 
    B_unit = P.Results.B_unit; 
    C_unit = P.Results.C_unit; 
    n_vec = P.Results.n_vec;
    new_X_g = P.Results.new_X_g;
    critic_vel_norm = P.Results.critic_vel_norm;
    coord_of_critical_vel = P.Results.coord_of_critical_vel;
    unit_vec_links_to_obstacle = P.Results.unit_vec_links_to_obstacle;
    unit_vec_of_links = P.Results.unit_vec_of_links;
    new_critical_vel_norm = P.Results.new_critical_vel_norm;

% ========================================================================================================== Ploting Section ======================================================================================================================== %

    subplots = {'h1', 'h2', 'h3'};

    for i=1:length(subplots)

        subplot_handle = subplots{i};

        % Plot links
        for j = 1:5
            set(plotHandles.(subplot_handle).lines{j}, 'XData', X(k,j:j+1), 'YData', Y(k,j:j+1), 'ZData', Z(k,j:j+1));
        end

        set(plotHandles.(subplot_handle).closest_point, 'XData', closest_point(1,:,k), 'YData', closest_point(2,:,k), 'ZData', closest_point(3,:,k));
        set(plotHandles.(subplot_handle).trajectory, 'XData', X(1:k,6), 'YData', Y(1:k,6), 'ZData', Z(1:k,6));

        % closest point trajectory
        %for l=1:4
        %    set(plotHandles.(subplot_handle).trajectory_closest_point{l}, 'XData', closest_point(1,l,1:k), 'YData', closest_point(2,l,1:k), 'ZData', closest_point(3,l,1:k));
        %end

        % ==================================================== for goal shifting ========================================================= %
        try
            set(plotHandles.(subplot_handle).unit_vec_ee_to_obstacle, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*A_unit(1,k), 'VData', const*A_unit(2,k), 'WData', const*A_unit(3,k));
            set(plotHandles.(subplot_handle).unit_vec_ee_to_goal, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*B_unit(1,k), 'VData', const*B_unit(2,k), 'WData', const*B_unit(3,k));
            set(plotHandles.(subplot_handle).plane, 'XData', [X_g, X(k,6), obstacle(1)], 'YData', [Y_g, Y(k,6), obstacle(2)], 'ZData', [Z_g, Z(k,6), obstacle(3)]);
            set(plotHandles.(subplot_handle).perpendicular_vector, 'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*n_vec(1,k), 'VData', const*n_vec(2,k), 'WData', const*n_vec(3,k));
        catch
        end

        try
            set(plotHandles.(subplot_handle).moving_goal, 'XData',new_X_g(1,k), 'YData', new_X_g(2,k), 'ZData', new_X_g(3,k)); % new moving goal state
            set(plotHandles.(subplot_handle).unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', const*C_unit(1,k), 'VData', const*C_unit(2,k), 'WData', const*C_unit(3,k));
        catch
            set(plotHandles.(subplot_handle).unit_vec_ee_to_moving_goal,  'XData', X(k,6), 'YData', Y(k,6), 'ZData', Z(k,6), 'UData', 0, 'VData', 0, 'WData', 0);
        end

        % =================================================== for velocity rotation ====================================================== %
        if ~isempty(critic_vel_norm)
            for f=1:4
                try
                    set(plotHandles.(subplot_handle).closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,k), 'YData', coord_of_critical_vel(2,f,k), 'ZData', coord_of_critical_vel(3,f,k), 'UData', const*critic_vel_norm(1,f,k), 'VData', const*critic_vel_norm(2,f,k), 'WData', const*critic_vel_norm(3,f,k));
                    if all(critic_vel_norm(:,f,k) ~= 0) 
                        set(plotHandles.(subplot_handle).unit_vec_closest_point_vel_to_obstacle{f}, 'XData', coord_of_critical_vel(1,f,k), 'YData', coord_of_critical_vel(2,f,k), 'ZData', coord_of_critical_vel(3,f,k), 'UData', const*unit_vec_links_to_obstacle(1,f,k), 'VData', const*unit_vec_links_to_obstacle(2,f,k), 'WData', const*unit_vec_links_to_obstacle(3,f,k));
                        set(plotHandles.(subplot_handle).unit_vec_perp_to_closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,k), 'YData', coord_of_critical_vel(2,f,k), 'ZData', coord_of_critical_vel(3,f,k), 'UData', const*unit_vec_of_links(1,f,k), 'VData', const*unit_vec_of_links(2,f,k), 'WData', const*unit_vec_of_links(3,f,k));
                        set(plotHandles.(subplot_handle).new_closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,k), 'YData', coord_of_critical_vel(2,f,k), 'ZData', coord_of_critical_vel(3,f,k), 'UData', const*new_critical_vel_norm(1,f,k), 'VData', const*new_critical_vel_norm(2,f,k), 'WData', const*new_critical_vel_norm(3,f,k));
                    end
                catch
                    set(plotHandles.(subplot_handle).closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,end), 'YData', coord_of_critical_vel(2,f,end), 'ZData', coord_of_critical_vel(3,f,end), 'UData', 0, 'VData', 0, 'WData', 0);
                    set(plotHandles.(subplot_handle).unit_vec_closest_point_vel_to_obstacle{f}, 'XData', coord_of_critical_vel(1,f,end), 'YData', coord_of_critical_vel(2,f,end), 'ZData', coord_of_critical_vel(3,f,end), 'UData', 0, 'VData', 0, 'WData', 0);
                    set(plotHandles.(subplot_handle).unit_vec_perp_to_closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,end), 'YData', coord_of_critical_vel(2,f,end), 'ZData', coord_of_critical_vel(3,f,end), 'UData', 0, 'VData', 0, 'WData', 0);
                    set(plotHandles.(subplot_handle).new_closest_point_vel{f}, 'XData', coord_of_critical_vel(1,f,end), 'YData', coord_of_critical_vel(2,f,end), 'ZData', coord_of_critical_vel(3,f,end), 'UData', 0, 'VData', 0, 'WData', 0);
                end
            end
        end
    end
end

% ======================================================================================================================================================================================================================================================== %
