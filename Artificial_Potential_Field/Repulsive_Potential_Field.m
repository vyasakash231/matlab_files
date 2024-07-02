function [D,dD_dx,Set_of_Points_of_force] = Repulsive_Potential_Field(radius, obstacle, X_cord, Y_cord, Z_cord)
    % vector of our line (here we have neglected the base coord [0,0,0])
    p1 = [X_cord(1),X_cord(3),X_cord(4),X_cord(5); 
          Y_cord(1),Y_cord(3),Y_cord(4),Y_cord(5); 
          Z_cord(1),Z_cord(3),Z_cord(4),Z_cord(5)];  % starting points [O_0,O_2,O_3,O_4]
    
    p2 = [X_cord(2),X_cord(4),X_cord(5),X_cord(6); 
          Y_cord(2),Y_cord(4),Y_cord(5),Y_cord(6); 
          Z_cord(2),Z_cord(4),Z_cord(5),Z_cord(6)];  % end points [O_1,O_3,O_4,O_5]
    
    vec_line = p2 - p1;  % vector btw revolute joints, (X_i+1 - X_i)
    [~,no_of_moving_links] = size(vec_line);
    
    vec_ob_line = obstacle - p1;  % vector btw obstacle and joint, (X_o - X_i)
    
    % calculate the projection normalized by length of arm segment
    %closest_dist = 100;
    for i = 1:no_of_moving_links
        projection = (dot(vec_ob_line(:,i), vec_line(:,i)) / dot(vec_line(:,i), vec_line(:,i)));  % {(X_i+1 - X_i).(X_o - X_i)} / {||X_i+1 - X_i||*||X_i+1 - X_i||}

        if projection < 0               
            % then closest point is the start of the segment
            closest = p1(:,i);  % X_m = X_i
        elseif projection > 1
            % then closest point is the end of the segment
            closest = p2(:,i);  % X_m = X_i+1
        else
            closest = p1(:,i) + projection * vec_line(:,i);  % X_m = X_i + {(X_i+1 - X_i).(X_o - X_i)} / {|X_i+1 - X_i|*|X_i+1 - X_i|} * (X_i+1 - X_i)
        end
        
        % sqrt(sum((obstacle-closest).^2)) --> this is the distance of the closest point on each link from obstacle centre
        % but we have to also consider the radius of the sphere
        D(i) = sqrt(sum((obstacle - closest).^2)) - radius;  % distance difference btw ||X_o - X_m|| - r  
        dD_dx(:,i) = (obstacle - closest) / D(i);
        Set_of_Points_of_force(:,i) = closest; % coordinates of X_m of each link
    end
end