function [X_cord,Y_cord,Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le)
    % Transformation matrix
    [T_new,~,O] = Transformation_matrix(n,alpha,a,d,theta); % all angles in degrees
     
    T_final = T_new;
    d_nn = [Le;0;0];
    P_00_Homo = T_final*[d_nn;1];
    P_00 = P_00_Homo(1:3);        % convert P_00_homogeneous to P_00_cartesian
    
    % Plot
    X_cord = [0,O(1,1:n),P_00(1)];   % X-coordinate of each link
    Y_cord = [0,O(2,1:n),P_00(2)];   % Y-coordinate of each link
    Z_cord = [0,O(3,1:n),P_00(3)];   % Z-coordinate of each link
end