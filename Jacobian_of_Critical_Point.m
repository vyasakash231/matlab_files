%% Assuming all joint are revolute
function [J,Jv,Jw] = Jacobian_of_Critical_Point(link_no,alpha,a,d,theta,Point_at_which_force_will_act)
    % Calculate Tranformation matrix
    [~,R,O] = Transformation_matrix(link_no,alpha,a,d,theta); % All angles in degree [R(4x3x3), O(3,4)]
    
    %R_n_0 = R(:,:,link_no);
    %O_n_0 = O(:,link_no);
    X_m = Point_at_which_force_will_act;  % coordinates of O_m wrt world frame

    Jv = zeros(3,4);
    
    for i = 1:link_no
        Z_i_0 = R(:,3,i);
        O_i_0 = O(:,i);
    
        Jv(:,i) = cross(Z_i_0,(X_m - O_i_0));
        Jw(:,i) = Z_i_0;
        J(:,i) = [Jv(:,i); Jw(:,i)];
    end
end
