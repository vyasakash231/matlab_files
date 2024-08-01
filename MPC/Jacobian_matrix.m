%% Assuming all joint are revolute
function [J,Jv,Jw] = Jacobian_matrix(n,alpha,a,d,theta)
    % Calculate Tranformation matrix
    [~,R,O] = Transformation_matrix(n,alpha,a,d,theta); % All angles in degree
    
    [~,C] = size(O);
    
    % En is 1 for revolute joint and 0 for prismatic joint
    Ei = ones(C);
    
    R_n_0 = R(:,:,n);
    O_n_0 = O(:,n);
    O_E_n = [0.126;0;0]; % coordinates of O_E wrt nth body frame
    O_E = O_n_0 + R_n_0 * O_E_n;  % coordinates of O_E wrt world frame
    
    for i = 1:n
        Z_i_0 = R(:,3,i);
        O_i_0 = O(:,i);
    
        Jv(:,i) = Ei(i)*cross(Z_i_0,(O_E - O_i_0)) + (1 - Ei(i))*Z_i_0;
        Jw(:,i) = Ei(i)*Z_i_0;
        J(:,i) = [Jv(:,i); Jw(:,i)];
    end
end
