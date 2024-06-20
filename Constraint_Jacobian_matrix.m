%% Assuming all joint are revolute
function Jc = Constraint_Jacobian_matrix(obstacle,D,Influence_radius,alpha,a,d,theta,critical_points)
    X_m = critical_points;  % (3 x No_of_moving_links), no of moving is generally equal to DOF
    X_o = obstacle;  % (3x1)
    
    [~,no_of_moving_links] = size(X_m); 

    for i=1:no_of_moving_links
        s = X_m(:,i) - X_o;
        norm_s = s / norm(s);  % Unit vector of s
        [~,Jx,~] = Jacobian_of_Critical_Point(i,alpha,a,d,theta,X_m(:,i));
        
        if D(i) <= 2*Influence_radius
            Jc(i,:) = norm_s' * Jx;
        else
            [R,C] = size(norm_s' * Jx);
            Jc(i,:) = zeros(R,C);
    end

end