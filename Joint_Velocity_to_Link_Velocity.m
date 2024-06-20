function  [critical_point_velocity, critical_point] = Joint_Velocity_to_Link_Velocity(D,O_m,Influence_radius,joint_velocity,alpha,a,d,theta)
    critical_point_velocity = zeros(3,length(theta));
    critical_point = zeros(3,length(theta));

    idx = [find(D <= 2*Influence_radius)];  % store all close point (within Influence sphere)

    if length(idx) > 1 % If multiple point on the manipulator are close to the obstacle 
        for w=1:length(idx) % If the point is too close apply force on that point
            [~,J_tau,~] = Jacobian_of_Critical_Point(idx(w),alpha,a,d,theta,O_m(:,idx(w)));  % always (3,n) 
            critical_point_velocity(:,idx(w)) = J_tau(:,1:idx(w)) * joint_velocity(1:idx(w),1);  % (3,m)
            critical_point(:,idx(w)) = O_m(:,idx(w));  % (3,m)
        end

    else  % If only one point on the manipulator is close to the obstacle 
        [~,J_tau,~] = Jacobian_of_Critical_Point(idx,alpha,a,d,theta,O_m(:,idx));
        critical_point_velocity(:,idx) = J_tau(:,1:idx) * joint_velocity(1:idx,1);  %#ok<*BDSCI> % (3,m)
        critical_point(:,idx) = O_m(:,idx);  % (3,1)
    end
end