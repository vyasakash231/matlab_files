%function  [Joint_velocity] = Link_Velocity_to_Joint_Velocity(D,O_m,Influence_radius,Links_velocity,alpha,a,d,theta)
%    Joint_velocity = zeros(length(theta),length(theta));
%
%    idx = [find(D <= 2*Influence_radius)];  % store all close point (within Influence sphere)
%
%    if length(idx) > 1 % If multiple point on the manipulator are close to the obstacle 
%        for  w=1:length(idx) % If the point is too close apply force on that point
%            [~,J_tau,~] = Jacobian_of_Critical_Point(idx(w),alpha,a,d,theta,O_m(:,idx(w)));
%            Joint_velocity(1:idx(w),idx(w)) = pinv(J_tau(:,1:idx(w))) * Links_velocity(:,idx(w));  
%        end
%
%    else  % If only one point on the manipulator is close to the obstacle 
%        [~,J_tau,~] = Jacobian_of_Critical_Point(idx,alpha,a,d,theta,O_m(:,idx));
%        Joint_velocity(1:idx,idx) = pinv(J_tau(:,1:idx)) * Links_velocity(:,idx);  %#ok<*BDSCI>
%    end
%
%end

function  [Joint_velocity] = Link_Velocity_to_Joint_Velocity(idx,O_m,Links_velocity,alpha,a,d,theta)
    Joint_velocity = zeros(length(theta),1);

    [~,J_tau,~] = Jacobian_of_Critical_Point(idx, alpha, a, d, theta, O_m);
    Joint_velocity(1:idx, 1) = pinv(J_tau(:,1:idx)) * Links_velocity;

end