function  [norm_tau,norm_F] = Repulsive_Torque(D,dD_dx,O_m,Influence_radius,alpha,a,d,theta,eta)
    norm_tau = [0;0;0;0]; % if obstacle is far from object apply zero torque
    norm_F = [0,0,0,0;0,0,0,0;0,0,0,0];
    if any(D <= Influence_radius) % check if any of the close point is within Influence sphere
        idx = [find(D <= Influence_radius)];  % store all close point (within Influence sphere)

        for  w=1:length(idx) % If the point is too close apply force on that point
            if length(idx) > 1 % If multiple point on the manipulator are close to the obstacle 
                F(:,idx(w)) = eta*(1/Influence_radius - 1/D(idx(w)))*(1/(D(idx(w))^2))*dD_dx(:,idx(w));
                [~,J_tau,~] = Jacobian_of_Critical_Point(idx(w),alpha,a,d,theta,O_m(:,idx(w)));
                tau(:,idx(w)) = J_tau' * F(:,idx(w));
                
                norm_F(:,idx(w)) = F(:,idx(w))/norm(F(:,idx(w)));

            else  % If only one point on the manipulator is close to the obstacle 
                F(:,idx) = eta*(1/Influence_radius - 1/D(idx))*(1/(D(idx)^2))*dD_dx(:,idx);
                [~,J_tau,~] = Jacobian_of_Critical_Point(idx,alpha,a,d,theta,O_m(:,idx));
                tau(:,idx) = J_tau' * F(:,idx);

                norm_F(:,idx(w)) = F(:,idx(w))/norm(F(:,idx(w)));
            end
        end

        norm_tau = sum(tau,2) / norm(sum(tau,2));

    end
end