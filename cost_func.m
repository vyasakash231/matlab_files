function v = cost_func(n,K,q,q_range,m)
q_c = mean(q_range,2); % column vector containing the mean of each row
del_q = q_range(:,2) - q_range(:,1); % Total working range of each joint

for i = 1:n
    if q(i) >= q_c(i)
        a(i) = (K(i,i)*((q(i) - q_c(i))/del_q(i)))^m;
        b(i) = (K(i,i)*((q(i) - q_c(i))/del_q(i)))^(m-1);
    elseif q(i) < q_c(i)
        a(i) = (K(i,i)*((q_c(i) - q(i))/del_q(i)))^m;
        b(i) = (K(i,i)*((q_c(i) - q(i))/del_q(i)))^(m-1);
    end
end
L = sum(a);

for j = 1:n
    if q(j) >= q_c(j)
        del_phi_del_q(j) = L^((1-m)/m)*b(j)*(K(j,j)/del_q(j));
    elseif q(j) < q_c(j)
        del_phi_del_q(j) = -L^((1-m)/m)*b(j)*(K(j,j)/del_q(j));
    end
end
v = -del_phi_del_q;
end