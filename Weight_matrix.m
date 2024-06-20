%% All angles in Radians
function [We,Wc,Wv] = Weight_matrix(m,n,q_range,epsilon,q)
We = zeros(m,m);
for j = 1:m
    We(j,j) = 60;
end

Wc = zeros(n,n);
w0 = 300;
    for i = 1:n
        if q(i) < q_range(i,1)
            Wc(i,i) = w0;
        elseif q_range(i,1) <= q(i) && q(i) <= (q_range(i,1) + epsilon(i))
            Wc(i,i) = (w0/2)*(1 + cos(pi*(q(i) - q_range(i,1))/epsilon(i)));
        elseif (q_range(i,1) + epsilon(i)) < q(i) && q(i) < (q_range(i,2) - epsilon(i))
            Wc(i,i) = 0;
        elseif (q_range(i,2) - epsilon(i)) <= q(i) && q(i) <= q_range(i,2)
            Wc(i,i) = (w0/2)*(1 + cos(pi*(q_range(i,2) - q(i))/epsilon(i)));
        elseif q(i) > q_range(i,2)
            Wc(i,i) = w0;
        end
    end

Wv = zeros(n,n);
for j = 1:n
    Wv(j,j) = 0.25;
end
end