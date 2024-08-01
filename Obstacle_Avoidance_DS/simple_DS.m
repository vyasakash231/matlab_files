clc; clear all; close all;

dt = 0.1;
T = 0:dt:100;

zeta_initial = -1;
for j=1:50
    Zeta_vec(:,1,j) = [0;zeta_initial];  % [zeta_1; zeta_2]
    zeta_initial = zeta_initial + 1;
end

for j=1:50
    for i=1:length(T)
        Zeta_dot_vec = [1;sin(Zeta_vec(1,i,j))];  % [zeta_dot_1; zeta_dot_2] = [1; sin(zeta_1)]
        Zeta_vec(:,i+1,j) = Zeta_vec(:,i,j) + Zeta_dot_vec * dt;
    end
end

figure;
for j=1:50
    plot(Zeta_vec(1,:,j), Zeta_vec(2,:,j), "LineWidth", 1.5, "Color", 'red')
    hold on
    axis([0 25 0 25])
    xlabel('\zeta_{1}','FontSize',15)
    ylabel('\zeta_{2}','FontSize',15)
end 
