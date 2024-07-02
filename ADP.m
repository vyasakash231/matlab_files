%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

X_g = -0.2; Y_g = -0.175; Z_g = 0.04;

n = 4; % No of Joint
p = 0.128; q = 0.024;

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = [0,79.38,-79.38,0]; % In degree
Le = 0.126; % End-effector length

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(1,:) = X_cord; 
Y(1,:) = Y_cord;
Z(1,:) = Z_cord;

theta_range = [-180 180;-117 90;-90 87.5;-103 114.5]; % range of motion of each joint

R = diag([0.1,0.1,0.2,0.2]);
Q = diag([1,1,1]);

% Neural Network initialization
l = 5; % Number of neurons
Wc = 0.01*rand(l,1); % Initial critic weights

control_input = [0;0;0;0];

i = 1; dX = 1;
while norm(dX) > 0.001
    % End-effector coord
    X_e = X(i,end);  Y_e = Y(i,end);  Z_e = Z(i,end);

    dX = [(X_e - X_g); (Y_e - Y_g); (Z_e - Z_g)];
    del_X(:,i) = dX;  % for plotting
    
    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));

    % Optimal control law
    W = dX * ones(1,15);
    layer = dX' * W;
    phi = 1 ./ (1 + layer);
    
    for j=1:3
        for k=1:l
            del_phi(k,j) = W(j,k) * phi(1,k) * (1 - phi(1,k));  % 1 x 3
        end
    end

    d_theta = -0.5 * (1-exp(-i*0.05)) * inv(R) * Jv' * (del_phi' * Wc); % Implement control law

    control_input(:,i+1) = deg2rad(d_theta);  % In radians for ploting
    
    % Solving using Euler-forward method
    theta(i+1,:) = theta(i,:) + d_theta'; % In degrees

    lyapunov_grad = dX; % lyapunov func: (e * e) / 2

    A = del_phi * Jv * inv(R) * Jv' * del_phi';  % (1x3).(3x4).(4x4).(4x3).(3x1)
    ec = -0.25 * Wc' * A * Wc + dX' * Q * dX;
    d_ec = -0.5 * A * Wc;
    
    if i == 1
        e_dot = del_X(:,i);
    else
        e_dot = (del_X(:,i) - del_X(:,i-1));
    end
        
    lr = (norm(ec) / norm(d_ec*ec)) * (1 + dX' * e_dot / (norm(dX)));

    % Critic weight update
    Wc = Wc + 0.005 * lr * del_phi * Jv * inv(R) * Jv' * lyapunov_grad; % Weight update rule
    
    % Next Step performed
    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta(i+1,:),Le);

    X(i+1,:) = X_cord;  Y(i+1,:) = Y_cord;  Z(i+1,:) = Z_cord; % for plotting

    i = i + 1;
    if i > 2500
        break
    end
end

figure
for k = 1:i-500
    plot3(X(1,6),Y(1,6),Z(1,6),"*g",'LineWidth',8)
    hold on
    plot3(X_g,Y_g,Z_g,"*r",'LineWidth',8)
    hold on
    plot3(X(k,1:2),Y(k,1:2),Z(k,1:2),'-','LineWidth',8);
    hold on
    plot3(X(k,2:3),Y(k,2:3),Z(k,2:3),'-','LineWidth',7);
    hold on
    plot3(X(k,3:4),Y(k,3:4),Z(k,3:4),'-','LineWidth',6);
    hold on
    plot3(X(k,4:5),Y(k,4:5),Z(k,4:5),'-','LineWidth',5);
    hold on
    plot3(X(k,5:6),Y(k,5:6),Z(k,5:6),'-','LineWidth',4);
    hold on
    plot3(X(1:k,end),Y(1:k,end),Z(1:k,end),LineWidth=2,Color='m')

    axis([-0.28, 0.28, -0.28, 0.28, 0, 0.56])
    axis square
    view(240,15);
    grid on
    xlabel('X-axis','interpreter','latex',FontSize=15); 
    ylabel('Y-axis','interpreter','latex',FontSize=15); 
    zlabel('Z-axis','interpreter','latex',FontSize=15);
    drawnow
    hold off
end

figure
subplot(1,2,1)
plot(1:size(del_X,2),del_X(1:3,:),'--','LineWidth',2)
hold on 
plot(1:size(del_X,2),rms(del_X(1:3,:)),'LineWidth',2)
xlabel('steps')
ylabel('$Error$','interpreter','latex',FontSize=15)
legend('e_{x}','e_{y}','e_{z}','RMS Error')
grid on
grid minor

subplot(1,2,2)
plot(1:size(control_input,2),control_input(:,1:i),'LineWidth',2)
hold on 
legend('\theta_{1}','\theta_{2}','\theta_{3}', '\theta_{1}')
xlabel('steps')
ylabel('$Joint Velocity (rad/s)$','interpreter','latex',FontSize=15)
grid on
grid minor

% figure
% subplot(2,2,1)
% plot(1:size(theta,1),theta(:,1),'LineWidth',2)
% hold on
% plot(1:size(theta,1),-180*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:size(theta,1),180*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{1}')
% grid on
% grid minor

% subplot(2,2,2)
% plot(1:size(theta,1),theta(:,2),'LineWidth',2)
% hold on
% plot(1:size(theta,1),-117*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:size(theta,1),90*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{2}')
% grid on
% grid minor

% subplot(2,2,3)
% plot(1:size(theta,1),theta(:,3),'LineWidth',2)
% hold on
% plot(1:size(theta,1),-90*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:size(theta,1),87.5*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{3}')
% grid on
% grid minor

% subplot(2,2,4)
% plot(1:size(theta,1),theta(:,4),'LineWidth',2)
% hold on
% plot(1:size(theta,1),-103*ones(1,i),'--','LineWidth',2)
% hold on
% plot(1:size(theta,1),114.5*ones(1,i),'--','LineWidth',2)
% hold off
% xlabel('No of Iteration')
% ylabel('\theta_{4}')
% grid on
% grid minor
