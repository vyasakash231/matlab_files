%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

%addpath E:\Matlab_Files\Robotics\Lab_Assignment % To access function from other folders

X_g = 0.1; Y_g = -0.2; Z_g = 0.1; 

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

R = diag([1,1,1,1]);
Q = diag([100000,50000,50000]);

d_t = 0.005;
t = 0:d_t:10; 

control_input = [0;0;0;0];

% del_X = ones(3,1);
% while max(abs(del_X(1:3,i))) > 0.01
for i=1:length(t)
    X_c = X(i,n+2);
    Y_c = Y(i,n+2);
    Z_c = Z(i,n+2);
    
    dX = [(X_c - X_g); (Y_c - Y_g); (Z_c - Z_g)];
    del_X(:,i) = dX;  % for plotting
    
    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));

    phi = (Jv' * dX) / norm(Jv' * dX); % (4,1)

    d_theta = - (1-exp(-i*0.05)) * sqrt(dX' * Q * dX) * (inv(R).^0.5 * phi); 

    control_input(:,i+1) = d_theta * (pi/180);
    
    % Solving using Euler-forward method
    theta_new = theta(i,:) + d_theta' * d_t; % In degrees
    
    theta(i+1,:) = theta_new; % In degrees  (for plotting)
    
    % Next Step performed
    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta_new,Le);

    X(i+1,:) = X_cord; % for plotting
    Y(i+1,:) = Y_cord; % for plotting
    Z(i+1,:) = Z_cord; % for plotting

    if rms(dX) < 0.005
        break
    end
end

figure
for k = 1:i-250
    set(gca,'Projection','perspective')
    plot3(X(k,1:2),Y(k,1:2),Z(k,1:2),'-','LineWidth',4);
    hold on
    plot3(X(k,2:3),Y(k,2:3),Z(k,2:3),'-','LineWidth',4);
    hold on
    plot3(X(k,3:4),Y(k,3:4),Z(k,3:4),'-','LineWidth',3);
    hold on
    plot3(X(k,4:5),Y(k,4:5),Z(k,4:5),'-','LineWidth',3);
    hold on
    plot3(X(k,5:6),Y(k,5:6),Z(k,5:6),'-','LineWidth',2);
    hold on

    axis([-0.5 0.5 -0.5 0.5 0 0.5])
    %axis square
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    grid on
    hold on
    view(60,30);
    drawnow
    hold off
end

figure
subplot(1,2,1)
plot(t(1:i),del_X(1:3,:),'--','LineWidth',2)
hold on 
plot(t(1:i),rms(del_X(1:3,:)),'LineWidth',2)
xlabel('time (s)')
ylabel('Position error(mm)')
legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})','RMS Error')
grid on
grid minor

subplot(1,2,2)
plot(t(1:i),control_input(:,1:i),'LineWidth',2)
hold on 
xlabel('time (s)')
ylabel('Control Input (rad/s)')
legend('J_{1}','J_{2}','J_{3}','J_{4}')
grid on
grid minor

figure
subplot(2,2,1)
plot(1:i+1,theta(:,1),'LineWidth',2)
hold on
plot(1:i,-180*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,180*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{1}')
grid on
grid minor

subplot(2,2,2)
plot(1:i+1,theta(:,2),'LineWidth',2)
hold on
plot(1:i,-117*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,90*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{2}')
grid on
grid minor

subplot(2,2,3)
plot(1:i+1,theta(:,3),'LineWidth',2)
hold on
plot(1:i,-90*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,87.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{3}')
grid on
grid minor

subplot(2,2,4)
plot(1:i+1,theta(:,4),'LineWidth',2)
hold on
plot(1:i,-103*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{4}')
grid on
grid minor
