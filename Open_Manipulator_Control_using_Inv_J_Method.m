%% Numerical Inverse Kinematics Algorithm
clc; clear all; close all;

%addpath E:\Matlab_Files\Robotics\Lab_Assignment % To access function from other folders

% X_g = input('Enter X-coordinate of goal position: ');
% Y_g = input('Enter X-coordinate of goal position: ');
% Z_g = input('Enter X-coordinate of goal position: ');
X_g = 0.17; Y_g = 0.13; Z_g = 0.1; 

n = 4; % No of Joint
p = 0.128; q = 0.024;

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = [0,79.38,-79.38,0]; % In degree
Le = 0.126; % End-effector length

i = 1;

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(i,:) = X_cord;
Y(i,:) = Y_cord;
Z(i,:) = Z_cord;

theta_range = [-180 180;-117 90;-90 87.5;-103 114.5]; % range of motion of each joint

del_X = ones(3,i); 
while max(abs(del_X(1:3,i))) > 0.01
    X_c = X(i,n+2); 
    Y_c = Y(i,n+2); 
    Z_c = Z(i,n+2);
    
    del_X(:,i+1) = [(X_g - X_c); (Y_g - Y_c); (Z_g - Z_c)];
    
    [J,~,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:));
    
    J_pseudo_inv = pinv(J(1:3,:));
    
    % Solving using Newton-Raphson Method
    theta_new = theta(i,:) + (J_pseudo_inv*del_X(:,i+1))'; % In degrees
    
    theta(i+1,:) = theta_new; % In degrees

    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta(i+1,:),Le);

    i = i + 1;

%     if theta_range(1,1) > theta(1) || theta(1) > theta_range(1,2)
%         disp(['Joint 1 Limit breached',theta(1)])
%         break
%     elseif theta_range(2,1) > theta(2) || theta(2) > theta_range(2,2)
%         disp(['Joint 2 Limit breached',theta(2)])
%         break
%     elseif theta_range(3,1) > theta(3) || theta(3) > theta_range(3,2)
%         disp(['Joint 2 Limit breached',theta(3)])
%         break
%     elseif theta_range(4,1) > theta(4) || theta(4) > theta_range(4,2)
%         disp(['Joint 2 Limit breached',theta(4)])
%         break
%     end
    
    X(i,:) = X_cord;
    Y(i,:) = Y_cord;
    Z(i,:) = Z_cord;
end

figure
for k = 1:i
    plot3(X(k,1:2),Y(k,1:2),Z(k,1:2),'-','LineWidth',5);
    hold on
    plot3(X(k,2:3),Y(k,2:3),Z(k,2:3),'-','LineWidth',5);
    hold on
    plot3(X(k,3:4),Y(k,3:4),Z(k,3:4),'-','LineWidth',4);
    hold on
    plot3(X(k,4:5),Y(k,4:5),Z(k,4:5),'-','LineWidth',3);
    hold on
    plot3(X(k,5:6),Y(k,5:6),Z(k,5:6),'-','LineWidth',3);
    hold on

    axis([-0.5 0.5 -0.5 0.5 0 0.5])
    axis square
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    grid on
    hold on

    drawnow
    hold off
end

figure
plot(2:i,del_X(1:3,2:end),'LineWidth',2)
xlabel('No of Iteration')
ylabel('Position error(mm)')
legend('(X_{goal} - X_{current})','(Y_{goal} - Y_{current})','(Z_{goal} - Z_{current})')
grid on
grid minor

figure
subplot(2,2,1)
plot(1:i,theta(:,1),'LineWidth',2)
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
plot(1:i,theta(:,2),'LineWidth',2)
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
plot(1:i,theta(:,3),'LineWidth',2)
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
plot(1:i,theta(:,4),'LineWidth',2)
hold on
plot(1:i,-103*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{4}')
grid on
grid minor
