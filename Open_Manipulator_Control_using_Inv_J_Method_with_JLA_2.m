%% Configuration control Method with Joint Limit avoidance using Kinematic Optimization
%{
%}
clc; clear all; close all;
%addpath E:\Matlab_Files\Robotics\Lab_Assignment % To access function from other folders

% Goal Location in Task Space
% X_g = input('Enter X-coordinate of goal position: ');
% Y_g = input('Enter X-coordinate of goal position: ');
% Z_g = input('Enter X-coordinate of goal position: ');
X_g = 0.17; Y_g = 0.13; Z_g = 0.1;
%X_g = 0.12; Y_g = 0.13; Z_g = 0.14;

m = 5; % mth norm of a vector

%{
weight to penalize high joint rate, 
causing the manipulator not to move 
close to the singularity posture.
%}
lambda = 0.1; % weights for Singularity avoidance

%{
In some practical cases, avoiding joint limit is more 
important for certain joints, in such cases a weight 
matrix K is multiplied to the mth norm.
%}
K = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];  

n = 4; % No of Joint
p = 0.128; q = 0.024;

Q = [0,0,0,0]; % Initia angle in hardware
offset = [0,79.38,-79.38,0]; % Joint angle offset in hardware

%DH parameters
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = Q + offset; % In degree
Le = 0.126; % End-effector length

i = 1;

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
X(i,:) = X_cord;
Y(i,:) = Y_cord;
Z(i,:) = Z_cord;

Q_range = [-180 180;-117 90;-90 87.5;-103 114.5]; % range of motion of each joint

del_X = ones(3,i);
while max(abs(del_X(1:3,i))) > 0.02
%     if theta_range(1,1) > theta(i,1) || theta(i,1) > theta_range(1,2)
%         disp('Joint 1 Limit breached')
%         break
%     elseif theta_range(2,1) > theta(i,2) || theta(i,2) > theta_range(2,2)
%         disp('Joint 2 Limit breached')
%         break
%     elseif theta_range(3,1) > theta(i,3) || theta(i,3) > theta_range(3,2)
%         disp('Joint 2 Limit breached')
%         break
%     elseif theta_range(4,1) > theta(i,4) || theta(i,4) > theta_range(4,2)
%         disp('Joint 2 Limit breached')
%         break
%     end

    X_c = X(i,n+2); 
    Y_c = Y(i,n+2); 
    Z_c = Z(i,n+2);

    v = cost_func(n,K,deg2rad(Q(i,:)),deg2rad(Q_range),m);
    
    del_X(:,i+1) = [(X_g - X_c); (Y_g - Y_c); (Z_g - Z_c)];
    
    [~,Je,~] = Jacobian_matrix(n,alpha,a,d,theta(i,:)); % Jacobain matrix of Main task

    J1 = (Je'*Je + lambda^2*eye(n))\Je';
    J2 = (eye(n) - J1*Je);
    
    % Solving using Newton-Raphson Method
    theta_new = theta(i,:) + (J1*del_X(:,i+1))' + (J2*v')'; % In degree --> [Q_new + offset] = [Q + offset] + (J*del_X(:,i+1))'
    Q(i+1,:) = theta_new - offset; % Q_new = [Q + offset] + (J*del_X(:,i+1))' - offset
    theta(i+1,:) = theta_new; % In degree

    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta(i+1,:),Le);

    i = i + 1;
    
    X(i,:) = X_cord;
    Y(i,:) = Y_cord;
    Z(i,:) = Z_cord;
end

%save JLA_using_Kinematic_Optimization X Y Z theta del_X % To save data

figure
for k = 1:i
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
plot(1:i,Q(:,1),'LineWidth',2)
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
plot(1:i,Q(:,2),'LineWidth',2)
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
plot(1:i,Q(:,3),'LineWidth',2)
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
plot(1:i,Q(:,4),'LineWidth',2)
hold on
plot(1:i,-103*ones(1,i),'--','LineWidth',2)
hold on
plot(1:i,114.5*ones(1,i),'--','LineWidth',2)
hold off
xlabel('No of Iteration')
ylabel('\theta_{4}')
grid on
grid minor
