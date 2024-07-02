%% Unicycle Model
clear all; clc;

N = 4; % Prediction Horizon

addpath(genpath('Utilities'))

%DH parameters
n = 4; % No of Joint
alpha = [0,90,0,0]; % In degree
a = [0,0,sqrt(0.128^2 + 0.024^2),0.124];
d = [0.077,0,0,0];
theta = [0,79.38,-79.38,0]; % In degree 
Le = 0.126; % End-effector length

[X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);
x = X_cord;
y = Y_cord;
z = Z_cord;

X = [X_cord(end); Y_cord(end); Z_cord(end)];

[~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta); % Jacobain matrix of Main task (3x4)

% Referance Input
X_d = [-0.2; -0.175; 0.04];

% Error Terms
e_state = X_d - X;

% Define the pattern
Q = [10,10,10];
R = [0.1,0.1,0.2,0.2];

% Define the number of repetitions
num_repeats = N;  % Adjust this to the number of times you want to repeat the pattern

% Replicate the pattern
extended_Q = repmat(Q, 1, num_repeats);
extended_R = repmat(R, 1, num_repeats);

%%Weight Matrix
Q = diag(extended_Q);  
R = diag(extended_R);

%%Prediction Model Matrix
A = eye(3);
A_ = A;  % shape(3,3)
for q = 2:N
    A_ = [A_;A];  % sincle A is Identity matrix, we can simplify [A_;A.^q];
end

i = 1;
while norm(e_state(:,i)) > 0.001
    %%System Model Matrix
    B = -Jv;  % (3x4)
    
    %%Prediction Model Matrix
    M = B; 
    for q1 = 2:N  % row
        for q2 = 1:q1  % column (dynamic)
            M = [M,B];
        end
    end
    
    p1 = 1; p2 = 1;
    [Row,Column] = size(B);  % (3,4)
    for q3 = 1:Row:Row*N  
        for q4 = 1:Column:p1  
            B_(q3:q3+Row-1, q4:q4+Column-1) = M(:, p2:p2+Column-1);
            p2 = p2 + Column;
        end
        p1 = p1 + Column; 
    end

    %%Optimization
    Ue = -inv(R + B_' * Q * B_) * B_' * Q * A_ * e_state(:,i); % U[w|k]

    d_theta(:,i) = (1-exp(-i*0.05)) * Ue(1:Column,1); % u[0|k] = U[1|k]

    theta = theta + d_theta(:,i)';  % in degrees

    [X_cord, Y_cord, Z_cord] = Forward_Kinematic(n,alpha,a,d,theta,Le);  % X[w+1|k] = A*X[w|k] + B*u[w|k]

    X(:,i+1) = [X_cord(end); Y_cord(end); Z_cord(end)];  
            
    [~,Jv,~] = Jacobian_matrix(n,alpha,a,d,theta); % Jacobain matrix of Main task (3x4)

    e_state(:,i+1) = X_d - X(:,i+1);

    i = i + 1;
    
    x(i,:) = X_cord;
    y(i,:) = Y_cord;
    z(i,:) = Z_cord;
end

figure(1)
for k=1:i-200
    plot3(x(1,6),y(1,6),z(1,6),"*g",'LineWidth',8)
    hold on
    plot3(X_d(1),X_d(2),X_d(3),"*r",'LineWidth',8)
    hold on
    plot3(x(k,1:2),y(k,1:2),z(k,1:2),'-','LineWidth',8);
    hold on
    plot3(x(k,2:3),y(k,2:3),z(k,2:3),'-','LineWidth',7);
    hold on
    plot3(x(k,3:4),y(k,3:4),z(k,3:4),'-','LineWidth',6);
    hold on
    plot3(x(k,4:5),y(k,4:5),z(k,4:5),'-','LineWidth',5);
    hold on
    plot3(x(k,5:6),y(k,5:6),z(k,5:6),'-','LineWidth',4);
    hold on
    plot3(X(1,1:k),X(2,1:k),X(3,1:k),LineWidth=2,Color='m')

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

figure(2)
subplot(1,2,1)
plot(1:size(e_state,2),e_state(1,:),'--',LineWidth=2,Color='g')
hold on
plot(1:size(e_state,2),e_state(2,:),'--',LineWidth=2,Color='b')
hold on
plot(1:size(e_state,2),e_state(3,:),'--',LineWidth=2,Color='r')
hold on 
plot(1:size(e_state,2),rms(e_state(1:3,:)),'LineWidth',2)
axis square
grid on
legend('e_{x}','e_{y}','e_{z}','RMS Error')
xlabel('steps')
ylabel('$Error$','interpreter','latex',FontSize=15)
hold off

subplot(1,2,2)
plot(1:size(d_theta,2),deg2rad(d_theta(1,:)),LineWidth=2,Color='g')
hold on
plot(1:size(d_theta,2),deg2rad(d_theta(2,:)),LineWidth=2,Color='b')
hold on
plot(1:size(d_theta,2),deg2rad(d_theta(3,:)),LineWidth=2,Color='r')
hold on
plot(1:size(d_theta,2),deg2rad(d_theta(4,:)),LineWidth=2,Color='c')
axis square
grid on
legend('\theta_{1}','\theta_{2}','\theta_{3}', '\theta_{1}')
xlabel('steps')
ylabel('$Joint Velocity (rad/s)$','interpreter','latex',FontSize=15)
hold off