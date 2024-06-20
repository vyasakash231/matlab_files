clc; clear;

n = 4;
p = 0.128; q = 0.024;

% DH parameters
alpha = [0,90,0,0];
a = [0,0,sqrt(p^2 + q^2),0.124];
d = [0.077,0,0,0];
theta = [0,atan2d(p,q),-atan2d(p,q),0];

% Transformation matrix
[T_new,~,O] = Transformation_matrix(n,alpha,a,d,theta); % all angles in degrees
 
T_final = T_new;
d_nn = [0.126;0;0];
P_00_Homo = T_final*[d_nn;1];
P_00 = P_00_Homo(1:3);        % convert P_00_homogeneous to P_00_cartesian

% Plot
X_cord = [0,O(1,1:n),P_00(1)];   % X-coordinate of each link
Y_cord = [0,O(2,1:n),P_00(2)];   % Y-coordinate of each link
Z_cord = [0,O(3,1:n),P_00(3)];   % Z-coordinate of each link
 
figure(1)
for i = 1:length(X_cord)-1
    plot3(X_cord(i:i+1),Y_cord(i:i+1),Z_cord(i:i+1),'-o','LineWidth',2);
    hold on
end
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
axis square
grid on
hold off 

