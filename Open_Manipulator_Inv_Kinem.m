clc;clear all;
syms theta1 theta2 theta3 theta4

n = 4;
p = 0.128; q = 0.024;
l1 = 0.077; l2 = sqrt(p^2 + q^2); l3 = 0.124;

% End Effector coordinates
Px = 0.274; Py = 0; Pz = 0.205; phi = 0;
O_E_0 = [Px;Py;Pz];

% Calculate theta1
th1 = atan2(Py,Px);

% O_4_0 Coordinated 
R_4_0 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
O_4_0 = O_E_0 - R_4_0*[0.126;0;0];

% O_1_0 Coordinated 
O_1_0 = [0;0;l1];

% Calculate theta3  
D1 = sqrt(sum((O_4_0 - O_1_0).^2));
cos_psi = (l3^2 + l2^2 - D1^2)/(2*l2*l3);
sin_psi = sqrt(1 - cos_psi^2);

psi_a = atan2(sin_psi,cos_psi);
th3_a = -pi + psi_a; % correct config

% Calculate theta2
cos_alpha = (l2^2 + D1^2 - l3^2)/(2*l2*D1);
sin_alpha = sqrt(1 - cos_alpha^2);

alpha_a = atan2(sin_alpha,cos_alpha);
beta = atan2((O_4_0(3)-O_1_0(3)),O_4_0(1));

th2 = alpha_a + beta;

% Calculate theta4
th4 = phi - th3_a - th2;

% DH parameters
alpha = [0,pi/2,0,0];
a = [0,0,sqrt(p^2 + q^2),l3];
d = [l1,0,0,0];
theta = [theta1, theta2, theta3, theta4];

I = eye(4);

% Transformation matrix (all angles in Radian)
for i = 1:(n) 
    T = [cos(theta(i))                          -sin(theta(i))                    0                    a(i) ;
     sin(theta(i))*cos(alpha(i))      cos(theta(i))*cos(alpha(i))     -sin(alpha(i))     -d(i)*sin(alpha(i));
     sin(theta(i))*sin(alpha(i))      cos(theta(i))*sin(alpha(i))      cos(alpha(i))      d(i)*cos(alpha(i));
                   0                                 0                           0                       1  ];
    T_new = I*T;
    R(1:3,1:3,i) = T_new(1:3,1:3);  
    O(1:3,i) = T_new(1:3,4);
    I = T_new;
end

% O_3_0 Coordinated 
R_3_0 = subs(R(:,:,3),[theta1 theta2 theta3],[th1 th2 th3_a]);
O_3_0 = simplify(O_4_0 - R_3_0*[l3;0;0]);
O_3_0 = double(O_3_0);

% O_2_0 Coordinated 
R_2_0 = subs(R(:,:,2),[theta1 theta2],[th1 th2]);
O_2_0 = round(vpa(O_3_0 - R_2_0*[l2;0;0]),4);

% Plot
X_cord = [0,O_1_0(1),O_2_0(1),O_3_0(1),O_4_0(1),Px]; % X-coordinate of each link
Y_cord = [0,O_1_0(2),O_2_0(2),O_3_0(2),O_4_0(2),Py]; % Y-coordinate of each link
Z_cord = [0,O_1_0(3),O_2_0(3),O_3_0(3),O_4_0(3),Pz]; % Z-coordinate of each link

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


