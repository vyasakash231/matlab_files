clc; clear all; close all;

dt = 0.1;
T = 0:dt:100;

seta_initial = -1;
for j=1:50
    Zeta_vec(:,1,j) = [0;seta_initial];  % [zeta_1; zeta_2]
    seta_initial = seta_initial + 1;
end

for j=1:50
    for i=1:length(T)
        Zeta_dot_vec = [1;sin(Zeta_vec(1,i,j))];  % [zeta_dot_1; zeta_dot_2] = [1; sin(zeta_1)]
        Zeta_vec(:,i+1,j) = Zeta_vec(:,i,j) + Zeta_dot_vec * dt;
    end
end

% figure;
% for j=1:50
%     plot(Zeta_vec(1,:,j), Zeta_vec(2,:,j), "LineWidth", 1.5, "Color", 'red')
%     hold on
%     axis([0 25 0 25])
%     xlabel('\zeta_{1}','FontSize',15)
%     ylabel('\zeta_{2}','FontSize',15)
% end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define parameters
x0 = 0.5; % Example value for x0
y0 = 0.5; % Example value for y0
z0 = 0.5; % Example value for z0
n = 4;  % Example value for n

xc = 0.5; yc = 0.5; zc = 0.5;  % center of box

% Define the function
% f = @(x, y, z) (x/x0).^n + (y/y0).^n + (z/z0).^n - 1;

% Create a grid for x and y
[x, y] = meshgrid(linspace(-1, 1, 500), linspace(-1, 1, 500));

% Solve for z
z_positive = ((1 - ((x-xc)/x0).^n - ((y-yc)/y0).^n) .* z0^n).^(1/n) - zc;
z_negative = -((1 - ((x-xc)/x0).^n - ((y-yc)/y0).^n) .* z0^n).^(1/n) - zc;

% Filter out complex values
z_positive(imag(z_positive) ~= 0) = NaN;
z_negative(imag(z_negative) ~= 0) = NaN;

% Plot the positive surface
figure;
surf(x, y, real(z_positive));
hold on;
% Plot the negative surface
surf(x, y, real(z_negative));

% Customize plot
xlabel('x');
ylabel('y');
zlabel('z');
title('Plot of (x-xc/x0)^n + (y-yc/y0)^n + (z-zc/z0)^n = 1');
grid on;
axis equal;
axis([-2 2 -2 2 -2 2])

% Additional customization for clarity
shading interp;
alpha 0.7;

