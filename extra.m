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

%% Define parameters
x0 = 0.4; % Example value for x0
y0 = 0.6; % Example value for y0
z0 = 0.5; % Example value for z0
n = 4;  % Example value for n

% Create a finer grid for x, y, and z
[x, y, z] = meshgrid(linspace(-1, 1, 100), linspace(-1, 1, 100), linspace(-1, 1, 100));

% Define the implicit function
F = (x/x0).^n + (y/y0).^n + (z/z0).^n - 1;

% Create the figure
figure;
hold on;

% Extract and plot the isosurface where F = 0
p = patch(isosurface(x, y, z, F, 0));
isonormals(x, y, z, F, p);
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');

% Apply interpolation to smooth the surface
% p = reducepatch(p, 0.1); % Reduce the number of patches for smoothing

% Enhance visualization
daspect([1 1 1]);
view(3);
axis([-2 2 -2 2 -2 2]);
camlight;
lighting gouraud;
xlabel('x');
ylabel('y');
zlabel('z');
title('Plot of (x/x0)^n + (y/y0)^n + (z/z0)^n = 1');
grid on;
colormap jet;
colorbar;

hold off;
