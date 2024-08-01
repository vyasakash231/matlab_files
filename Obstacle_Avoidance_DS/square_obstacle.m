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
disp(size(p.Vertices))  % coordinates of points on the isosurface
% isonormals(x, y, z, F, p);
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');

view(3);
axis([-2 2 -2 2 -2 2]);
xlabel('x');
ylabel('y');
zlabel('z');
title('Plot of (x/x0)^n + (y/y0)^n + (z/z0)^n = 1');
grid on;
hold off;
