% Parameters
a = 1;
b = 1;
m = 5;       % Number of arms of the starfish
n1 = 0.25;      % Controls the overall shape
n2 = 1.5;      % Controls the shape along the x-axis
n3 = 1.5;      % Controls the shape along the y-axis
num_points = 500;

% Generate angles
theta = linspace(0, 2*pi, num_points);

% Polar equation for the generalized superellipse  (https://mathworld.wolfram.com/Superellipse.html)
r = ((abs(cos(m * theta / 4) / a).^n2 + abs(sin(m * theta / 4) / b).^n3).^(-1/n1));

% Parametric equations
x = r .* cos(theta);
y = r .* sin(theta);

% Plot the starfish shape
figure;
plot(x, y, 'b-', 'LineWidth', 2);
axis equal;
title('Starfish Shape using Generalized Superellipse Equation');
xlabel('x');
ylabel('y');
grid on;
