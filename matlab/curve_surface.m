% Define the coefficients
a = -2;
b = 0.5;
c = -0.1;
d = -0.02;
e = 0.4;
f = 1;

% Define your equation f(x, y)
f = @(x, y) a * x.^2 + b * x + c * x .* y + d * y + e * y.^2 + f;

% Create a grid of x and y values
x_min = -5;
x_max = 5;
y_min = -5;
y_max = 5;
num_points = 100;
x = linspace(x_min, x_max, num_points);
y = linspace(y_min, y_max, num_points);
[X, Y] = meshgrid(x, y);

% Evaluate the function
Z = f(X, Y);

% Sample 500 points from the surface
num_samples = 500;
rng('default');  % Reset the random number generator for reproducibility
indices = randperm(numel(X), num_samples);  % Randomly select indices
sampled_x = X(indices);
sampled_y = Y(indices);
sampled_z = Z(indices);

% Create a matrix containing x, y, and z coordinates
sampled_data = [sampled_x(:), sampled_y(:), sampled_z(:)];

% Save the sampled data to a .txt file
filename = 'sampled_data.txt';
dlmwrite(filename, sampled_data, 'delimiter', '\t');

disp(['Sampled data saved to ' filename]);

% Create a 3D plot of the original curve surface
figure;
surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeAlpha', 0.2); % Original surface

hold on;

% Plot the sampled points as red dots
scatter3(sampled_x, sampled_y, sampled_z, 'r', 'filled'); % Sampled points

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Original Curve Surface with Sampled Points');
hold off;
