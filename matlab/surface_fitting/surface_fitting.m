% Define the coefficients
a = -2;
b = 0.5;
c = -0.1;
d = -0.02;
e = 0.4;
f = 1;

% Define your equation f(x, y)
surfaceF = @(x, y) a * x.^2 + b * x + c * x .* y + d * y + e * y.^2 + f;

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
Z = surfaceF(X, Y);

% Sample 500 points from the surface
num_samples = 500;
rng('default');  % Reset the random number generator for reproducibility
indices = randperm(numel(X), num_samples);  % Randomly select indices
sampled_x = X(indices);
sampled_y = Y(indices);
sampled_z = Z(indices);



% Define a 3x4 transformation matrix
% Define the rotation angles
theta_x = pi/36; % Example rotation angle for x-axis
theta_y = pi/20; % Example rotation angle for y-axis
theta_z = pi/24; % Example rotation angle for z-axis

% Define the individual rotation matrices
R_x = [1, 0, 0;
       0, cos(theta_x), -sin(theta_x);
       0, sin(theta_x), cos(theta_x)];

R_y = [cos(theta_y), 0, sin(theta_y);
       0, 1, 0;
       -sin(theta_y), 0, cos(theta_y)];

R_z = [cos(theta_z), -sin(theta_z), 0;
       sin(theta_z), cos(theta_z), 0;
       0, 0, 1];

% Combine the rotation matrices
R = R_x * R_y * R_z;
translation = [2; 3; 10]; % Translation vector

T = [R, translation];
% Display the rotation matrix
disp('Rotation Matrix R:');
disp(R);

% Apply the transformation to the sampled points
homogeneous_coords = [sampled_x(:)'; sampled_y(:)'; sampled_z(:)'; ones(1, numel(sampled_x))];
transformed_data_homogeneous = T * homogeneous_coords;
disp(T)
disp(homogeneous_coords)

% Convert back from homogeneous to Cartesian coordinates
transformed_data = transformed_data_homogeneous(1:3, :)';

% Extract transformed x, y, and z coordinates
transformed_x = transformed_data(:, 1);
transformed_y = transformed_data(:, 2);
transformed_z = transformed_data(:, 3);

% Save the transformed data to a .txt file
filename_transformed = 'transformed_data.txt';
dlmwrite(filename_transformed, transformed_data, 'delimiter', ' ');

disp(['Transformed data saved to ' filename_transformed]);

% Create a 3D plot of the original curve surface
figure;
surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeAlpha', 0.2); % Original surface

hold on;

% Plot the transformed sampled points as blue dots
scatter3(transformed_x, transformed_y, transformed_z, 'b', 'filled'); % Transformed points

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Original Curve Surface with Transformed Sampled Points using 3x4 Matrix Transformation');
hold off;

% 
% Initial guess for the coefficients
initial_guess = [0,0,0,0,0,0]; 

% Combine the x, y, and z data into matrices
xdata = [transformed_x, transformed_y, transformed_z];


% original zdata = transformed_z;
zdata = transformed_z * 0;


% %
% Use lsqcurvefit to fit the surface equation to the data
opts = optimset('Display', 'off');  % Turn off display


fitted_params = lsqcurvefit(@transformed_z, initial_guess, xdata, zdata);


% Display the fitted parameters
disp('Fitted parameters:');
disp(fitted_params);




% check the result of fitted_params


% Create a 3D plot of the original curve surface
figure;
surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeAlpha', 0.2); % Original surface

hold on;

fitted_points = transform_function(fitted_params, transformed_data);

fitted_x = fitted_points(:,1);
fitted_y = fitted_points(:,2);
fitted_z = fitted_points(:,3);

% Plot the transformed sampled points as blue dots
scatter3(fitted_x, fitted_y, fitted_z, 'r', 'filled'); % Transformed points

xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Original Curve Surface with Transformed Sampled Points using 3x4 Matrix Transformation');
hold off;
