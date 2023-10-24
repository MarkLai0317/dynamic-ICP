function transformed_z = transformed_z(transform_params, original_points)
    
    a = -2;
    b = 0.5;
    c = -0.1;
    d = -0.02;
    e = 0.4;
    f = 1;
    
    % Define your equation f(x, y)
    surfaceF = @(x, y) a * x.^2 + b * x + c * x .* y + d * y + e * y.^2 + f;

    transformed_points = transform_function(transform_params, original_points);

    X = transformed_points(:, 1);
    Y = transformed_points(:, 2);
    Z = surfaceF(X,Y);
    %transformed_z = Z;

    transformed_z = transformed_points(:, 3)-Z;


    % Assuming transformed_points_computed and transformed_points_actual are N-by-3 matrices
    % and we want to compute the Euclidean distance for each point
    
end
