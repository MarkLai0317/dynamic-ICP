function curve_fitting()

    % ... (rest of your code)
    
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

    


    % Set up the optimization
    original_points = [sampled_x(:), sampled_y(:)];
    target_z = sampled_z(:);

    % Initial guess for the parameters (e.g., [0; 0; 0; 0; 0; 0])
    initial_params = zeros(6, 1);

    % Call lsqcurvefit
    opts = optimoptions('lsqcurvefit', 'Display', 'iter');  % Show iteration output
    optimal_params = lsqcurvefit(@error_function, initial_params, original_points, target_z, [], [], opts);
    disp('Optimal Parameters:');
    disp(optimal_params);
    % ... (rest of your code)

    function err = error_function(params, original_points)
        transformed_points = transform_function(params, original_points);
        % Assume the surface is defined by z = f(x, y)
        err = transformed_points(:, 3) - target_z;
    end

end