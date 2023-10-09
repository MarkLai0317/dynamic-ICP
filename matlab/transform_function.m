function transformed_points = transform_function(params, original_points)
    % params contains [alpha, beta, gamma, tx, ty, tz]
    % Construct the rotation matrix
    alpha = params(1);
    beta = params(2);
    gamma = params(3);
    R_x = [1, 0, 0;
           0, cos(alpha), -sin(alpha);
           0, sin(alpha), cos(alpha)];

    R_y = [cos(beta), 0, sin(beta);
           0, 1, 0;
           -sin(beta), 0, cos(beta)];
    
    R_z = [cos(gamma), -sin(gamma), 0;
           sin(gamma), cos(gamma), 0;
           0, 0, 1];
    R = R_z * R_y * R_x;  % Z-Y-X rotation sequence
    translation = params(4:6)';
    disp(translation);
    % disp(R)
    % Create a 4x4 transformation matrix
    T = [R, translation];  % Now this line is correct

    homogeneous_coords = [original_points, ones(size(original_points,1), 1)];
   
    % disp( T)
    % disp( homogeneous_coords)

    transformed_data_homogeneous = (T * homogeneous_coords')';
    disp(transformed_data_homogeneous)
    
    transformed_points = transformed_data_homogeneous;
end
