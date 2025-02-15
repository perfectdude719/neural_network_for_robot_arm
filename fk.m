function result_transform = fk(n, theta, alpha, a, d)
    % Initialize the result transformation matrix as an identity matrix
    result_transform = eye(4);

    % Iterate through each link
    for i = 1:n
        % Compute the transformation matrix for the current link
        trans_mat = [
            cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
            sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1
        ];

        % Multiply the current transformation matrix with the result
        result_transform = result_transform * trans_mat;
    end
end