function [X, y] = generate_dataset(num_samples, n, a, d, alpha)
    X = [];
    y = [];

    for i = 1:num_samples
        % Randomly generate joint angles (theta)
        theta = -pi + 2 * pi * rand(1, n);

        % Compute the end-effector position using forward kinematics
        T = fk(n, theta, alpha, a, d);
        end_effector_pos = T(1:3, 4);  % Extract x, y, z from the transformation matrix

        % Append to dataset
        X = [X; end_effector_pos'];
        y = [y; theta];
    end
end