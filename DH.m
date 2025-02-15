% Check the number of links in the robot
num_links = numel(smiData.Bodies);

% Initialize DH parameters
a = zeros(1, num_links);      % Link lengths
alpha = zeros(1, num_links);  % Link twists
d = zeros(1, num_links);      % Link offsets
theta = sym('theta', [1, num_links]); % Joint angles (symbolic for revolute joints)

% Extract transformations and compute DH parameters
for i = 1:num_links
    if i == 1
        T = robot.Bodies{i}.Joint.HomeTransform; % First joint relative to base
    else
        T_prev = robot.Bodies{i-1}.Joint.HomeTransform;
        T_curr = robot.Bodies{i}.Joint.HomeTransform;
        T = T_prev \ T_curr; % Relative transform
    end
    
    % Calculate DH parameters
    a(i) = T(1,4); % Link length
    alpha(i) = atan2(T(3,2), T(3,3)); % Link twist
    d(i) = T(3,4); % Link offset
end

% Display the DH parameters
disp('DH Parameters:');
disp(table(a', alpha', d', theta', 'VariableNames', {'a', 'alpha', 'd', 'theta'}));
