
% Main script
% Define robot parameters
n = 7;  % Number of links
a = [0, 218, 196, 36, 0, 7.5, 11];  % Link lengths
d = [147, 0, 0, 0, 425, 0, 0];  % Link offsets
alpha = [pi/2, 0, 0, pi/2, -pi/2, -pi/2, 0];  % Constant link twists

% Generate dataset
num_samples = 100000;
[X, y] = generate_dataset(num_samples, n, a, d, alpha);

% Split the dataset into training and validation sets
cv = cvpartition(size(X, 1), 'HoldOut', 0.2);
X_train = X(cv.training,:);
y_train = y(cv.training,:);
X_val = X(cv.test,:);
y_val = y(cv.test,:);

% Build the neural network model
input_shape = size(X_train, 2);  % 3 (x, y, z)
output_shape = size(y_train, 2);  % n (joint angles)

layers = [
    featureInputLayer(input_shape)
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(output_shape)
    regressionLayer
];

% Define training options
options = trainingOptions('adam', ...
    'MaxEpochs', 5, ...
    'MiniBatchSize', 32, ...
    'ValidationData', {X_val, y_val}, ...
    'Verbose', false, ...
    'Plots', 'training-progress');

% Train the model
net = trainNetwork(X_train, y_train, layers, options);
% Save the trained network to a .mat file
save('trained_network.mat', 'net');