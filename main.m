%% Kalman Filter on a Constant Acceleration Model

% Simulation Setup
NUM_STEPS = 25;
u = [1, 1]'; % Input Acceleration
a1 = accelparams( ...
    "MeasurementRange", 19.6, ...
    "NoiseDensity", 0.003924);
imu = imuSensor("SampleRate", 1000);
imu.Accelerometer = a1;
zv = [0, 0];

% Variables
estimate = []; % list of estimates
K = {}; % list of Kalman Gain Matrices
z={}; % List of measure vectors
inn = []; % list of innovations


del_t = 1; % Time Step
sigma_a = 100; % Process noise of acceleration
sigma_x = 3; % Measurement Noise in X Direction
sigma_y = 3; % Measurement Noise in Y Direction
 

% State Transistion Matrix
f_1 = [1 del_t; 0 1];
F = [f_1 zeros(2,2); zeros(2,2) f_1];

% Control Matrix
g_1 = [0.5*del_t^2; del_t];
G = [g_1, zeros(2,1); zeros(2,1), g_1];

% Estimate Uncertainity
P_0 = [diag([500 500 500 500])];
P = {P_0 0; 0 0}; 

% Process Noise Uncertainity
q_1 = [0.25*del_t^4 0.5*del_t^3; 0.5*del_t^3 del_t^2];
Q = [q_1 zeros(2,2); zeros(2,2) q_1] * sigma_a^2;

% Observation Matrix
H = [1 0 0 0; 0 0 1 0];

% Measurement Uncertainity
R = [sigma_x^2 0; 0 sigma_y^2];

x_hat = {zeros(4,1), 0; 0 0}; % List of state estimation matrices

% First Iteration
x_hat{2, 1} = F * x_hat{1, 1} + G * u;
P{2, 1} = UpdateUncertainity(P{1, 1}, F, Q);
z{1} = [393.68; 300.4];

% Further Iterations
for i = 2:NUM_STEPS
    % Measure
    K{i-1} = KalmanGain(P{i, i-1}, H, R);
   
    % Update
    x_hat{i, i} = x_hat{i, i-1} + K{i-1} * (z{i-1} - H*x_hat{i,i-1});
    P{i, i} = UpdateCovariance(K{i-1}, H, P{i, i-1}, R);
    [z{i}, zv] = IMuMeasurement(imu, del_t, z{i-1}', zv);
    
    % Predict
    x_hat{i+1, i} = F * x_hat{i, i} + G * u;
    P{i+1, i} = UpdateUncertainity(P{i, i}, F, Q);

    inn(i-1,:) = (z{i-1} - H*x_hat{i,i-1})';
    
end

measure = cell2mat(z); % list of measurements
xm = measure(1,3:NUM_STEPS);
ym = measure(2,3:NUM_STEPS);



for i = 1:NUM_STEPS
    estimate(:, i) = (H * x_hat{i+1,i})';
end

xe = estimate(1,3:NUM_STEPS);
ye = estimate(2,3:NUM_STEPS);
rmse = sqrt((xm-xe).^2 + (ym-ye).^2); % Root Mean Square
avg_rmse = sum(rmse)/length(rmse); % Avertage RMSE

figure(1)
plot(xm,ym,'o-', xe, ye,'v-');
xlabel('X-distance'); ylabel('Y-distance')
title('XY-Position of the Vehicle')
grid on
legend('Measured Position', 'Estimated Position', 'Location','northwest');

figure(2)
hold on
plot(1:length(rmse),rmse);
xlabel('Time Index'); ylabel('RMSE')
title('Root Mean Square Error of the Position')
grid on

figure(3)
plot(inn,'s-')
xlabel('Time Index'); ylabel('Innovation')
title('Innovation over Time')
legend('X-Position', 'Y-Position');
grid on