% ============================================================
% KALMAN_FILTER_1D
% Estimate the constant weight of an object from noisy scale readings.
%
% Idea: every time a new (noisy) measurement comes in, blend it with
% our current best guess. How much we trust each one depends on its
% uncertainty (variance). Each step has two parts:
%
%   1) PREDICT - carry the previous estimate forward
%   2) UPDATE  - correct that prediction using the new measurement
% ============================================================

clear;
clc;
close all;

%% 1. Example data

x_true = 10;                 % True weight [kg] (unknown in real life,
                              % only here so we can check our estimate)

% Measurements and how noisy each one is (given, not calculated).
z = [8, 12, 9, 10.5, 14];    % Measured weights [kg]
R = [4, 1, 4, 0.25, 9];      % Measurement variance [kg^2] (bigger = noisier)

% The weight doesn't change over time, so there is no process noise.
Q = 0;                        % Process-noise variance [kg^2]

N = numel(z);


%% 2. Initialise the filter

x_est      = zeros(1, N);     % Estimated weight after each update
P_est      = zeros(1, N);     % Uncertainty (variance) of that estimate
x_pred     = nan(1, N);       % Predicted weight before each update
P_pred     = nan(1, N);       % Uncertainty of that prediction
innovation = nan(1, N);       % Difference between measurement and prediction
K          = nan(1, N);       % Kalman gain: how much we trust the new data

% Use the first measurement as the starting estimate.
x_est(1) = z(1);
P_est(1) = R(1);


%% 3. Process each new measurement

for k = 2:N

    % --- Predict ---
    % Nothing is expected to change, so the prediction is just the
    % last estimate (with slightly more uncertainty from Q).
    x_pred(k) = x_est(k-1);
    P_pred(k) = P_est(k-1) + Q;

    % --- Update ---
    % See how far off the prediction was from the real measurement.
    innovation(k) = z(k) - x_pred(k);

    % Kalman gain balances prediction vs. measurement.
    % K near 1 -> trust the new measurement more.
    % K near 0 -> trust the prediction more.
    K(k) = P_pred(k) / (P_pred(k) + R(k));

    % Nudge the prediction toward the measurement, scaled by K.
    x_est(k) = x_pred(k) + K(k) * innovation(k);
    P_est(k) = (1 - K(k)) * P_pred(k);

end


%% 4. Display the results

estimation_error = x_est - x_true;

results = table((1:N)', z', R', x_pred', P_pred', innovation', ...
    K', x_est', P_est', estimation_error', ...
    'VariableNames', {'Step', 'Measurement', 'R', 'Prediction', ...
    'P_prediction', 'Innovation', 'KalmanGain', 'Estimate', ...
    'P_estimate', 'EstimationError'});

disp(results);


%% 5. Plot the results

steps = 1:N;

figure('Color', 'w');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Measurements vs. the filter's running estimate.
nexttile;
errorbar(steps, z, sqrt(R), 'ro', ...
    'LineWidth', 1.2, 'MarkerFaceColor', 'r');
hold on;
errorbar(steps, x_est, sqrt(P_est), 'bs-', ...
    'LineWidth', 1.5, 'MarkerFaceColor', 'b');
yline(x_true, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Measurement step');
ylabel('Weight [kg]');
title('Measurements and estimates');
legend('Measurement \pm 1\sigma', 'Estimate \pm 1\sigma', ...
    'True weight', 'Location', 'best');
xticks(steps);

% How the uncertainty shrinks as more measurements arrive.
nexttile;
plot(steps, R, 'ro-', 'LineWidth', 1.5);
hold on;
plot(steps, P_est, 'bs-', 'LineWidth', 1.5);
plot(2:N, P_pred(2:end), 'kd--', 'LineWidth', 1.2);
grid on;
xlabel('Measurement step');
ylabel('Variance [kg^2]');
title('Uncertainty');
legend('Measurement R', 'Updated P', 'Predicted P^-', ...
    'Location', 'best');
xticks(steps);

% How much each new measurement was trusted.
nexttile;
stem(2:N, K(2:end), 'filled', 'LineWidth', 1.5);
grid on;
xlabel('Measurement step');
ylabel('Kalman gain K');
title('Kalman gain');
xticks(steps);
ylim([0 1]);

% The residual (measurement minus prediction) at each step.
nexttile;
stem(2:N, innovation(2:end), 'filled', 'LineWidth', 1.5);
hold on;
yline(0, 'k--');
grid on;
xlabel('Measurement step');
ylabel('Innovation [kg]');
title('Measurement residual');
xticks(steps);

sgtitle('Scalar Kalman filter');