% ============================================================
% KALMAN_FILTER_2D
% Estimate the constant position [x; y] of an object from noisy
% position measurements.
%
% Same idea as the 1D case, just with 2 numbers (x and y) tracked
% together instead of 1. Each measurement now has its own 2x2
% covariance matrix, since x and y can be noisy by different amounts.
%
%   1) PREDICT - carry the previous estimate forward
%   2) UPDATE  - correct that prediction using the new measurement
% ============================================================

clear;
clc;
close all;

%% 1. Example data

x_true = [10; 5];             % True position [x; y] in metres

% Each column is one position measurement.
z = [8.0, 11.0, 9.5, 10.2, 13.0;
     6.0,  4.5, 5.8,  5.1,  7.0];

N = size(z, 2);

% Measurement covariance for each step: a 2x2 matrix per column of z,
% so we store them in a cell array (one cell per step).
% Off-diagonal terms are 0 here, meaning x-noise and y-noise are
% assumed independent of each other.
R = cell(N, 1);
R{1} = [4.00, 0; 0, 1.00];
R{2} = [1.00, 0; 0, 4.00];
R{3} = [2.25, 0; 0, 2.25];
R{4} = [0.25, 0; 0, 0.25];
R{5} = [9.00, 0; 0, 9.00];

Q = zeros(2);                 % Position is constant, so no process noise
I = eye(2);

%% 2. Initialise the filter

x_est      = zeros(2, N);     % Estimated [x; y] after each update
x_pred     = nan(2, N);       % Predicted [x; y] before each update
innovation = nan(2, N);       % Difference between measurement and prediction

P_est  = cell(N, 1);          % Uncertainty of the estimate (2x2 per step)
P_pred = cell(N, 1);          % Uncertainty of the prediction (2x2 per step)
K      = cell(N, 1);          % Kalman gain (2x2 per step)

% Use the first measurement as the starting estimate.
x_est(:,1) = z(:,1);
P_est{1} = R{1};

%% 3. Process each new measurement

for k = 2:N

    % --- Predict ---
    % Nothing is expected to change, so the prediction is just the
    % last estimate (with slightly more uncertainty from Q).
    x_pred(:,k) = x_est(:,k-1);
    P_pred{k} = P_est{k-1} + Q;

    % --- Update ---
    % See how far off the prediction was from the real measurement.
    innovation(:,k) = z(:,k) - x_pred(:,k);

    % Kalman gain balances prediction vs. measurement (matrix version
    % of the same K = P / (P + R) idea as the scalar filter).
    K{k} = P_pred{k} / (P_pred{k} + R{k});

    % Nudge the prediction toward the measurement, scaled by K.
    x_est(:,k) = x_pred(:,k) + K{k} * innovation(:,k);
    P_est{k} = (I - K{k}) * P_pred{k};

end

%% 4. Prepare and display the results

% Pull out just the diagonal (x and y) parts of each matrix for
% easy plotting and tabulating.
K_diag = nan(2, N);
P_diag = nan(2, N);
R_diag = nan(2, N);

for k = 1:N
    P_diag(:,k) = diag(P_est{k});
    R_diag(:,k) = diag(R{k});

    if k > 1
        K_diag(:,k) = diag(K{k});
    end
end

position_error = sqrt( ...
    (x_est(1,:) - x_true(1)).^2 + ...
    (x_est(2,:) - x_true(2)).^2);

results = table((1:N)', z(1,:)', z(2,:)', ...
    x_est(1,:)', x_est(2,:)', position_error', ...
    K_diag(1,:)', K_diag(2,:)', ...
    'VariableNames', {'Step', 'MeasuredX', 'MeasuredY', ...
    'EstimatedX', 'EstimatedY', 'PositionError', 'GainX', 'GainY'});

disp(results);

%% 5. Plot the results

steps = 1:N;

figure('Color', 'w');
tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% Position in the x-y plane.
nexttile;
plot(z(1,:), z(2,:), 'ro', 'MarkerFaceColor', 'r');
hold on;
plot(x_est(1,:), x_est(2,:), 'bs-', 'LineWidth', 1.5);
plot(x_true(1), x_true(2), 'kp', ...
    'MarkerFaceColor', 'y', 'MarkerSize', 12);
grid on;
axis equal;
xlabel('x-position [m]');
ylabel('y-position [m]');
title('2D position');
legend('Measured', 'Estimated', 'True', 'Location', 'best');

% x-coordinate over time.
nexttile;
plot(steps, z(1,:), 'ro-', steps, x_est(1,:), 'bs-', ...
    'LineWidth', 1.3);
hold on;
yline(x_true(1), 'k--');
grid on;
xlabel('Step');
ylabel('x [m]');
title('x-coordinate');
xticks(steps);

% y-coordinate over time.
nexttile;
plot(steps, z(2,:), 'ro-', steps, x_est(2,:), 'bs-', ...
    'LineWidth', 1.3);
hold on;
yline(x_true(2), 'k--');
grid on;
xlabel('Step');
ylabel('y [m]');
title('y-coordinate');
xticks(steps);

% Position error.
nexttile;
plot(steps, position_error, 'mo-', 'LineWidth', 1.5);
grid on;
xlabel('Step');
ylabel('Error [m]');
title('Position error');
xticks(steps);

% Kalman gain.
nexttile;
plot(2:N, K_diag(1,2:end), 'o-', ...
     2:N, K_diag(2,2:end), 's-', 'LineWidth', 1.5);
grid on;
xlabel('Step');
ylabel('Kalman gain');
title('Kalman gain');
legend('K_x', 'K_y', 'Location', 'best');
xticks(steps);
ylim([0 1]);

% Uncertainty.
nexttile;
plot(steps, P_diag(1,:), 'o-', steps, P_diag(2,:), 's-', ...
     steps, R_diag(1,:), 'o--', steps, R_diag(2,:), 's--', ...
     'LineWidth', 1.2);
grid on;
xlabel('Step');
ylabel('Variance [m^2]');
title('Uncertainty');
legend('P_x', 'P_y', 'R_x', 'R_y', 'Location', 'best');
xticks(steps);

sgtitle('2D Kalman filter');