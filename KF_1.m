% ============================================================
% SCALAR KALMAN FILTER EXAMPLE
% Estimating the constant weight of an object
% ============================================================

clear;
clc;
close all;

%% ------------------------------------------------------------
% 1. Define the example
% ------------------------------------------------------------

% True weight. This is known only because this is a simulation.
x_true = 10;                    % [kg]

% Three measured weights.
z = [8, 12, 9];                % [kg]

% Measurement variances.
% A smaller value means a more reliable measurement.
R = [4, 1, 4];                 % [kg^2]

% Process-noise variance.
% Q = 0 because the true weight is assumed to remain constant.
Q = 0;                         % [kg^2]

N_measurements = numel(z);
disp (N_measurements)

% Storage for results.
x_est     = zeros(1, N_measurements);
P_est     = zeros(1, N_measurements);

