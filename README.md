# Kalman Filter Learning in MATLAB

Simple MATLAB examples for learning the basic Kalman-filter steps. They are designed for students and new learners.

## Examples

### `KF_1.m` — Scalar example

Estimates the constant weight of an object from five noisy measurements.

Each measurement has a different variance, so the filter does not trust every measurement equally.

### `KF_2.m` — Two-dimensional example

Estimates the constant position \([x; y]\) of an object.

Each measurement and estimate is a two-element vector. Their uncertainties are represented by 2-by-2 covariance matrices.

The filtering process is the same as in the scalar example, but numbers are replaced by vectors and matrices.

### `Kalman_Filter_Explained.pptx` — Presentation

Explains the scalar and two-dimensional examples using equations and worked numerical calculations.

## Kalman-filter steps

For each new measurement, the filter:

1. Predicts the state and its uncertainty.
2. Compares the measurement with the prediction.
3. Calculates the Kalman gain.
4. Updates the estimate and its uncertainty.

A high Kalman gain gives more weight to the measurement. A low gain gives more weight to the prediction.

## Main symbols

| Symbol | Meaning |
| --- | --- |
| `z` | Measurement |
| `R` | Measurement variance or covariance |
| `x_pred` | Predicted state |
| `P_pred` | Predicted uncertainty |
| `K` | Kalman gain |
| `x_est` | Updated state estimate |
| `P_est` | Updated estimate uncertainty |
| `Q` | Process-noise variance or covariance |

In both examples, `Q = 0` because the true weight or position is assumed to remain constant.

The first measurement initialises the filter. Therefore, no prediction, innovation or Kalman gain is calculated at step 1.

## Recommended order

1. Read the presentation for a visual explanation.
2. Run `KF_1.m` and follow the scalar calculations.
3. Run `KF_2.m` and compare the matrix calculations.
4. Change the measurements or uncertainties and observe the results.

## Requirements

- A recent version of MATLAB
- No additional toolboxes or external files are required