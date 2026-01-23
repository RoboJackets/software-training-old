#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace mineral_deposit_tracking
{

template<int StateSize>
class KalmanFilter
{
public:
  // Type aliases for convenience
  using VectorType = Eigen::Matrix<double, StateSize, 1>;
  using MatrixType = Eigen::Matrix<double, StateSize, StateSize>;

  // Constructor
  KalmanFilter(
    const MatrixType & transition_matrix,
    const MatrixType & process_covariance,
    const MatrixType & observation_matrix)
  : transition_matrix_(transition_matrix),
    process_covariance_(process_covariance),
    observation_matrix_(observation_matrix),
    estimate_(VectorType::Zero()),
    estimate_covariance_(MatrixType::Identity() * 500)
  {
  }

  // Getters
  const VectorType & GetEstimate() const
  {
    return estimate_;
  }

  const MatrixType & GetEstimateCovariance() const
  {
    return estimate_covariance_;
  }

  // Reset function to set initial state
  void Reset(const VectorType & initial_state, const MatrixType & initial_covariance)
  {
    estimate_ = initial_state;
    estimate_covariance_ = initial_covariance;
  }

  // Prediction Step (Time Update)
  // x = A * x
  // P = A * P * A_transpose + Q
  void TimeUpdate()
  {
    estimate_ = transition_matrix_ * estimate_;
    estimate_covariance_ = (transition_matrix_ * estimate_covariance_ * transition_matrix_.transpose()) + process_covariance_;
  }

  // Correction Step (Measurement Update)
  void MeasurementUpdate(const VectorType & measurement, const MatrixType & measurement_covariance)
  {
    // 1. Calculate the innovation covariance (S)
    // S = H * P * H_transpose + R
    const MatrixType innovation_covariance = (observation_matrix_ * estimate_covariance_ * observation_matrix_.transpose()) + measurement_covariance;

    // 2. Calculate the Kalman gain (K)
    // K = P * H_transpose * S_inverse
    const MatrixType gain = estimate_covariance_ * observation_matrix_.transpose() * innovation_covariance.inverse();

    // 3. Update the state estimate (x)
    // x = x + K * (y - H * x)
    estimate_ = estimate_ + (gain * (measurement - (observation_matrix_ * estimate_)));

    // 4. Compute temporary variable (T) for readability
    // T = I - K * H
    const MatrixType tmp = MatrixType::Identity() - (gain * observation_matrix_);

    // 5. Update the estimate covariance (P) using the Joseph form for numerical stability
    // P = T * P * T_transpose + K * R * K_transpose
    estimate_covariance_ = (tmp * estimate_covariance_ * tmp.transpose()) + (gain * measurement_covariance * gain.transpose());
  }

private:
  // Model Constants
  const MatrixType transition_matrix_;    // A
  const MatrixType process_covariance_;   // Q
  const MatrixType observation_matrix_;   // H

  // State Variables
  VectorType estimate_;                   // x
  MatrixType estimate_covariance_;        // P
};

}  // namespace mineral_deposit_tracking

#endif  // KALMAN_FILTER_HPP_