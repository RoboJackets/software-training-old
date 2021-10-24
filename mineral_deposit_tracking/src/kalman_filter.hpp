// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace mineral_deposit_tracking
{

template<int StateSize>
class KalmanFilter
{
public:
  using VectorType = Eigen::Matrix<double, StateSize, 1>;
  using MatrixType = Eigen::Matrix<double, StateSize, StateSize>;

  KalmanFilter(
    const MatrixType & transition_matrix,
    const MatrixType & process_covariance,
    const MatrixType & observation_matrix)
  : transition_matrix_(transition_matrix),
    process_covariance_(process_covariance),
    observation_matrix_(observation_matrix),
    estimate_(0.0, 0.0),
    estimate_covariance_(MatrixType::Identity() * 500)
  {
  }

  void TimeUpdate()
  {
    estimate_ = transition_matrix_ * estimate_;
    estimate_covariance_ = transition_matrix_ * estimate_covariance_ *
      transition_matrix_.transpose() + process_covariance_;
  }

  void MeasurementUpdate(
    const VectorType & measurement,
    const MatrixType & measurement_covariance)
  {
    const MatrixType innovation_covariance =
      (observation_matrix_ * estimate_covariance_ * observation_matrix_.transpose()) +
      measurement_covariance;

    const MatrixType gain = estimate_covariance_ * observation_matrix_.transpose() *
      innovation_covariance.inverse();

    estimate_ = estimate_ + (gain * (measurement - (observation_matrix_ * estimate_)));

    const MatrixType tmp = MatrixType::Identity() - (gain * observation_matrix_);

    estimate_covariance_ = (tmp * estimate_covariance_ * tmp.transpose()) +
      (gain * measurement_covariance * gain.transpose());
  }

  void Reset(const VectorType & initial_state, const MatrixType & initial_covariance)
  {
    estimate_ = initial_state;
    estimate_covariance_ = initial_covariance;
  }

  const VectorType & GetEstimate()
  {
    return estimate_;
  }

  const MatrixType & GetEstimateCovariance()
  {
    return estimate_covariance_;
  }

private:
  const MatrixType transition_matrix_;
  const MatrixType process_covariance_;
  const MatrixType observation_matrix_;
  VectorType estimate_;
  MatrixType estimate_covariance_;
};

}  // namespace mineral_deposit_tracking

#endif  // KALMAN_FILTER_HPP_
