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

#include "kalman_filter.hpp"

namespace mineral_deposit_tracking
{

KalmanFilter::KalmanFilter(
  const Eigen::Matrix2d & transition_matrix,
  const Eigen::Matrix2d & process_covariance,
  const Eigen::Matrix2d & observation_matrix)
: transition_matrix_(transition_matrix),
  process_covariance_(process_covariance),
  observation_matrix_(observation_matrix),
  estimate_(0.0, 0.0),
  estimate_covariance_(Eigen::Matrix2d::Identity() * 500)
{
}

void KalmanFilter::TimeUpdate()
{
  estimate_ = transition_matrix_ * estimate_;
  estimate_covariance_ = transition_matrix_ * estimate_covariance_ *
    transition_matrix_.transpose() + process_covariance_;
}

void KalmanFilter::MeasurementUpdate(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & measurement_covariance)
{
  const Eigen::Matrix2d innovation_covariance =
    (observation_matrix_ * estimate_covariance_ * observation_matrix_.transpose()) +
    measurement_covariance;

  const Eigen::Matrix2d gain = estimate_covariance_ * observation_matrix_.transpose() *
    innovation_covariance.inverse();

  estimate_ = estimate_ + (gain * (measurement - (observation_matrix_ * estimate_)));

  const Eigen::Matrix2d tmp = Eigen::Matrix2d::Identity() - (gain * observation_matrix_);

  estimate_covariance_ = (tmp * estimate_covariance_ * tmp.transpose()) +
    (gain * measurement_covariance * gain.transpose());
}

void KalmanFilter::Reset(
  const Eigen::Vector2d & initial_state,
  const Eigen::Matrix2d & initial_covariance)
{
  estimate_ = initial_state;
  estimate_covariance_ = initial_covariance;
}

const Eigen::Vector2d & KalmanFilter::GetEstimate()
{
  return estimate_;
}

const Eigen::Matrix2d & KalmanFilter::GetEstimateCovariance()
{
  return estimate_covariance_;
}

}  // namespace mineral_deposit_tracking
