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

class KalmanFilter
{
public:
  KalmanFilter(
    const Eigen::Matrix2d & transition_matrix,
    const Eigen::Matrix2d & process_covariance,
    const Eigen::Matrix2d & observation_matrix);

  void TimeUpdate();

  void MeasurementUpdate(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & measurement_covariance);

  void Reset(const Eigen::Vector2d & initial_state, const Eigen::Matrix2d & initial_covariance);

  const Eigen::Vector2d & GetEstimate();

  const Eigen::Matrix2d & GetEstimateCovariance();

private:
  const Eigen::Matrix2d transition_matrix_;
  const Eigen::Matrix2d process_covariance_;
  const Eigen::Matrix2d observation_matrix_;
  Eigen::Vector2d estimate_;
  Eigen::Matrix2d estimate_covariance_;
};

}  // namespace mineral_deposit_tracking

#endif  // KALMAN_FILTER_HPP_
