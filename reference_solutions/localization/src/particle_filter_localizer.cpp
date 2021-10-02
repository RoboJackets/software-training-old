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

#include "particle_filter_localizer.hpp"
#include <angles/angles.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <memory>
#include <vector>
#include <utility>
#include "aruco_sensor_model.hpp"
// BEGIN STUDENT CODE
#include "odometry_sensor_model.hpp"
// END STUDENT CODE

namespace localization
{

using namespace std::chrono_literals;

ParticleFilterLocalizer::ParticleFilterLocalizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("particle_filter_localizer", options),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(this),
  motion_model_(*this)
{
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&ParticleFilterLocalizer::CmdCallback, this, std::placeholders::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/pose_estimate", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/particles", 1);

  num_particles_ = declare_parameter<int>("num_particles", 300);
  declare_parameter<double>("resample_threshold", 0.1);
  declare_parameter<double>("low_percentage_particles_to_drop", 0.1);

  initial_range_.min_x = declare_parameter<double>("initial_range.min_x", -0.6);
  initial_range_.max_x = declare_parameter<double>("initial_range.max_x", 0.6);
  initial_range_.min_y = declare_parameter<double>("initial_range.min_y", -0.3);
  initial_range_.max_y = declare_parameter<double>("initial_range.max_y", 0.3);

  InitializeParticles();

  const double state_update_rate = declare_parameter<double>("state_update_rate", 10);
  pose_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / state_update_rate),
    std::bind(&ParticleFilterLocalizer::CalculateStateAndPublish, this));

  const double resample_rate = declare_parameter<double>("resample_rate", 10);
  resample_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / resample_rate),
    std::bind(&ParticleFilterLocalizer::ResampleParticles, this));

  sensor_models_.push_back(std::make_unique<ArucoSensorModel>(*this));
  // BEGIN STUDENT CODE
  sensor_models_.push_back(std::make_unique<OdometrySensorModel>(*this));
  // END STUDENT CODE
}

void ParticleFilterLocalizer::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  motion_model_.updateParticles(particles_, msg, now());
}

void ParticleFilterLocalizer::InitializeParticles()
{
  particles_.resize(num_particles_);
  std::generate(
    particles_.begin(), particles_.end(),
    std::bind(&ParticleFilterLocalizer::GenerateNewParticle, this));
  NormalizeWeights();
}

Particle ParticleFilterLocalizer::GenerateNewParticle()
{
  // uniform_noise_.Sample() to get a uniform sample
  Particle p;
  p.x = initial_range_.min_x + uniform_noise_.Sample() *
    (initial_range_.max_x - initial_range_.min_x);
  p.y = initial_range_.min_y + uniform_noise_.Sample() *
    (initial_range_.max_y - initial_range_.min_y);
  p.yaw = (uniform_noise_.Sample() * M_2_PI) - M_1_PI;
  p.x_vel = 0.0;
  p.yaw_vel = 0.0;
  return p;
}

void ParticleFilterLocalizer::NormalizeWeights()
{
  // run through list of particles and normalize the weights

  auto index_to_drop = std::round(
    get_parameter(
      "low_percentage_particles_to_drop").as_double() * num_particles_);
  std::nth_element(
    particles_.begin(), particles_.begin() + index_to_drop, particles_.end(),
    [](const Particle & lhs, const Particle & rhs) {return lhs.weight < rhs.weight;});

  double smallest_weight_to_allow = particles_.at(index_to_drop).weight;

  double normalizer = 0;
  for (Particle & particle : particles_) {
    if (particle.weight < smallest_weight_to_allow) {
      particle.weight = 0;
    }
    normalizer += particle.weight;
  }
  // if all particles are zero
  double resample_threshold = get_parameter("resample_threshold").as_double();
  if (normalizer < resample_threshold) {
    RCLCPP_INFO(
      get_logger(), "resampling particles from initial distribution since normalizer is low %f\n",
      normalizer / num_particles_);
    std::generate_n(
      particles_.begin(), num_particles_,
      std::bind(&ParticleFilterLocalizer::GenerateNewParticle, this));
    normalizer = num_particles_;
  }
  if (normalizer == 0) {
    RCLCPP_INFO(get_logger(), "normalizer is zero%f\n", normalizer);
    normalizer = 1;
  }
  search_weights_.clear();
  double running_sum = 0;
  min_weight_ = 1.0;
  max_weight_ = 0.0;
  for (Particle & particle : particles_) {
    particle.weight /= normalizer;
    min_weight_ = std::min(min_weight_, particle.weight);
    max_weight_ = std::max(max_weight_, particle.weight);
    search_weights_.push_back(running_sum);
    running_sum += particle.weight;
  }
}

Particle ParticleFilterLocalizer::CalculateEstimate()
{
  Particle estimate;
  estimate.weight = 0.0;
  estimate = std::accumulate(
    particles_.begin(), particles_.end(), Particle{}, [](const auto & total, const auto & p) {
      return total + p.Weighted();
    });

  double yaw_alternative = std::accumulate(
    particles_.begin(), particles_.end(), 0.0, [](const auto & total, const auto & p) {
      return total + (angles::normalize_angle_positive(p.yaw) * p.weight);
    });

  // if very close to the discontinuity of pi--pi, use the one with
  // a discontinuity at a different place
  if (std::abs(std::abs(estimate.yaw) - M_PI) < 1.0) {
    estimate.yaw = yaw_alternative;
  }

  estimate.yaw = angles::normalize_angle(estimate.yaw);

  return estimate;
}


Particle ParticleFilterLocalizer::CalculateCovariance(const Particle & estimate)
{
  Particle cov;
  double cov_normalizer = 0;
  for (const Particle & particle : particles_) {
    cov.x += pow(estimate.x - particle.x, 2) * particle.weight;
    cov.y += pow(estimate.y - particle.y, 2) * particle.weight;
    double yaw_error = estimate.yaw - particle.yaw;
    yaw_error = angles::normalize_angle(yaw_error);
    cov.yaw += pow(yaw_error, 2) * particle.weight;
    cov.x_vel += pow(estimate.x_vel - particle.x_vel, 2) * particle.weight;
    cov.yaw_vel += pow(estimate.yaw_vel - particle.yaw_vel, 2) * particle.weight;

    cov_normalizer += pow(particle.weight, 2);
  }

  cov.x /= (1 - cov_normalizer);
  cov.y /= (1 - cov_normalizer);
  cov.yaw /= (1 - cov_normalizer);
  cov.x_vel /= (1 - cov_normalizer);
  cov.yaw_vel /= (1 - cov_normalizer);

  return cov;
}

void ParticleFilterLocalizer::ResampleParticles()
{
  // resample particles uniformly across the entire state space
  std::vector<Particle> new_particles;
  for (int i = 0; i < num_particles_; i++) {
    // get a target values 0-1
    double target_val = uniform_noise_.Sample();
    int left_idx = 0;
    int right_idx = num_particles_ - 1;
    int cur_index = (left_idx + right_idx) / 2;

    bool found_sol = false;
    // binary search the summed weights to find the particle
    while (left_idx <= right_idx && !found_sol) {
      cur_index = (left_idx + right_idx) / 2;
      double cur_val = search_weights_[cur_index];

      if (cur_index == num_particles_ - 1 || cur_index == 0) {
        found_sol = true;
      } else if (target_val >= search_weights_[cur_index - 1] && target_val <= cur_val) {
        // if we have found our target, return it
        found_sol = true;
      } else if (cur_val < target_val) {
        // if we have a current smaller than target search to the right
        left_idx = cur_index + 1;
      } else if (cur_val > target_val) {
        // if we have a current smaller than target search to the left
        right_idx = cur_index - 1;
      } else {
        RCLCPP_INFO(get_logger(), "Invalid search weights or cur_val %f\n", cur_val);
      }
    }
    new_particles.push_back(particles_[cur_index]);
  }
  particles_ = new_particles;
}

void ParticleFilterLocalizer::CalculateAllParticleWeights(const rclcpp::Time & current_time)
{
  for (Particle & particle : particles_) {
    CalculateParticleWeight(particle, current_time);
  }
  NormalizeWeights();
}


void ParticleFilterLocalizer::CalculateParticleWeight(
  Particle & particle,
  const rclcpp::Time & current_time)
{
  double log_probability = 0.0;
  for (const auto & model : sensor_models_) {
    if (model->IsMeasurementAvailable(current_time)) {
      log_probability += -0.5 * model->ComputeLogProb(particle);
      log_probability -= model->ComputeLogNormalizer();
    }
  }
  particle.weight = exp(log_probability);
}

void ParticleFilterLocalizer::CalculateStateAndPublish()
{
  // create a weighted average of particles based off of the weights and publish it

  const rclcpp::Time current_time = now();

  CalculateAllParticleWeights(current_time);

  const Particle best_estimate_particle = CalculateEstimate();

  const Particle covariance = CalculateCovariance(best_estimate_particle);

  PublishEstimateOdom(best_estimate_particle, covariance, current_time);

  PublishEstimateTF(best_estimate_particle, current_time);

  PublishParticleVisualization();
}

void ParticleFilterLocalizer::PublishEstimateOdom(
  const Particle & estimate,
  const Particle & covariance,
  const rclcpp::Time & current_time)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "map";
  odom_msg.header.stamp = current_time;
  odom_msg.pose.pose.position.x = estimate.x;
  odom_msg.pose.pose.position.y = estimate.y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, -estimate.yaw);
  odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);
  odom_msg.twist.twist.linear.x = estimate.x_vel;
  odom_msg.twist.twist.angular.z = estimate.yaw_vel;

  odom_msg.pose.covariance[0] = covariance.x;
  odom_msg.pose.covariance[7] = covariance.y;
  odom_msg.pose.covariance[35] = covariance.yaw;

  odom_msg.twist.covariance[0] = covariance.x_vel;
  odom_msg.twist.covariance[35] = covariance.yaw_vel;
  odom_pub_->publish(odom_msg);
}

void ParticleFilterLocalizer::PublishEstimateTF(
  const Particle & estimate,
  const rclcpp::Time & current_time)
{
  tf2::Transform map_odom_transform;
  if (tf_buffer_.canTransform("base_footprint", "odom", tf2::TimePointZero)) {
    // Get base_footprint -> odom transform from TF
    const auto base_odom_transform_msg = tf_buffer_.lookupTransform(
      "base_footprint", "odom",
      tf2::TimePointZero);
    tf2::Transform base_odom_transform;
    tf2::fromMsg(base_odom_transform_msg.transform, base_odom_transform);

    // Set map -> base_footprint transform to our estimated pose
    tf2::Transform map_base_transform;
    map_base_transform.setOrigin(
      tf2::Vector3(
        estimate.x, estimate.y,
        0));
    map_base_transform.setRotation(
      tf2::Quaternion(
        tf2::Vector3(0, 0, 1),
        -estimate.yaw));

    // Set map -> odom transform by compining map -> base_footprint -> odom
    map_odom_transform.mult(map_base_transform, base_odom_transform);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(), 1.0, "Waiting for odom->base_footprint transform...");
    // Use identity transform until odom is available
    map_odom_transform.setIdentity();
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = current_time;
  transform.child_frame_id = "odom";
  transform.transform = tf2::toMsg(map_odom_transform);
  tf_broadcaster_.sendTransform(transform);
}

void ParticleFilterLocalizer::PublishParticleVisualization()
{
  if (marker_pub_->get_subscription_count() == 0) {
    // Don't bother building visualization if nobody's subscribed
    return;
  }

  // publish visualization of all particles likelihood
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.stamp = now();
  marker.id = 10;

  marker.color.r = 1.0;
  marker.color.a = 1.0;

  marker.colors.resize(num_particles_);
  for (int i = 0; i < num_particles_; i++) {
    geometry_msgs::msg::Point front_point, right_point, left_point;

    // create three points based off of the particle
    // The marker uses three points to create a triangle between them
    double x = particles_[i].x;
    double y = particles_[i].y;
    double yaw = particles_[i].yaw;

    front_point.x = x + 0.1 * cos(yaw);
    front_point.y = y - 0.1 * sin(yaw);

    right_point.x = x + 0.025 * cos(yaw + M_PI_2);
    right_point.y = y - 0.025 * sin(yaw + M_PI_2);

    left_point.x = x + 0.025 * cos(yaw - M_PI_2);
    left_point.y = y - 0.025 * sin(yaw - M_PI_2);

    marker.points.push_back(front_point);
    marker.points.push_back(right_point);
    marker.points.push_back(left_point);

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // sets the color intensity based off of the max weight particle
    marker.colors[i].r = (particles_[i].weight / max_weight_);
    marker.colors[i].g = 0;
    marker.colors[i].b = 0;
    marker.colors[i].a = 0.7;
  }

  marker_pub_->publish(marker);
}


}  // namespace localization

RCLCPP_COMPONENTS_REGISTER_NODE(localization::ParticleFilterLocalizer)
