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

#include "particle_filter_localizer.h"

namespace localization
{

using namespace std::chrono_literals;

ParticleFilterLocalizer::ParticleFilterLocalizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("particle_filter_localizer", options),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(*this),
  motion_model_(noise_, this), aruco_model_(this), odom_model_(this)
{
  tag_sub_ = create_subscription<stsl_interfaces::msg::TagArray>(
    "/tags", 1, std::bind(&ParticleFilterLocalizer::TagCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ParticleFilterLocalizer::OdometryCallback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, std::bind(&ParticleFilterLocalizer::ImuCallback, this, std::placeholders::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/pose_estimate", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/particles", 1);

  this->declare_parameter<int>("num_particles", 100);
  this->get_parameter("num_particles", num_particles_);

  this->declare_parameter<std::vector<double>>("min_init_vals", {-1.0, -1.0, -1.0, 0.0, 0.0});
  std::vector<double> min_vals;
  this->get_parameter("min_init_vals", min_vals);
  min_.x = min_vals[0];
  min_.y = min_vals[1];
  min_.yaw = min_vals[2];
  min_.vx = min_vals[3];
  min_.vy = min_vals[4];

  this->declare_parameter<std::vector<double>>("max_init_vals", {1.0, 1.0, 1.0, 0.0, 0.0});
  std::vector<double> max_vals;
  this->get_parameter("max_init_vals", max_vals);
  max_.x = max_vals[0];
  max_.y = max_vals[1];
  max_.yaw = max_vals[2];
  max_.vx = max_vals[3];
  max_.vy = max_vals[4];

  // initialize particle filter
  for(int i = 0; i < num_particles_; i++) {
    particles_.push_back(InitializeParticle());
  }
  // TODO move to generate
  //particles_ = std::vector<Particle>(num_particles_);
  //std::generate(particles_.begin(), particles_.end(), std::mem_fn(this->InitializeParticle));
  NormalizeWeights();

  pose_timer_ = create_wall_timer(
          0.05s, std::bind(&ParticleFilterLocalizer::CalculateStateAndPublish, this));
}

void ParticleFilterLocalizer::TagCallback(const stsl_interfaces::msg::TagArray::SharedPtr msg)
{
  aruco_model_.UpdateMeasurement(msg);
}

void ParticleFilterLocalizer::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_model_.UpdateMeasurement(msg);
}


void ParticleFilterLocalizer::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // updates the particles based off of the IMU message
  motion_model_.updateParticles(particles_, msg);
}

//void ParticleFilterLocalizer::PublishPose()
//{
//  geometry_msgs::msg::TransformStamped transform;
//  transform.header.frame_id = "map";
//  transform.header.stamp = now();
//  transform.child_frame_id = "odom";
//  transform.transform.rotation.w = 1;
//  transform.transform.rotation.x = 0;
//  transform.transform.rotation.y = 0;
//  transform.transform.rotation.z = 0;
//  transform.transform.translation.x = 0;
//  transform.transform.translation.y = 0;
//  transform.transform.translation.z = 0;
//  tf_broadcaster_.sendTransform(transform);
//}

Particle ParticleFilterLocalizer::InitializeParticle() {
  Particle p;
  p.x = min_.x + noise_->sampleGaussian()*(max_.x - min_.x);
  p.y = min_.y + noise_->sampleGaussian()*(max_.y - min_.y);
  p.yaw = min_.yaw + noise_->sampleGaussian()*(max_.yaw - min_.yaw);
  p.vx = min_.vx + noise_->sampleGaussian()*(max_.vx - min_.vx);
  p.vy = min_.vy + noise_->sampleGaussian()*(max_.vy - min_.vy);
  return p;
}

void ParticleFilterLocalizer::NormalizeWeights()
{
  // run through list of particles and normalize the weights

  // TODO move to accumulate
  double normalizer = 0;
  for(Particle & particle : particles_)
  {
    normalizer += particle.weight;
  }
  // if all particles are zero
  if (normalizer == 0){
    std::cerr << "all particle weights are zero" << std::endl;
    exit(-1);
  }
  search_weights_.clear();
  double running_sum = 0;
  min_weight_ = 1.0;
  max_weight_ = 0.0;
  for(Particle & particle : particles_) {
    particle.weight /= normalizer;
    if (particle.weight < min_weight_)
    {
      min_weight_ = particle.weight;
    }
    if (particle.weight > max_weight_)
    {
      max_weight_ = particle.weight;
    }
    search_weights_.push_back(running_sum);
    running_sum += particle.weight;
  }
}

void ParticleFilterLocalizer::ResampleParticles()
{
  // resample particles uniformly across the entire state space
  std::vector<Particle> new_particles;
  for(int i = 0; i < particles_.size(); i++) {
    // get a target values 0-1
    double target_val = noise_->sampleUniform();
    int left_idx = 0;
    int right_idx = particles_.size()-1;
    int cur_index = (left_idx+right_idx)/2;

    bool found_sol = false;
    // binary search the summed weights to find the particle
    while(left_idx <= right_idx && !found_sol) {
      cur_index = (left_idx+right_idx)/2;
      double cur_val = search_weights_[cur_index];

      if (target_val >= search_weights_[cur_index-1] && target_val <= cur_val)
      {
        // if we have found our target, return it
        found_sol = true;
      } else if (cur_val < target_val)
      {
        // if we have a current smaller than target search to the right
        left_idx = cur_index+1;
      } else if (cur_val > target_val)
      {
        // if we have a current smaller than target search to the left
        right_idx = cur_index-1;
      }
    }
    new_particles.push_back(particles_[cur_index]);
  }
}

void ParticleFilterLocalizer::CalculateStateAndPublish()
{
  // create a weighted average of particles based off of the weights and publish it

  SensorModel::ComputeLogProbs(particles_, this->now(), aruco_model_, odom_model_);

  NormalizeWeights();

  Particle best_estimate_particle;
  for(Particle & particle : particles_)
  {
    best_estimate_particle.x += particle.x * particle.weight;
    best_estimate_particle.y += particle.y * particle.weight;
    best_estimate_particle.yaw += particle.yaw * particle.weight;
    best_estimate_particle.vx += particle.vx * particle.weight;
    best_estimate_particle.vy += particle.vy * particle.weight;
  }

  Particle cov;
  for(Particle & particle : particles_)
  {

    cov.x += pow(best_estimate_particle.x - particle.x, 2)*particle.weight;
    cov.y += pow(best_estimate_particle.y - particle.y, 2)*particle.weight;
    cov.yaw += pow(best_estimate_particle.yaw - particle.yaw, 2)*particle.weight;
    cov.vx += pow(best_estimate_particle.vx - particle.vx, 2)*particle.weight;
    cov.vy += pow(best_estimate_particle.vy - particle.vy, 2)*particle.weight;
  }

  // TODO use estimated pose to determine the new transform from odom to calculated odom position
  // TODO how to publish uncertainty

  PublishParticleVisualization();
}

void ParticleFilterLocalizer::PublishParticleVisualization()
{
  // TODO check for subscriber

  // TODO publish visualization of all particles likelihood
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.stamp = this->now();
  marker.id = 10;

  marker.colors.resize(num_particles_);
  for(int i = 0; i < num_particles_; i++) {
    geometry_msgs::msg::Point front_point, right_point, left_point;
    // TODO create three points based off of the particle
    double x = particles_[i].x;
    double y = particles_[i].y;
    double yaw = particles_[i].yaw;

    front_point.x = x + 0.1*cos(yaw) + 0.1*sin(yaw);
    front_point.y = y - 0.1*sin(yaw) + 0.1*cos(yaw);

    right_point.x = x + 0.05*cos(yaw+M_PI_2) + 0.05*sin(yaw+M_PI_2);
    right_point.y = y - 0.05*sin(yaw+M_PI_2) + 0.05*cos(yaw+M_PI_2);

    left_point.x = x + 0.05*cos(yaw-M_PI_2) + 0.05*sin(yaw-M_PI_2);
    left_point.y = y - 0.05*sin(yaw-M_PI_2) + 0.05*cos(yaw-M_PI_2);

    marker.points.push_back(front_point);
    marker.points.push_back(right_point);
    marker.points.push_back(left_point);

    marker.colors[i].r = 255 * (particles_[i].weight / max_weight_);
    marker.colors[i].g = 0;
    marker.colors[i].b = 0;
    marker.colors[i].a = 0.7;
  }

  marker_pub_->publish(marker);

}


}  // namespace localization

RCLCPP_COMPONENTS_REGISTER_NODE(localization::ParticleFilterLocalizer)
