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
  noise_(std::make_shared<ParticleNoise>()), motion_model_(noise_, this)
{
  std::cout << "out" <<std::endl;
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10, std::bind(&ParticleFilterLocalizer::CmdCallback, this, std::placeholders::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/pose_estimate", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/particles", 1);

  num_particles_ = declare_parameter<int>("num_particles", 300);
  declare_parameter<double>("resample_threshold", 0.1);
  declare_parameter<double>("low_percentage_particles_to_drop", 0.1);

  std::vector<double> min_vals = declare_parameter<std::vector<double>>("min_init_vals", {-0.6, -0.3, -M_PI, 0.0, 0.0});
  min_.x = min_vals[0];
  min_.y = min_vals[1];
  min_.yaw = min_vals[2];
  min_.vx = min_vals[3];
  min_.yaw_rate = min_vals[4];

  std::vector<double> max_vals = declare_parameter<std::vector<double>>("max_init_vals", {0.6, 0.3, M_PI, 0.0, 0.0});
  max_.x = max_vals[0];
  max_.y = max_vals[1];
  max_.yaw = max_vals[2];
  max_.vx = max_vals[3];
  max_.yaw_rate = max_vals[4];

  // initialize particle filter
  particles_ = std::vector<Particle>(num_particles_);
  std::generate_n(particles_.begin(), num_particles_, std::bind(&ParticleFilterLocalizer::InitializeParticle, this));
  NormalizeWeights();

  double state_update_rate = declare_parameter<double>("state_update_rate", 10);
  pose_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0/state_update_rate),
          std::bind(&ParticleFilterLocalizer::CalculateStateAndPublish, this));

  double resample_rate = declare_parameter<double>("resample_rate", 10);
  resample_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0/resample_rate),
          std::bind(&ParticleFilterLocalizer::ResampleParticles, this));

  // check what is enabled
  sensor_models_.push_back(std::move(std::make_unique<ArucoSensorModel>(this)));
  sensor_models_.push_back(std::move(std::make_unique<OdometrySensorModel>(this)));
}

void ParticleFilterLocalizer::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // updates the particles based off of the IMU message
  motion_model_.updateParticles(particles_, msg, now());
}

Particle ParticleFilterLocalizer::InitializeParticle() {
  // noise_->sampleUniform() to get a uniform sample
  // BEGIN STUDENT CODE
  Particle p;
  p.x = min_.x + noise_->sampleUniform()*(max_.x - min_.x);
  p.y = min_.y + noise_->sampleUniform()*(max_.y - min_.y);
  p.yaw = min_.yaw + noise_->sampleUniform()*(max_.yaw - min_.yaw);
  p.vx = min_.vx + noise_->sampleUniform()*(max_.vx - min_.vx);
  p.yaw_rate = min_.yaw_rate + noise_->sampleUniform()*(max_.yaw_rate - min_.yaw_rate);
  return p;
  // END STUDENT CODE
}

void ParticleFilterLocalizer::NormalizeWeights()
{
  // run through list of particles and normalize the weights

  // TODO should be just finding pivot
  std::sort(particles_.begin(), particles_.end(),
            [](const Particle& lhs, const Particle& rhs) {return lhs.weight < rhs.weight;});

  double low_percentage_particles_to_drop = get_parameter("low_percentage_particles_to_drop").as_double();
  double smallest_weight_to_allow =
          particles_.at(num_particles_*low_percentage_particles_to_drop).weight;

  double normalizer = 0;
  for(Particle & particle : particles_)
  {
    if (particle.weight < smallest_weight_to_allow)
    {
      particle.weight = 0;
    }
    normalizer += particle.weight;
  }
  // if all particles are zero
  double resample_threshold = get_parameter("resample_threshold").as_double();
  if (normalizer < resample_threshold)
  {
    RCLCPP_INFO(get_logger(), "resampling particles from initial distribution since normalizer is low %f\n", normalizer/num_particles_);
    std::generate_n(particles_.begin(), num_particles_, std::bind(&ParticleFilterLocalizer::InitializeParticle, this));
    normalizer = num_particles_;
  }
  if(normalizer == 0)
  {
    RCLCPP_INFO(get_logger(), "normalizer is zero%f\n", normalizer);
    normalizer = 1;
  }
  search_weights_.clear();
  double running_sum = 0;
  min_weight_ = 1.0;
  max_weight_ = 0.0;
  for(Particle & particle : particles_) {
    particle.weight /= normalizer;
    min_weight_ = std::min(min_weight_, particle.weight);
    max_weight_ = std::max(max_weight_, particle.weight);
    search_weights_.push_back(running_sum);
    running_sum += particle.weight;
  }
}

void ParticleFilterLocalizer::ResampleParticles()
{
  // resample particles uniformly across the entire state space
  std::vector<Particle> new_particles;
  for(int i = 0; i < num_particles_; i++) {
    // get a target values 0-1
    double target_val = noise_->sampleUniform();
    int left_idx = 0;
    int right_idx = num_particles_-1;
    int cur_index = (left_idx+right_idx)/2;

    bool found_sol = false;
    // binary search the summed weights to find the particle
    while(left_idx <= right_idx && !found_sol) {
      cur_index = (left_idx+right_idx)/2;
      double cur_val = search_weights_[cur_index];

      if (cur_index == num_particles_ - 1 || cur_index == 0)
      {
        found_sol = true;
      } else if (target_val >= search_weights_[cur_index-1] && target_val <= cur_val)
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
      } else {
        RCLCPP_INFO(get_logger(), "Invalid search weights or cur_val %f\n", cur_val);
      }
    }
    new_particles.push_back(particles_[cur_index]);
  }
  particles_ = new_particles;
}

void ParticleFilterLocalizer::ComputeLogProbs()
{
  auto cur_time = this->now();
  for(Particle & particle : particles_)
  {
    double log_prob = 0;
    for(const std::unique_ptr<SensorModel>& sensor_model : sensor_models_)
    {
      if (sensor_model->IsMeasUpdateValid(cur_time))
      {
        log_prob += -0.5*sensor_model->ComputeLogProb(particle);
        log_prob -= sensor_model->ComputeLogNormalizer();
      }
    }
    particle.weight = exp(log_prob);
  }
}

void ParticleFilterLocalizer::CalculateStateAndPublish()
{
  // create a weighted average of particles based off of the weights and publish it

  ComputeLogProbs();

  NormalizeWeights();

  Particle best_estimate_particle;
  double yaw_sum_2 = 0;
  for(const Particle & particle : particles_)
  {
    best_estimate_particle.x += particle.x * particle.weight;
    best_estimate_particle.y += particle.y * particle.weight;
    // this gives zero if we are facing pi
    best_estimate_particle.yaw += particle.yaw * particle.weight;
    best_estimate_particle.vx += particle.vx * particle.weight;
    best_estimate_particle.yaw_rate += particle.yaw_rate * particle.weight;

    double yaw = particle.yaw;
    if (yaw < 0)
    {
      yaw += 2*M_PI;
    }
    yaw_sum_2 += yaw * particle.weight;
  }
  if (abs(yaw_sum_2 - M_PI) < 1.0){
    best_estimate_particle.yaw = yaw_sum_2;
  }
  while(best_estimate_particle.yaw >= M_PI)
  {
    best_estimate_particle.yaw -= 2*M_PI;
  }
  while(best_estimate_particle.yaw < -M_PI)
  {
    best_estimate_particle.yaw += 2*M_PI;
  }

  Particle cov;
  double cov_normalizer = 0;
  for(const Particle & particle : particles_)
  {
    cov.x += pow(best_estimate_particle.x - particle.x, 2)*particle.weight;
    cov.y += pow(best_estimate_particle.y - particle.y, 2)*particle.weight;
    double yaw_error = best_estimate_particle.yaw - particle.yaw;
    while (yaw_error >= M_PI)
    {
      yaw_error -= M_PI;
    }
    while (yaw_error < -M_PI)
    {
      yaw_error += M_PI;
    }
    cov.yaw += pow(yaw_error, 2)*particle.weight;
    cov.vx += pow(best_estimate_particle.vx - particle.vx, 2)*particle.weight;
    cov.yaw_rate += pow(best_estimate_particle.yaw_rate - particle.yaw_rate, 2)*particle.weight;

    cov_normalizer += pow(particle.weight, 2);
  }

  cov.x /= (1-cov_normalizer);
  cov.y /= (1-cov_normalizer);
  cov.yaw /= (1-cov_normalizer);
  cov.vx /= (1-cov_normalizer);
  cov.yaw_rate /= (1-cov_normalizer);

  // use estimated pose to determine the new transform from odom to calculated odom position
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = now();
  transform.child_frame_id = "odom";
  transform.transform.rotation.w = 1;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.translation.x = 0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0;
  tf_broadcaster_.sendTransform(transform);

  // TODO how to publish uncertainty

  PublishParticleVisualization();
}

void ParticleFilterLocalizer::PublishParticleVisualization()
{
  // TODO check for subscriber

  // publish visualization of all particles likelihood
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "/odom";
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.stamp = this->now();
  marker.id = 10;

  marker.color.r = 1.0;
  marker.color.a = 1.0;

  marker.colors.resize(num_particles_);
  for(int i = 0; i < num_particles_; i++) {
    geometry_msgs::msg::Point front_point, right_point, left_point;

    // create three points based off of the particle
    // The marker uses three points to create a triangle between them
    double x = particles_[i].x;
    double y = particles_[i].y;
    double yaw = particles_[i].yaw;

    front_point.x = x + 0.1*cos(yaw);
    front_point.y = y - 0.1*sin(yaw);

    right_point.x = x + 0.025*cos(yaw+M_PI_2);
    right_point.y = y - 0.025*sin(yaw+M_PI_2);

    left_point.x = x + 0.025*cos(yaw-M_PI_2);
    left_point.y = y - 0.025*sin(yaw-M_PI_2);

    marker.points.push_back(front_point);
    marker.points.push_back(right_point);
    marker.points.push_back(left_point);

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.colors[i].r = (particles_[i].weight / max_weight_);
    marker.colors[i].g = 0;
    marker.colors[i].b = 0;
    marker.colors[i].a = 0.7;
  }

  marker_pub_->publish(marker);

}


}  // namespace localization

RCLCPP_COMPONENTS_REGISTER_NODE(localization::ParticleFilterLocalizer)
