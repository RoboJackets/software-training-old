#ifndef ODOMETRY_SENSOR_MODEL_HPP_
#define ODOMETRY_SENSOR_MODEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_model.hpp"

namespace localization
{

class OdometrySensorModel : public SensorModel
{
public:
    OdometrySensorModel(rclcpp::Node& node);

    void UpdateMeasurement(const nav_msgs::msg::Odometry::SharedPtr msg);

    double ComputeLogProb(const Particle& particle) override;
    double ComputeLogNormalizer() override;
    bool IsMeasurementAvailable(const rclcpp::Time& current_time) override;

private:
    nav_msgs::msg::Odometry last_msg_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace localization

#endif  // ODOMETRY_SENSOR_MODEL_HPP_