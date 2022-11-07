//
// Created by jason on 10/25/22.
//

#ifndef SOFTWARE_TRAINING_CONTROLLER_HELPERS_H
#define SOFTWARE_TRAINING_CONTROLLER_HELPERS_H


namespace controllers {

  inline Eigen::Vector3d StateFromMsg(const geometry_msgs::msg::PoseStamped & pose)
  {
    Eigen::Quaterniond orientation;
    tf2::fromMsg(pose.pose.orientation, orientation);
    Eigen::Vector3d state;
    state << pose.pose.position.x, pose.pose.position.y,
      orientation.toRotationMatrix().eulerAngles(0, 1, 2)[2];
    return state;
  }

}


#endif //SOFTWARE_TRAINING_CONTROLLER_HELPERS_H
