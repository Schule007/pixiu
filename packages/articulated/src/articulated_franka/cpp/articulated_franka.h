#ifndef ARTICULATED_FRANKA_H_
#define ARTICULATED_FRANKA_H_

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <time.h>
#include <string>
#include <functional>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "articulated/ImpedanceRegulationState.h"
#include <realtime_tools/realtime_publisher.h>

namespace articulated_franka {

enum Status {running, idle, stopping};

class ArticulatedFranka {
  public:
    ArticulatedFranka(
      std::shared_ptr<franka::Robot> p_robot,
      std::shared_ptr<franka::Model> p_model
    );
    Eigen::Matrix<double, 7, 1, Eigen::ColMajor> get_joint_position();
    Eigen::Matrix4d get_o_T_ee();
    void regulate_o_T_ee(
      Eigen::Matrix4d o_T_ee_desired,
      double translational_stiffness,
      double rotational_stiffness
    );
    Status get_status();
    void stop();
  private:
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    Eigen::Matrix<double, 6, 6> stiffness_, damping_;
    Status status_{idle};
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    franka::Torques impedance_regulation_cb(const franka::RobotState&, franka::Duration);
    double time_since_{0.0};
    realtime_tools::RealtimePublisher<articulated::ImpedanceRegulationState> pub_;
    bool has_set_default_behavior_{false};
};

}  // namespace articulated_franka

#endif // ARTICULATED_FRANKA_H_
