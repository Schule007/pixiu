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
#include <franka/gripper.h>
#include <map>
#include <time.h>
#include <string>
#include <functional>
#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "articulated/ImpedanceRegulationState.h"
#include <articulated/franka/control_law.h>
#include <realtime_tools/realtime_publisher.h>

namespace articulated_franka {

enum Status {running, idle, stopping};

double clamp(double value, double lower_limit, double upper_limit);

class ArticulatedFranka {
  public:
    ArticulatedFranka(
      std::shared_ptr<franka::Robot> p_robot,
      std::shared_ptr<franka::Model> p_model
    );
    Eigen::Matrix<double, 7, 1, Eigen::ColMajor> get_joint_position();
    Eigen::Matrix4d get_o_T_ee();
    Status get_status();
    void stop();
    void start_torque_control(std::shared_ptr<FrankaControlLaw> p_control_law);
  private:
    std::shared_ptr<franka::Robot> robot_;
    std::shared_ptr<franka::Model> model_;
    Status status_{idle};
    bool has_set_default_behavior_{false};
    void set_default_behavior_();
};

}  // namespace articulated_franka

#endif // ARTICULATED_FRANKA_H_
