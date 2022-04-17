#include "articulated/franka/articulated_franka.h"
#include <array>
#include <franka/control_types.h>
#include <stdexcept>

namespace articulated_franka {

using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;

ArticulatedFranka::ArticulatedFranka(
  std::shared_ptr<franka::Robot> p_robot,
  std::shared_ptr<franka::Model> p_model
) : robot_(p_robot), model_(p_model), status_(idle) {}

Vector7d ArticulatedFranka::get_joint_position()
{
  franka::RobotState state = robot_->readOnce();
  Vector7d ret(Vector7d::Map(state.q.data()));
  return ret;
}

Eigen::Matrix4d ArticulatedFranka::get_o_T_ee()
{
  franka::RobotState state = robot_->readOnce();
  Eigen::Matrix4d ret(Eigen::Matrix4d::Map(state.O_T_EE.data()));
  return ret;
}

Status ArticulatedFranka::get_status()
{
  return status_;
}

void ArticulatedFranka::start_torque_control(std::shared_ptr<FrankaControlLaw> p_control_law) {
  if (status_ != idle)
    throw std::runtime_error("Franka is not idle!");
  this->set_default_behavior_();

  float time_since = 0.0;
  auto f = [this, &time_since, p_control_law]
    (const franka::RobotState& robot_state, franka::Duration duration) -> franka::Torques {
    time_since = time_since + duration.toSec();
    auto cmd = franka::Torques(p_control_law->compute(robot_state, time_since, model_));
    if (this->status_ == stopping)
      return franka::MotionFinished(cmd);
    return cmd;
  };
  status_ = running;
  ROS_INFO("Changed status to running. Entering control loop.");
  robot_->control(f);
  ROS_INFO("Exited control loop. Changing status to idle.");
  status_ = idle;
  ROS_INFO("Changed status to idle.");
}

void ArticulatedFranka::stop()
{
  ROS_INFO("Changing status to stopping");
  status_ = stopping;
  ROS_INFO("Changed status to stopping");
}

void ArticulatedFranka::set_default_behavior_() {
  if (!has_set_default_behavior_) {
    ROS_INFO("Setting the default behavior for the first time.");
    // TODO: understand and refactor the below
    // setDefaultBehavior in example common
    robot_->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    // setDefaultBehavior in cartesian example
    robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    has_set_default_behavior_ = true;
  }
}

}  // namespace articulated_franka
