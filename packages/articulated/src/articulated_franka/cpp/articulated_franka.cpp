#include "articulated_franka.h"

using namespace articulated_franka;
using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;

ImpedanceRegulation::ImpedanceRegulation(
  std::shared_ptr<franka::Robot> p_robot,
  std::shared_ptr<franka::Model> p_model,
  double translational_stiffness=150,
  double rotational_stiffness=10
) : robot_(p_robot), model_(p_model), status_(idle)
{
  stiffness_.setZero();
  stiffness_.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping_.setZero();
  damping_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
  ros::NodeHandle nh;
  pub_.init(nh, "franka_msg", 1);
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
}

Vector7d ImpedanceRegulation::get_joint_position()
{
  franka::RobotState state = robot_->readOnce();
  Vector7d ret(Vector7d::Map(state.q.data()));
  return ret;
}

Eigen::Matrix4d ImpedanceRegulation::get_o_T_ee()
{
  franka::RobotState state = robot_->readOnce();
  Eigen::Matrix4d ret(Eigen::Matrix4d::Map(state.q.data()));
  return ret;
}

void ImpedanceRegulation::regulate_o_T_ee(Eigen::Matrix4d o_T_ee_desired)
{
  time_since_ = 0.0;
  Eigen::Affine3d desired_transform(o_T_ee_desired);
  position_d_ = desired_transform.translation();
  orientation_d_ = desired_transform.linear();
  auto f = [this] (const franka::RobotState& robot_state, franka::Duration duration) -> franka::Torques {
    return this->control_loop(robot_state, duration);
  };
  status_ = running;
  robot_->control(f);
  status_ = idle;
}

Status ImpedanceRegulation::get_status()
{
  return status_;
}

void ImpedanceRegulation::stop()
{
  status_ = stopping;
}

franka::Torques ImpedanceRegulation::control_loop(const franka::RobotState& robot_state, franka::Duration duration)
{
  time_since_ = time_since_ + duration.toSec();

  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  // convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  // compute error to desired equilibrium pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;
  // orientation error
  // "difference" quaternion
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7), f_task(6);
  // Spring damper system with damping ratio=1
  f_task << -stiffness_ * error - damping_ * (jacobian * dq);
  tau_task << jacobian.transpose() * f_task;
  tau_d << tau_task + coriolis;
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  // ROS stuff
  if (pub_.trylock()) {
    pub_.msg_.dim_q = 7;
    for (size_t i = 0; i < 7; ++i) {
      pub_.msg_.q[i] = robot_state.q[i];
      pub_.msg_.tau[i] = tau_d_array[i];
    }
    for (size_t i = 0; i < 16; ++i) {
      pub_.msg_.o_T_ee[i] = robot_state.O_T_EE[i];
    }
    for (size_t i = 0; i < 6; ++i) {
      pub_.msg_.ee_Ftask[i] = f_task[i];
      pub_.msg_.ee_F_ee[i] = robot_state.K_F_ext_hat_K[i];
    }
    pub_.msg_.time = time_since_;
    pub_.unlockAndPublish();
  }

  if (status_ == stopping) {
    return franka::MotionFinished(franka::Torques(tau_d_array));
  }
  return franka::Torques(tau_d_array);
}
