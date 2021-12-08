#include <articulated/franka/control_law.h>

namespace articulated_franka {

double clamp(double value, double lower_limit, double upper_limit) {
  if (value < lower_limit) {
    return lower_limit;
  }
  else if (value > upper_limit) {
    return upper_limit;
  }
  else {
    return value;
  }
}

ImpedanceRegulationControlLaw::ImpedanceRegulationControlLaw (std::string ros_msg_prefix) {
  ros::NodeHandle nh;
  pub_.init(nh, ros_msg_prefix + "impedance_regulation_state", 1);
}

/* Update the impedance regulation parameters.
 *
 * If the robot is not running, update the current active buffer.
 *
 * If the robot is not running, update the inactive buffer, and flag to switch
 * the inactive buffer in the next compute call.
 *
 * */
void ImpedanceRegulationControlLaw::config(
  Eigen::Matrix4d o_T_ee_desired,
  double translational_stiffness,
  double rotational_stiffness,
  double ee_control_force_bound,
  double ee_control_torque_bound
) {
  // Block if configuration is already going on
  const std::lock_guard<std::mutex> lock(config_mutex_);
  // Block if previous configuration is finished but active buffer is not
  // updated yet.
  Eigen::Affine3d desired_transform(o_T_ee_desired);
  position_d_ = desired_transform.translation();
  orientation_d_ = desired_transform.linear();

  ee_control_force_bound_ = ee_control_force_bound;
  ee_control_torque_bound_ = ee_control_torque_bound;
  stiffness_.setZero();
  stiffness_.topLeftCorner(3, 3) <<
    translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness_.bottomRightCorner(3, 3) <<
    rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping_.setZero();
  damping_.topLeftCorner(3, 3) <<
    2.5 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping_.bottomRightCorner(3, 3) <<
    2.5 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  // Switch the parameter buffer if the robot is running
}

std::array<double, 7> ImpedanceRegulationControlLaw::compute(
  franka::RobotState robot_state,
  float time_since_start,
  std::shared_ptr<franka::Model> p_model
) {
  const std::lock_guard<std::mutex> lock(config_mutex_);
  std::array<double, 7> coriolis_array = p_model->coriolis(robot_state);
  std::array<double, 42> jacobian_array = p_model->zeroJacobian(franka::Frame::kEndEffector, robot_state);
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
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  // compute control
  Eigen::VectorXd tau_task(7), tau_d(7);
  const std::lock_guard<std::mutex> lock_f_task(f_task_mutex_);
  f_task_ << -stiffness_ * error - damping_ * (jacobian * dq);
  for (size_t i = 0; i < 3; ++i) {
    f_task_(i) = clamp(
      f_task_(i), -ee_control_force_bound_, ee_control_force_bound_
    );
  }
  for (size_t i = 3; i < 6; ++i) {
    f_task_(i) = clamp(
      f_task_(i), -ee_control_torque_bound_, ee_control_torque_bound_
    );
  }
  tau_task << jacobian.transpose() * f_task_;
  tau_d << tau_task + coriolis;
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  if (pub_.trylock()) {
    pub_.msg_.dim_q = 7;
    for (size_t i = 0; i < 7; ++i) {
      pub_.msg_.q[i] = robot_state.q[i];
      pub_.msg_.tau[i] = tau_d_array[i];
    }
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
          // O_T_EE is column major
          pub_.msg_.o_T_ee[i * 4 + j] = robot_state.O_T_EE[j * 4 + i];
        }
    }
    for (size_t i = 0; i < 6; ++i) {
      pub_.msg_.ee_Ftask[i] = f_task_[i];
      pub_.msg_.ee_F_ee[i] = - robot_state.K_F_ext_hat_K[i];
    }
    pub_.msg_.time = time_since_start;
    pub_.unlockAndPublish();
  }

  return tau_d_array;
}

Vector6d ImpedanceRegulationControlLaw::get_task_f()
{
  const std::lock_guard<std::mutex> lock_f_task(f_task_mutex_);
  return f_task_;
}

}  // namespace articulated_franka
