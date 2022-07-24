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
  std::array<double, 6> stiffness,
  std::array<double, 6> damping,
  double ee_control_force_bound,
  double ee_control_torque_bound
) {
  int to_config_idx = 1 - current_config_idx_;
  Eigen::Affine3d desired_transform(o_T_ee_desired);
  configs_.at(to_config_idx).position_d_ = desired_transform.translation();
  configs_.at(to_config_idx).orientation_d_ = desired_transform.linear();

  configs_.at(to_config_idx).ee_control_force_bound_ = ee_control_force_bound;
  configs_.at(to_config_idx).ee_control_torque_bound_ = ee_control_torque_bound;

  configs_.at(to_config_idx).stiffness_.setZero();
  configs_.at(to_config_idx).damping_.setZero();
  for (size_t i = 0; i < 6; ++i) {
    configs_.at(to_config_idx).stiffness_(i, i) = stiffness[i];
    configs_.at(to_config_idx).damping_(i, i) = damping[i];
  }
  // Switch the parameter buffer if the robot is running
  const std::lock_guard<std::mutex> lock(config_mutex_);
  current_config_idx_ = to_config_idx;
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
  error.head(3) << position - configs_.at(current_config_idx_).position_d_;
  // orientation error
  if (configs_.at(current_config_idx_).orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * configs_.at(current_config_idx_).orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  // compute control
  Eigen::VectorXd tau_task(7), tau_null(7), tau_d(7);
  const std::lock_guard<std::mutex> lock_f_task(f_task_mutex_);
  f_task_ << -configs_.at(current_config_idx_).stiffness_ * error - configs_.at(current_config_idx_).damping_ * (jacobian * dq);
  for (size_t i = 0; i < 3; ++i) {
    f_task_(i) = clamp(
      f_task_(i),
      -configs_.at(current_config_idx_).ee_control_force_bound_,
      configs_.at(current_config_idx_).ee_control_force_bound_
    );
  }
  for (size_t i = 3; i < 6; ++i) {
    f_task_(i) = clamp(
      f_task_(i),
      -configs_.at(current_config_idx_).ee_control_torque_bound_,
      configs_.at(current_config_idx_).ee_control_torque_bound_
    );
  }
  tau_task << jacobian.transpose() * f_task_;

  ///////// TODO: optimize speed. Too slow to meet 1000Hz control rate.
  // Eigen::JacobiSVD<Eigen::MatrixXd> jacobian_svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // double pinv_tolerance = std::numeric_limits<double>::epsilon()
  //   * std::max(jacobian.cols(), jacobian.rows())
  //   * jacobian_svd.singularValues().array().abs()(0);
  // Eigen::Matrix<double, 7, 6> jacobian_pinv = jacobian_svd.matrixV() * (
  //   jacobian_svd.singularValues().array().abs() > pinv_tolerance
  // ).select(
  //   jacobian_svd.singularValues().array().inverse(), 0
  // ).matrix().asDiagonal() * jacobian_svd.matrixU().adjoint();;
  // Eigen::Matrix<double, 7, 7> task_orthogonal_projector = jacobian_pinv * jacobian;
  // Eigen::Matrix<double, 7, 7> null_orthogonal_projector = Eigen::Matrix<double, 7, 7>::Identity() - task_orthogonal_projector;
  // tau_null << - null_orthogonal_projector * dq;

  // tau_d << tau_task + tau_null + coriolis;
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
