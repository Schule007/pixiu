#ifndef FRANKA_CONTROL_LAW_H_
#define FRANKA_CONTROL_LAW_H_

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <chrono>
#include <array>
#include <Eigen/Dense>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <articulated/ImpedanceRegulationState.h>

namespace articulated_franka {

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class FrankaControlLaw {
  public:
    virtual std::array<double, 7> compute(
      franka::RobotState state,
      float time_since_start,
      std::shared_ptr<franka::Model> p_model
    ) = 0;
};

struct ImpedanceRegulationControlLawConfig {
  Eigen::Matrix<double, 6, 6> stiffness_, damping_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  double ee_control_force_bound_, ee_control_torque_bound_;
};

class ImpedanceRegulationControlLaw: public FrankaControlLaw {
  public:
    ImpedanceRegulationControlLaw(std::string ros_msg_prefix);
    std::array<double, 7> compute(
      franka::RobotState state,
      float time_since_start,
      std::shared_ptr<franka::Model> p_model
    ) override;
    void config(
      Eigen::Matrix4d o_T_ee_desired,
      std::array<double, 6> stiffness,
      std::array<double, 6> damping,
      double ee_control_force_bound,
      double ee_control_torque_bound
    );
    Vector6d get_task_f();

  private:
    int current_config_idx_ = 0;
    std::array<ImpedanceRegulationControlLawConfig, 2> configs_;
    std::mutex config_mutex_, f_task_mutex_;
    realtime_tools::RealtimePublisher<articulated::ImpedanceRegulationState> pub_;
    Vector6d f_task_;
};

}  // namespace artculated_franka

#endif // FRANKA_CONTROL_LAW_H_
