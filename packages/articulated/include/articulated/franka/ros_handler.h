#ifndef ARTICULATED_FRANKA_ROS_HANDLER_H_
#define ARTICULATED_FRANKA_ROS_HANDLER_H_

#include "articulated/ImpedanceRegulationControlConfig.h"
#include "ros/node_handle.h"
#include <array>
#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <articulated/franka/articulated_franka.h>
#include <articulated/GetEeTransform.h>
#include <articulated/SetRegulateEeConfig.h>
#include <articulated/WordWord.h>
#include <articulated/FrankaGripperCommand.h>
#include <articulated/RegulateEeTransformAction.h>
#include <stdexcept>

namespace articulated_franka {
class FrankaRosHandler {
  public:
    FrankaRosHandler(
      std::string prefix,
      ros::NodeHandle nh,
      std::shared_ptr<ArticulatedFranka> p_franka,
      std::shared_ptr<franka::Gripper> p_gripper,
      std::shared_ptr<ImpedanceRegulationControlLaw> p_impedance_law
    );

    bool get_ee_transform_cb(
      articulated::GetEeTransform::Request &req,
      articulated::GetEeTransform::Response &res
    );

    bool set_regulate_ee_config_cb(
      articulated::SetRegulateEeConfig::Request &req,
      articulated::SetRegulateEeConfig::Response &res
    );

    void regulate_ee_transform_cb(const articulated::RegulateEeTransformGoalConstPtr& goal);

    void preempt_cb();

    bool stop_cb(
      articulated::WordWord::Request &req,
      articulated::WordWord::Response &res
    );

    bool gripper_command_cb(
      articulated::FrankaGripperCommand::Request &req,
      articulated::FrankaGripperCommand::Response &res
    );

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<ArticulatedFranka> p_franka_;
    std::shared_ptr<franka::Gripper> p_gripper_;
    std::shared_ptr<ImpedanceRegulationControlLaw> p_impedance_law_;
    actionlib::SimpleActionServer<articulated::RegulateEeTransformAction> impedance_regulation_server_;
    ros::ServiceServer get_ee_transform_service_, stop_service_, grasp_service_, config_service_;
    void config_impedance_regulation_(articulated::ImpedanceRegulationControlConfig config);
};
}  // namespace articulated_franka


#endif // ARTICULATED_FRANKA_ROS_HANDLER_H_
