#include <articulated/franka/ros_handler.h>

namespace articulated_franka {

FrankaRosHandler::FrankaRosHandler(
  std::string prefix,
  ros::NodeHandle nh,
  std::shared_ptr<ArticulatedFranka> p_franka,
  std::shared_ptr<franka::Gripper> p_gripper,
  std::shared_ptr<ImpedanceRegulationControlLaw> p_impedance_law
):
  nh_(nh), p_franka_(p_franka), p_gripper_(p_gripper), p_impedance_law_(p_impedance_law),
  impedance_regulation_server_(
    nh, prefix + "regulate_ee_transform", boost::bind(&FrankaRosHandler::regulate_ee_transform_cb, this, _1), false
  )
{
  impedance_regulation_server_.registerPreemptCallback(boost::bind(&FrankaRosHandler::preempt_cb, this));
  impedance_regulation_server_.start();
  get_ee_transform_service_ = nh.advertiseService(
    prefix + "get_ee_transform", &FrankaRosHandler::get_ee_transform_cb, this
  );
  stop_service_ = nh.advertiseService(
    prefix + "stop", &FrankaRosHandler::stop_cb, this
  );
  grasp_service_ = nh.advertiseService(
    prefix + "gripper_command", &FrankaRosHandler::gripper_command_cb, this
  );
  config_service_ = nh.advertiseService(
    prefix + "set_regulate_ee_config", &FrankaRosHandler::set_regulate_ee_config_cb, this
  );
}

bool FrankaRosHandler::get_ee_transform_cb(
  articulated::GetEeTransform::Request &req,
  articulated::GetEeTransform::Response &res
) {
  Eigen::Matrix4d o_T_ee = p_franka_->get_o_T_ee();
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      res.o_T_ee[r * 4 + c] = o_T_ee(r, c);
    }
  }
  return true;
}

void FrankaRosHandler::config_impedance_regulation_(
  articulated::ImpedanceRegulationControlConfig config
) {
  Eigen::Matrix4d desired_transform;
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      // goal.o_T_ee_desired is row major
      desired_transform(r, c) = config.o_T_ee_desired[r * 4 + c];
    }
  }
  std::array<double, 6> stiffness, damping;
  for (size_t i = 0; i < 6; ++i) {
    stiffness[i] = config.stiffness[i];
    damping[i] = config.damping[i];
  }
  p_impedance_law_->config(
    desired_transform,
    stiffness,
    damping,
    config.ee_control_force_bound,
    config.ee_control_torque_bound
  );
}

bool FrankaRosHandler::set_regulate_ee_config_cb(
  articulated::SetRegulateEeConfig::Request &req,
  articulated::SetRegulateEeConfig::Response &res
) {
  if (p_franka_->get_status() != running) return false;
  config_impedance_regulation_(req.config);
  return true;
}

void FrankaRosHandler::regulate_ee_transform_cb(
  const articulated::RegulateEeTransformGoalConstPtr& goal
) {
  articulated::RegulateEeTransformResult result;
  try {
    config_impedance_regulation_(goal->config);
    p_franka_->start_torque_control(p_impedance_law_);
    if (impedance_regulation_server_.isPreemptRequested() || !ros::ok()) {
      result.status = "regulation-preempted";
      impedance_regulation_server_.setPreempted(result);
    }
    else {
      result.status = "terminate-without-preemption";
      impedance_regulation_server_.setSucceeded(result);
    }
  }
  catch (const franka::Exception& ex) {
    ROS_WARN("Exception during control loop %s", ex.what());
    result.status = ex.what();
    impedance_regulation_server_.setAborted(result);
  }
  catch (const std::runtime_error& ex) {
    ROS_WARN("Exception before control loop started %s", ex.what());
    result.status = ex.what();
    impedance_regulation_server_.setAborted(result);
  }
}

void FrankaRosHandler::preempt_cb() {
  ROS_INFO("Stopping robot control loop");
  p_franka_->stop();
  ros::Rate r(1000);
  while (p_franka_->get_status() != idle and ros::ok()) {
    r.sleep();
  }
}

bool FrankaRosHandler::stop_cb(
  articulated::WordWord::Request &req,
  articulated::WordWord::Response &res
) {
  try {
    p_franka_->stop();
    res.res = "success";
    return true;
  }
  catch (const franka::Exception& ex) {
    res.res = ex.what();
    return false;
  }
}

bool FrankaRosHandler::gripper_command_cb(
  articulated::FrankaGripperCommand::Request &req,
  articulated::FrankaGripperCommand::Response &res
) {
  if (req.kind == "grasp") {
    bool success = p_gripper_->grasp(req.width, req.speed, req.force, req.epsilon_inner, req.epsilon_outer);
    if (success) {res.status="success";} else {res.status="fail";}
    return true;
  }
  if (req.kind == "move") {
    ROS_INFO("Start to move gripper");
    bool success = p_gripper_->move(req.width, req.speed);
    ROS_INFO("Finish to move gripper");
    if (success) {res.status="success";} else {res.status="fail";}
    return true;
  }
  if (req.kind == "homing") {
    bool success = p_gripper_->homing();
    if (success) {res.status="success";} else {res.status="fail";}
    return true;
  }
  if (req.kind == "stop") {
    bool success = p_gripper_->stop();
    if (success) {res.status="success";} else {res.status="fail";}
    return true;
  }
  if (req.kind == "read") {
    franka::GripperState state = p_gripper_->readOnce();
    res.status = "read";
    res.width = state.width;
    res.max_width = state.max_width;
    res.temperature = state.temperature;
    res.is_grasped = state.is_grasped;
    return true;
  }
  res.status = "unknown-command";
  return true;
}


}
