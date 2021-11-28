#include <array>
#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <articulated/articulated_franka.h>
#include <articulated/GetEeTransform.h>
#include <articulated/RegulateEeTransform.h>
#include <articulated/WordWord.h>
#include <articulated/FrankaGripperCommand.h>
#include <articulated/RegulateEeTransformAction.h>

namespace af = articulated_franka;

class FrankaRosServiceHandler {
  public:
    FrankaRosServiceHandler(
      std::shared_ptr<af::ArticulatedFranka> p_franka,
      std::shared_ptr<franka::Gripper> p_gripper
    ): p_franka_(p_franka), p_gripper_(p_gripper) {}

    bool get_ee_transform_cb(
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

    void regulate_ee_transform_cb(
      const articulated::RegulateEeTransformGoalConstPtr& goal,
      actionlib::SimpleActionServer<articulated::RegulateEeTransformAction>* as
    ) {
      articulated::RegulateEeTransformResult result;
      try {
        Eigen::Matrix4d desired_transform;
        for (size_t r = 0; r < 4; ++r) {
          for (size_t c = 0; c < 4; ++c) {
            // goal.o_T_ee_desired is row major
            desired_transform(r, c) = goal->o_T_ee_desired[r * 4 + c];
          }
        }
        p_franka_->regulate_o_T_ee(
          desired_transform,
          goal->translational_stiffness,
          goal->rotational_stiffness,
          goal->ee_control_force_bound,
          goal->ee_control_torque_bound
        );
        if (as->isPreemptRequested() || !ros::ok()) {
          result.status = "reglation-preempted";
          as->setPreempted(result);
        }
        else {
          result.status = "terminate-without-preemption";
          as->setSucceeded(result);
        }

      }
      catch (const franka::Exception& ex) {
        ROS_WARN("Exception during control loop %s", ex.what());
        result.status =ex.what();
        as->setAborted(result);
      }
    }

    void preempt_regulate_ee_transform_cb() {
      ROS_INFO("Stopping robot control loop");
      p_franka_->stop();
      ros::Rate r(1000);
      while (p_franka_->get_status() != af::idle and ros::ok()) {
        r.sleep();
      }
    }

    bool stop_cb(
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

    bool gripper_command_cb(
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

  private:
    std::shared_ptr<af::ArticulatedFranka> p_franka_;
    std::shared_ptr<franka::Gripper> p_gripper_;
};


int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  std::string service_prefix = "franka_";
  ros::init(argc, argv, "franka_server");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle n;
  auto p_robot = std::make_shared<franka::Robot>(argv[1]);
  auto p_model = std::make_shared<franka::Model>(p_robot->loadModel());
  auto p_franka = std::make_shared<af::ArticulatedFranka>(p_robot, p_model);
  auto p_gripper = std::make_shared<franka::Gripper>(argv[1]);
  auto service_handler = FrankaRosServiceHandler(p_franka, p_gripper);

  actionlib::SimpleActionServer<articulated::RegulateEeTransformAction> server(
    n, service_prefix + "regulate_ee_transform",
    boost::bind(&FrankaRosServiceHandler::regulate_ee_transform_cb, service_handler, _1, &server), false
  );
  server.registerPreemptCallback(
    boost::bind(&FrankaRosServiceHandler::preempt_regulate_ee_transform_cb, service_handler)
  );
  server.start();

  ros::ServiceServer get_ee_transform_service = n.advertiseService(
    service_prefix + "get_ee_transform",
    &FrankaRosServiceHandler::get_ee_transform_cb,
    &service_handler
  );

  ros::ServiceServer stop_service = n.advertiseService(
    service_prefix + "stop",
    &FrankaRosServiceHandler::stop_cb,
    &service_handler
  );

  ros::ServiceServer grasp_service = n.advertiseService(
    service_prefix + "gripper_command",
    &FrankaRosServiceHandler::gripper_command_cb,
    &service_handler
  );

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
