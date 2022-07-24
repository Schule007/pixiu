#include <memory>
#include <ros/ros.h>
#include <articulated/franka/articulated_franka.h>
#include <articulated/franka/ros_handler.h>

namespace af = articulated_franka;

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  std::string service_prefix = "franka_";
  ros::init(argc, argv, "franka_server");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle n;
  auto p_robot = std::make_shared<franka::Robot>(argv[1]);
  auto p_model = std::make_shared<franka::Model>(p_robot->loadModel());
  auto p_franka = std::make_shared<af::ArticulatedFranka>(p_robot, p_model);
  auto p_gripper = std::make_shared<franka::Gripper>(argv[1]);
  auto p_impedance_law = std::make_shared<af::ImpedanceRegulationControlLaw>(service_prefix);
  af::FrankaRosHandler service_handler(service_prefix, n, p_franka, p_gripper, p_impedance_law);
  spinner.start();
  ROS_INFO("Franka server ready!");
  ros::waitForShutdown();

  return 0;
}
