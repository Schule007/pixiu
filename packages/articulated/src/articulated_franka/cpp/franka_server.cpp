#include <array>
#include <memory>
#include "ros/ros.h"
#include "articulated_franka.h"
#include "articulated/GetEeTransform.h"
#include "articulated/RegulateEeTransform.h"
#include "articulated/WordWord.h"

namespace af = articulated_franka;

class FrankaRosServiceHandler {
  public:
    FrankaRosServiceHandler(std::shared_ptr<af::ArticulatedFranka> p_franka);
    bool get_ee_transform_cb(articulated::GetEeTransform::Request &req,
                             articulated::GetEeTransform::Response &res);
    bool regulate_ee_transform_cb(articulated::RegulateEeTransform::Request &req,
                                  articulated::RegulateEeTransform::Response &res);
    bool stop_cb(articulated::WordWord::Request &req,
                 articulated::WordWord::Response &res);
  private:
    std::shared_ptr<af::ArticulatedFranka> p_franka_;
};

FrankaRosServiceHandler::FrankaRosServiceHandler(
  std::shared_ptr<af::ArticulatedFranka> p_franka
): p_franka_(p_franka) {}

bool FrankaRosServiceHandler::get_ee_transform_cb(
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

bool FrankaRosServiceHandler::regulate_ee_transform_cb(
  articulated::RegulateEeTransform::Request &req,
  articulated::RegulateEeTransform::Response &res
) {
  try {
    Eigen::Matrix4d desired_transform;
    for (size_t r = 0; r < 4; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        // req.o_T_ee_desired is row major
        desired_transform(r, c) = req.o_T_ee_desired[r * 4 + c];
      }
    }
    p_franka_->regulate_o_T_ee(
      desired_transform,
      req.translational_stiffness,
      req.rotational_stiffness
    );
    res.status = "success";
    return true;
  }
  catch (const franka::Exception& ex) {
    res.status = ex.what();
    return true;
  }
}

bool FrankaRosServiceHandler::stop_cb(
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
  auto service_handler = FrankaRosServiceHandler(p_franka);

  std::string get_ee_transform_service_name = service_prefix + "get_ee_transform";
  ros::ServiceServer get_ee_transform_service = n.advertiseService(
    get_ee_transform_service_name,
    &FrankaRosServiceHandler::get_ee_transform_cb,
    &service_handler
  );

  std::string regulate_ee_transform_service_name = service_prefix + "regulate_ee_transform";
  ros::ServiceServer regulate_ee_transform_service = n.advertiseService(
    regulate_ee_transform_service_name,
    &FrankaRosServiceHandler::regulate_ee_transform_cb,
    &service_handler
  );

  std::string stop_service_name = service_prefix + "stop";
  ros::ServiceServer stop_service = n.advertiseService(
    stop_service_name,
    &FrankaRosServiceHandler::stop_cb,
    &service_handler
  );

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
