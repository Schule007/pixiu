import logging

import rospy
import articulated.srv as articulated_srv


logger = logging.getLogger(__name__)


class Gripper:
    def __init__(self, service_name="franka_gripper_command"):
        logger.info("Init a ROS client for franka gripper service %s", service_name)
        self._client = rospy.ServiceProxy(
            service_name, articulated_srv.FrankaGripperCommand
        )

    def homing(self):
        logger.info("Homing the franka gripper")
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "homing"
        self._client.call(req)
        return req.status

    def stop(self):
        logger.info("Stopping the franka gripper")
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "stop"
        self._client.call(req)
        return req.status

    def move(self, width, speed):
        logger.info("Moving the franka gripper. Width: %s, speed: %s", width, speed)
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "move"
        req.width = width
        req.speed = speed
        self._client.call(req)
        return req.status

    def grasp(
        self,
        width,
        speed,
        force,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
    ):
        logger.info(
            "Grasping with the franka gripper. Width: [%s, %s], speed: %s, force: %s",
            width - epsilon_inner,
            width + epsilon_outer,
            speed,
            force,
        )
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "grasp"
        req.width = width
        req.speed = speed
        req.force = force
        req.epsilon_inner = epsilon_inner
        req.epsilon_outer = epsilon_outer
        self._client.call(req)
        return req.status
