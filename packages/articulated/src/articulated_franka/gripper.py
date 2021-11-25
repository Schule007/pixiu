import logging

import rospy
import articulated.srv as articulated_srv


logger = logging.getLogger(__name__)


class Gripper:
    """Wrapper ROS client.

    Docs: https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html
    """
    def __init__(self, service_name="franka_gripper_command"):
        logger.info("Init a ROS client for franka gripper service %s", service_name)
        self._client = rospy.ServiceProxy(
            service_name, articulated_srv.FrankaGripperCommand
        )

    def homing(self):
        logger.info("Homing the franka gripper")
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "homing"
        res = self._client.call(req)
        return res.status

    def stop(self):
        logger.info("Stopping the franka gripper")
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "stop"
        res = self._client.call(req)
        return res.status

    def move(self, width, speed=0.01):
        """Move the gripper.

        Args:
            width: Openning, [m]
            speed: Speed of change of openning, [m/s]
        """
        logger.info("Moving the franka gripper. Width: %s, speed: %s", width, speed)
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "move"
        req.width = width
        req.speed = speed
        res = self._client.call(req)
        return res.status

    def grasp(
        self,
        width,
        speed,
        force,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
    ):
        """Grasp an object.

        A grasp is considered successful if
                width - epsilon_inner < d < width + epsilon_outer
        where d is the distance between the fingers.

        Args:
            width: Openning [m].
            speed: Speed of change of openning [m/s].
            force: Grasp force [N].
        """
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
        res = self._client.call(req)
        return res.status

    def read(self) -> articulated_srv.FrankaGripperCommandResponse:
        logger.info("Reading gripper state")
        req = articulated_srv.FrankaGripperCommandRequest()
        req.kind = "read"
        res = self._client.call(req)
        return res
