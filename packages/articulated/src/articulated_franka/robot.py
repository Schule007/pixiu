import threading
import time
import logging

import numpy as np
import rospy
import articulated.srv as articulated_srv


logger = logging.getLogger(__name__)


class ImpedanceRegulation:
    def __init__(self, prefix="franka_"):
        self._c_get_T_ee = rospy.ServiceProxy(
            f"/{prefix}get_ee_transform", articulated_srv.GetEeTransform
        )
        self._c_regulate_ee = rospy.ServiceProxy(
            f"/{prefix}regulate_ee_transform", articulated_srv.RegulateEeTransform
        )
        self._c_stop = rospy.ServiceProxy(f"/{prefix}stop", articulated_srv.WordWord)
        self._regulate_ee_thread = None
        self._status = "idle"
        self._last_info = None

    @property
    def status(self):
        return self._status, self._last_info

    def get_ee_transform(self):
        """Return the transform from origin to EE o_T_ee."""
        res = self._c_get_T_ee.call()
        return np.array(res.o_T_ee).reshape(4, 4)

    def regulate_ee(
        self,
        target_transform,
        k_translation=150.0,
        k_rotation=10.0,
        ee_control_force_bound=50,
        ee_control_torque_bound=4,
    ) -> None:
        """Return the transform from origin to EE o_T_ee."""
        # TODO: change to action client
        req = articulated_srv.RegulateEeTransformRequest()
        req.o_T_ee_desired = target_transform.flatten().tolist()
        req.translational_stiffness = k_translation
        req.rotational_stiffness = k_rotation
        req.ee_control_force_bound = ee_control_force_bound
        req.ee_control_torque_bound = ee_control_torque_bound
        if self._regulate_ee_thread is not None:
            self.stop()
        self._regulate_ee_thread = threading.Thread(
            target=self._await_regulate_ee, args=[req]
        )
        self._status = "running"
        self._last_info = None
        self._regulate_ee_thread.start()
        time.sleep(0.1)  # HACK: will change to action client
        if self._last_info != "success" and self._last_info is not None:
            raise RuntimeError(
                "Fail to start the impedance regulation control! Info: %s"
                % self._last_info
            )

    def _await_regulate_ee(self, req):
        res = self._c_regulate_ee.call(req)
        self._status = "idle"
        self._last_info = res.status

    def stop(self):
        if self._regulate_ee_thread is not None:
            logger.info("Stopping")
            res = self._c_stop("stop")
            logger.info("Stopping response: %s", res.res)
            self._regulate_ee_thread.join()
            self._status = "idle"
            self._regulate_ee_thread = None
        else:
            logger.info("No thread running")


if __name__ == "__main__":
    rospy.init_node("franka_client_interface")
    robot = FrankaImpedanceRegulation()
    T = robot.get_ee_transform()
    print(T)
    input("Enter to regulate at ee")
    robot.regulate_ee(T, ee_control_force_bound=5)
    t0 = time.time()
    while robot.status[0] != "idle":
        time.sleep(0.5)
        if time.time() - t0 > 3:
            print("Command to stop")
            robot.stop()
    print(robot.status)
