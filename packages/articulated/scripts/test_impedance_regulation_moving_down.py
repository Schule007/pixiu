#!/usr/bin/env python3
import numpy as np
import rospy
from articulated_franka import ImpedanceRegulation


def main():
    rospy.init_node("franka_client")
    robot = ImpedanceRegulation()

    # Default server topic name:
    # /franka_impedance_regulation_state

    input("Move robot to target pose")
    T = robot.get_ee_transform()

    input("Enter to start movement")
    robot.regulate_ee(
        T,
        ee_control_force_bound=7,
        k_translation=150,
        k_rotation=10,
    )

    T_ref = np.array(T)
    for i in range(300):
        T_ref[2, 3] -= 0.0003
        robot.set_ee(
            T_ref,
            ee_control_force_bound=7,
            k_translation=150,
            k_rotation=10,
        )
        rospy.sleep(0.01)

    input("Enter to stop")
    robot.stop()


if __name__ == "__main__":
    main()
