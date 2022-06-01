#!/usr/bin/env python3
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
        stiffness=[150, 150, 150, 10, 10, 10],
        damping=[30, 30, 30, 8, 8, 8],
    )

    input("Enter to stop")
    robot.stop()


if __name__ == "__main__":
    main()
