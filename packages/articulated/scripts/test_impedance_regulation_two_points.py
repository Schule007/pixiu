#!/usr/bin/env python3

import time

import rospy
from articulated_franka import ImpedanceRegulation


def main():
    """Object-load based handover."""
    rospy.init_node("franka_client")
    robot = ImpedanceRegulation()

    # Default server topic name:
    # /franka_impedance_regulation_state

    input("Move robot to target pose")
    T_other = robot.get_ee_transform()
    input("Move robot to starting pose")
    T = robot.get_ee_transform()

    input("Enter to start movement")
    target_other = True
    DURATION = 2
    t_prev = time.time() - DURATION
    for i in range(3):
        print(f"Start to move! [{i}]")
        T_target = T_other if target_other else T
        target_other = not target_other
        t_prev = time.time()
        if i == 0:
            robot.regulate_ee(
                T_target,
                ee_control_force_bound=7,
                k_translation=150,
                k_rotation=10,
            )
        else:
            robot.set_ee(
                T_target,
                ee_control_force_bound=7,
                k_translation=150,
                k_rotation=10,
            )
        while time.time() - t_prev < DURATION:
            time.sleep(0.01)
            if rospy.is_shutdown():
                robot.stop()
                raise RuntimeError("rospy shutdown")

    robot.stop()


if __name__ == "__main__":
    main()
