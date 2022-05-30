float64[16] o_T_ee_desired  # desired end effector transform, row major
# World frame stiffness and damping [x, y, z, rx, ry, rz]
float64[6] stiffness
float64[6] damping
# max absolute value of operational space control force
float64 ee_control_force_bound
# max absolute value of operational space control torque
float64 ee_control_torque_bound
