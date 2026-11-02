# MPC Control message for recording control inputs at one time point

# Header with timestamp and frame information
std_msgs/Header header

# Thrust velocity optimums for each motor (dynamic array)
float64[] thrust_velocity_uo

# Thrust commands for each motor (dynamic array)
float64[] thrust_cmd

# Servo angle velocity optimums for each joint (dynamic array)
float64[] servo_angle_velocity_uo

# Servo angle commands for each joint (dynamic array)
float64[] servo_angle_cmd
