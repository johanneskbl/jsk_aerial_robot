# MPC Control message for recording control inputs at one time point

# Header with timestamp and frame information
std_msgs/Header header

# Thrust commands for each motor (dynamic array)
float64[] thrust_commands

# Servo angle commands for each joint (dynamic array)
float64[] servo_angle_commands
