# MPC State message for recording a single state at one time point
# Similar structure to geometry_msgs/Pose but with additional fields for MPC states

# Header with timestamp and frame information
std_msgs/Header header

# Position in 3D space
geometry_msgs/Point position

# Linear velocity in 3D space
geometry_msgs/Vector3 linear_velocity

# Linear acceleration in body frame
geometry_msgs/Vector3 linear_acceleration

# Orientation as quaternion
geometry_msgs/Quaternion orientation

# Angular velocity in body frame
geometry_msgs/Vector3 angular_velocity

# Servo angles as state (dynamic array for flexible robot configurations)
float64[] servo_angles

# Servo angle velocity as state
float64[] servo_angle_velocity

# Thrust as state
float64[] thrust

# Thrust velocity as state
float64[] thrust_velocity

# Internal wrench as state in Body frame
geometry_msgs/Vector3 wrench_forces
geometry_msgs/Vector3 wrench_torques
