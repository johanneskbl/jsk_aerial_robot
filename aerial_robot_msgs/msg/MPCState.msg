# MPC State message for recording a single state at one time point
# Similar structure to geometry_msgs/Pose but with additional fields for MPC states

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

# Servo/joint angles as state (dynamic array for flexible robot configurations)
float64[] servo_angles
