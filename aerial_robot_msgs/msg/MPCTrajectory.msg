# MPC Trajectory message for recording a sequence of states and controls
# Used for predicted, reference, and current trajectories in MPC controller

# Header with timestamp and frame information
std_msgs/Header header

# Array of MPC states along the prediction/reference horizon
# For current state: array will have 1 element
# For predicted/reference trajectories: array will have NN+1 elements (full horizon)
aerial_robot_msgs/MPCState[] states

# Array of MPC control inputs along the prediction/reference horizon
# For current state: array will be empty
# For predicted/reference trajectories: array will have NN elements
aerial_robot_msgs/MPCControl[] controls
