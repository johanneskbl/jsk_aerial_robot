import os, sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tilt_qd_servo_thrust_diff import NMPCTiltQdServoThrustDiff
from tilt_qd_servo_thrust_diff_second_order import NMPCTiltQdServoThrustDiffSecondOrder


#print("Generating differential controller...")
#NMPCTiltQdServoThrustDiff()
#print("Successfully generated differential controller!")
#print("========================================")
print("Generating differential second-order controller...")
NMPCTiltQdServoThrustDiffSecondOrder()
print("Successfully generated differential second-order controller!")
print("========================================")
