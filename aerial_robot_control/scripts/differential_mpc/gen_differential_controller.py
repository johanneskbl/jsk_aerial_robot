import os, sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tilt_qd_servo_thrust_diff import NMPCTiltQdServoThrustDiff


print("Generating differential controller...")
NMPCTiltQdServoThrustDiff()
print("Successfully generated differential controller!")
print("========================================")
