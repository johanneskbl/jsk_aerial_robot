import os, sys
import argparse
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/tilt_qd")    # Add tilt_qd directory to path to allow relative imports without import adjustment
import sim_nmpc as new_ws


if __name__ == "__main__":
    if 1:
        parser = argparse.ArgumentParser(description="Run the simulation of different NMPC models.")
        parser.add_argument(
            "model",
            type=int,
            default=1,
            help="The NMPC model to be simulated. Options: 0 (no_servo_delay), 1 (default: servo), 2 (thrust), 3(servo+thrust), 21 (servo+dist), \
                22(servo+thrust+dist), 91(no_servo_new_cost), 92(servo_old_cost), 93(servo_vel_input), 94(servo_drag_dist).",
        )
        parser.add_argument(
            "-sim",
            "--sim_model",
            type=int,
            default=0,
            help="The simulation model. " "Options: 0 (default: NMPCTiltQdServoThrust), 1 (NMPCTiltQdServoThrustDrag).",
        )
        parser.add_argument("-p", "--plot_type", type=int, default=0, help="The type of plot. Options: 0 (full), 1, 2.")

        # args = parser.parse_args()

        class Temp():
            def __init__(self):
                self.model = 0 # 0-91, 1-1, 2-2, 3-3: good but when thrust model slight deviance
                # TODO investigate thrust model for small deviations
                # 21-21 & 22-22 strong deviations!
                # 92-92 NOT SOLVING FOR OLD VERSION!
                self.sim_model = 0
                self.plot_type = 0
                self.arch = 'bi'
        args = Temp()

        x_now, x_now_sim, u_cmd, xr, ur = new_ws.main(args)
        os.chdir("/home/johannes/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/nmpc")
        os.remove("mat.npz")
        np.savez('mat.npz', x_now=x_now, x_now_sim=x_now_sim, u_cmd=u_cmd, xr=xr, ur=ur)
        print("Data saved! :)")
    else:
        os.chdir("/home/johannes/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/nmpc")
        new_data = np.load('mat.npz')

        old_data = np.load('mat_old.npz')

        for key in new_data:
            print("===== " + key + " =====")
            print(np.max(np.abs(new_data[key] - old_data[key]), axis=0))
            print("Abs max: ", np.max(np.max(np.abs(new_data[key] - old_data[key])), axis=0))
    
        print("Check finished successfully! :)")

