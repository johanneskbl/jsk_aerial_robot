import os, sys
import argparse
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/tilt_qd")    # Add tilt_qd directory to path to allow relative imports without import adjustment
import sim_nmpc as new_ws


if __name__ == "__main__":
    if 0:
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
                self.model = 23
                # quad
                # 0-91 (with and without sim_nmpc = nmpc) -> 0 deviations
                # 1-1 (with and without sim_nmpc = nmpc) -> 0 deviations
                # 2-2 (without sim_nmpc = nmpc) -> e-7 deviations [with sim_nmpc = nmpc doesnt solve]
                # 3-3 (with and without sim_nmpc = nmpc) -> e-8 deviations
                # 21-21 (with and without sim_nmpc = nmpc) -> 0 deviations -> Previously strong deviations! Reason: old version had no constraint in servo angle command and different and different velocity constraint in yaml
                # 22-22 (with and without sim_nmpc = nmpc) -> e-8 deviations -> Previously strong deviations! Reason: old version had no constraint in servo angle command and different and different velocity constraint in yaml
                # 23-23
                # 24-24
                # 91-0 (without sim_nmpc = nmpc) -> 0 deviations [with sim_nmpc = nmpc doesnt solve]
                # 92-92 (with and without sim_nmpc = nmpc) -> 0 deviations
                # 93-93 DOESNT SOLVE FOR EITHER VERSION
                # 94-94 (with and without sim_nmpc = nmpc) -> 0 deviations
                # 95-95
                # 96-96

                # bi
                # 0-0 (with sim_nmpc = nmpc) -> e-2 deviations
                # 0-0 (with sim_nmpc != nmpc) -> 0 deviations
                # 1-1 (sim_nmpc = nmpc) -> 0 deviations

                # tri
                # 0-0 (with and without sim_nmpc = nmpc) -> 0 deviations
                # 1-1 (sim_nmpc = nmpc) -> 0 deviations

                self.sim_model = 0
                self.plot_type = -1
                self.arch = 'qd'
                self.nmpc_type = 1#  type=int, help="The type of NMPC. 0 means disturbance , 1 means impedance.")
                self.est_dist_type = 1
                self.if_use_ang_acc = 0
        args = Temp()

        import tilt_qd.sim_nmpc_impedance as new_ws

        if args.plot_type == -1:
            x_now, x_now_sim, u_cmd, xr, ur = new_ws.main(args)
            os.chdir("/home/johannes/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/nmpc")
            if os.path.exists('mat.npz'): os.remove("mat.npz")
            np.savez('mat.npz', x_now=x_now, x_now_sim=x_now_sim, u_cmd=u_cmd, xr=xr, ur=ur)
            print("Data saved! :)")
        else:
            new_ws.main(args)
    else:
        os.chdir("/home/johannes/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/aerial_robot_control/scripts/nmpc")
        new_data = np.load('mat.npz')

        old_data = np.load('mat_old.npz')

        for key in new_data:
            print("===== " + key + " =====")
            print(np.max(np.abs(new_data[key] - old_data[key]), axis=0))
            print("Abs max: ", np.max(np.max(np.abs(new_data[key] - old_data[key])), axis=0))
    
        # for 

        print("Check finished successfully! :)")

