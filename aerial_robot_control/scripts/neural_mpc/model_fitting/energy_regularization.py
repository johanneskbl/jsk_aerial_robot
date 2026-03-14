import sys, os
import numpy as np
from sim_environment.forward_prop import init_forward_prop, forward_prop
from sim_environment.forward_prop_neural import init_forward_prop_neural, forward_prop_neural
from neural_controller import NeuralMPC
from config.configurations import EnvConfig

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni


class struct(object):
    pass

class EnergyRegularization:
    def __init__(self):
        # Nominal model
        # Simulate intermediate acceleration before integration
        mpc = NeuralMPC(
            EnvConfig.model_options,
            EnvConfig.solver_options,
            EnvConfig.sim_options,
            EnvConfig.run_options
        )
        self.mass = mpc.phys.mass
        self.I = np.diag(np.array([mpc.phys.Ixx, mpc.phys.Iyy, mpc.phys.Izz]))
        self.gravity = mpc.phys.gravity
        self.T_samp = mpc.T_samp
        self.T_step = mpc.T_step

        # Nominal model
        self.discretized_dynamics = init_forward_prop(mpc)

        # Neural-enhanced model based on current network predictions
        self.discretized_dynamics_neural = init_forward_prop_neural(mpc)


    def compute_residual_energy(self, x, y_pred, nx):
        """
        Energy-based regularization term for model fitting.
        This function computes the kinetic energy of the predicted accelerations and adds it as a regularization term to the loss function during training.
        The idea is to encourage the model to predict physically plausible accelerations that do not lead to excessive energy, which can help with convergence and stability of the NMPC.
        """
        T_prop_horizon = self.T_samp
        T_prop_step = self.T_samp

        E_delta = np.zeros((x.shape[0],))
        for i in range(x.shape[0]):  # Iterate over batch
            state_0 = x[i, :nx].detach().cpu().numpy()
            u_cmd = x[i, nx:].detach().cpu().numpy()
            # Nominal model
            state_out = forward_prop(self.discretized_dynamics, state_0, u_cmd, T_prop_horizon, T_prop_step)
            h = state_out[2]
            v = state_out[3:6]
            w = state_out[10:13]

            # Neural-enhanced model
            state_out_neural = forward_prop_neural(self.discretized_dynamics_neural, state_0, u_cmd, y_pred[i, :].detach().cpu().numpy(), T_prop_horizon, T_prop_step)
            h_neural = state_out_neural[2]
            v_neural = state_out_neural[3:6]
            w_neural = state_out_neural[10:13]

            # Energy of nomnial state out (kinetic incl. rotational + potential)
            E = 0.5 * self.mass * v @ v + 0.5 * w.T @ self.I @ w + self.mass * self.gravity * h

            # Energy of neural-enhanced state out (kinetic incl. rotational + potential)
            E_neural = 0.5 * self.mass * v_neural @ v_neural + 0.5 * w_neural.T @ self.I @ w_neural + self.mass * self.gravity * h_neural

            # Regularization term: penalize excessive energy in the neural-enhanced prediction compared to the nominal model
            E_delta[i] = E_neural - E
            # E_delta[i] = max(0, E_neural - E)  # Only penalize if the neural-enhanced energy exceeds the nominal energy

        return E_delta

        

