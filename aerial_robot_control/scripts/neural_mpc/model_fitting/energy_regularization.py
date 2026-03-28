import torch

from utils.geometry_utils import q_to_rot_mat_torch
from config.configurations import EnvConfig, ModelFitConfig
from neural_controller import NeuralMPC


class EnergyRegularization:
    def __init__(self, state_feats, u_feats, device, dtype):
        """Differentiable energy regularization.

        The network predicts residual linear acceleration (World frame). We penalize the change in total mechanical
        energy when adding this residual term, compared to the nominal analytical model.

        Important: The returned value must be a torch Tensor so that gradients flow back to y_pred.
        """

        # Reuse nominal MPC instance created by model_fitting/dataset.py.
        # This avoids a second expensive acados solver build during training.
        mpc = NeuralMPC(
            EnvConfig.model_options,
            EnvConfig.solver_options,
            EnvConfig.sim_options,
            EnvConfig.run_options
        )
        self.T_samp = torch.tensor(mpc.T_samp, device=device, dtype=dtype)
        self.T_step = torch.tensor(mpc.T_step, device=device, dtype=dtype)
        self.mass = torch.tensor(mpc.phys.mass, device=device, dtype=dtype)
        self.gravity = torch.tensor(mpc.phys.gravity, device=device, dtype=dtype)
        self.g_w = torch.tensor([0, 0, -self.gravity], device=device, dtype=dtype)

        # For the long-horizon dataset, labels are computed as (state_out - state_prop) / T_step.
        if ModelFitConfig.prop_long_horizon:
            self.T_prop_horizon = self.T_step
        else:
            self.T_prop_horizon = self.T_samp

        self.state_feats = state_feats
        self.u_feats = u_feats

        self.tilt = mpc.tilt
        self.include_servo_model = mpc.include_servo_model
        self.include_thrust_model = mpc.include_thrust_model
        assert self.tilt and self.include_servo_model
        assert not self.include_thrust_model

        # Geometry constants for omni model.
        # p_b: rotor positions in body frame (4x3)
        self.dr = torch.tensor([mpc.phys.dr1, mpc.phys.dr2, mpc.phys.dr3, mpc.phys.dr4], device=device, dtype=dtype)
        self.kq_d_kt = torch.tensor(mpc.phys.kq_d_kt, device=device, dtype=dtype)
        self.p_b = torch.stack(
            [
                torch.tensor(mpc.phys.p1_b, device=device, dtype=dtype).reshape(3),
                torch.tensor(mpc.phys.p2_b, device=device, dtype=dtype).reshape(3),
                torch.tensor(mpc.phys.p3_b, device=device, dtype=dtype).reshape(3),
                torch.tensor(mpc.phys.p4_b, device=device, dtype=dtype).reshape(3),
            ],
            axis=0,
        )
        norm_xy = torch.sqrt(self.p_b[:, 0] ** 2 + self.p_b[:, 1] ** 2)
        self.sin_theta = (self.p_b[:, 1] / norm_xy).unsqueeze(0)
        self.cos_theta = (self.p_b[:, 0] / norm_xy).unsqueeze(0)

        # Indices to get features from input x
        try:
            self._idx_z = state_feats.index(2)
            self._idx_v = [state_feats.index(i) for i in range(3, 6)]
            self._idx_q = [state_feats.index(i) for i in range(6, 10)]
            # self._idx_w = [state_feats.index(i) for i in range(10, 13)]
            self._idx_ft = [len(state_feats) + u_feats.index(i) for i in range(4)]
            if self.tilt:
                if self.include_servo_model and set([13, 14, 15, 16]).issubset(state_feats):
                    if set([4, 5, 6, 7]).issubset(u_feats):
                        print("[WARN] Servo angles are included in both the state and control features. Defaulting to using servo angle for energy regularization.")
                    self._idx_a = [state_feats.index(i) for i in range(13, 17)]
                elif set([4, 5, 6, 7]).issubset(u_feats):
                    self._idx_a = [len(state_feats) + u_feats.index(i) for i in range(4, 8)]
                else:
                    raise ValueError("Servo angles are not included in the input features.")
        except ValueError as e:
            raise ValueError(f"Required features for energy regularization not found in state and control features: {e}")

    def compute_residual_energy(self, x: torch.Tensor, y_pred: torch.Tensor) -> torch.Tensor:
        """
        Compute batch-wise energy of the next state predicted by the nominal and neural-enhanced dynamics.
        It is differentiable w.r.t. y_pred (and also w.r.t x if x.requires_grad=True).
        """
        if y_pred.shape[1] != 3:
            raise ValueError(f"Expected y_pred to have 3 dims for residual acceleration, got shape {tuple(y_pred.shape)}")

        z =  x[:, self._idx_z]
        v =  x[:, self._idx_v]
        q =  x[:, self._idx_q]
        a =  x[:, self._idx_a]
        # w =  x[:, self._idx_w]
        ft = x[:, self._idx_ft]
        
        # Compute rotation matrix from quaternion
        rot_wb = q_to_rot_mat_torch(q)

        # Velocity might be in Body frame depending on ModelFitConfig so we need to transform it back to World frame
        if ModelFitConfig.input_transform:
            v = torch.bmm(rot_wb, v.unsqueeze(-1)).squeeze(-1)

        # Transform thrust from Rotor frame to World frame (vectorized, matches sim_environment/forward_prop.py)
        ft_r_z = ft
        # tau_r_z = -self.dr * ft_r_z * self.kq_d_kt

        ft_e_y = -torch.sin(a) * ft_r_z
        ft_e_z = torch.cos(a) * ft_r_z
        
        # tau_e_y = -torch.sin(a) * tau_r_z
        # tau_e_z = torch.cos(a) * tau_r_z

        ft_b_x = -self.sin_theta * ft_e_y
        ft_b_y = self.cos_theta * ft_e_y
        ft_b_z = ft_e_z

        # tau_b_x = -self.sin_theta * tau_e_y
        # tau_b_y = self.cos_theta * tau_e_y
        # tau_b_z = tau_e_z

        # cross_tau_b_x = self.p_b[:, 1] * ft_b_z - self.p_b[:, 2] * ft_b_y  # p_y * f_z - p_z * f_y
        # cross_tau_b_y = self.p_b[:, 2] * ft_b_x - self.p_b[:, 0] * ft_b_z  # p_z * f_x - p_x * f_z
        # cross_tau_b_z = self.p_b[:, 0] * ft_b_y - self.p_b[:, 1] * ft_b_x  # p_x * f_y - p_y * f_x


        fu_b = torch.stack([ft_b_x.sum(dim=1), ft_b_y.sum(dim=1), ft_b_z.sum(dim=1)], dim=1)  # (B, 3)
        fu_w = torch.bmm(rot_wb, fu_b.unsqueeze(-1)).squeeze(-1)  # (B, 3)

        # tau_u_b = torch.stack(
        #     [
        #         (tau_b_x + cross_tau_b_x).sum(dim=1),
        #         (tau_b_y + cross_tau_b_y).sum(dim=1),
        #         (tau_b_z + cross_tau_b_z).sum(dim=1),
        #     ],
        #     dim=1,
        # )

        # Analytical model
        a_nominal = fu_w / self.mass + self.g_w

        # Neural-enhanced model
        a_res = y_pred[:, :3]
        # If labels are transformed to body frame, rotate back to world frame for physical energy computation.
        if ModelFitConfig.label_transform:
            a_res = torch.bmm(rot_wb, a_res.unsqueeze(-1)).squeeze(-1)
        a_neural = a_nominal + a_res

        # Propagate translational dynamics with RK4
        # State: z (World z position), v (World linear velocity). Dynamics: z_dot = v_z, v_dot = a.
        z_prop_nominal, v_prop_nominal = self.rk4_step(z, v, a_nominal, self.T_prop_horizon)
        z_prop_neural, v_prop_neural = self.rk4_step(z, v, a_neural, self.T_prop_horizon)

        # Compute energy of propagated state
        # TODO - include rotational energy when we actually predict ang acc but for now E_rot_nominal = E_rot_neural
        # E_rot = 0.5 * w.T @ self.I @ w
        E = 0.5 * self.mass * torch.sum(v_prop_nominal * v_prop_nominal, dim=1) + self.mass * self.gravity * z_prop_nominal
        E_neural = 0.5 * self.mass * torch.sum(v_prop_neural * v_prop_neural, dim=1) + self.mass * self.gravity * z_prop_neural

        # Only penalize if neural-enhanced energy exceeds nominal energy
        return torch.relu(E_neural - E)
        # Penalize total energy
        # return E_neural
    
    def rk4_step(self, z_in: torch.Tensor, v_in: torch.Tensor, a_in: torch.Tensor, dt: torch.Tensor):
        k1_z = v_in[:, 2]
        k1_v = a_in

        v2 = v_in + 0.5 * dt * k1_v
        k2_z = v2[:, 2]
        k2_v = a_in

        v3 = v_in + 0.5 * dt * k2_v
        k3_z = v3[:, 2]
        k3_v = a_in

        v4 = v_in + dt * k3_v
        k4_z = v4[:, 2]
        k4_v = a_in

        z_out = z_in + dt / 6.0 * (k1_z + 2.0 * k2_z + 2.0 * k3_z + k4_z)
        v_out = v_in + dt / 6.0 * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
        return z_out, v_out
