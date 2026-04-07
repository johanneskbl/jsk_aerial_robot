import os, sys
import json
from typing import Union, List
import numpy as np
import torch
from acados_template import AcadosOcpSolver

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.configurations import DirectoryConfig
from network_architecture.mlp import MLP
from network_architecture.vae import VAE
from utils.geometry_utils import v_dot_q, quaternion_inverse


def compute_jacobian(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        """
        Compute the Jacobian of the network output with respect to the input at point x.
        
        Args:
            x: Input tensor of shape (batch_size, input_size) or (input_size,)
            ATTENTION: x must require gradients for autograd to work properly, e.g. "x.clone().detach().requires_grad_(True)"
            y: Output tensor of shape (batch_size, output_size) or (output_size,), e.g. "y = neural_model(x)"

        Returns:
            Jacobian matrix of shape (batch_size, output_size, input_size) or (output_size, input_size)
        """        
        assert x.dim() == y.dim()
        is_single = x.dim() == 1
        batch_size = 1 if is_single else x.shape[0]
        jacobian = torch.zeros(batch_size, y.shape[-1], x.shape[-1], device=x.device)
        
        for i in range(y.shape[-1]):
            # Compute gradient of output i with respect to input
            grad_outputs = torch.zeros_like(y)
            if is_single:
                grad_outputs[i] = 1.0
            else:
                grad_outputs[:, i] = 1.0
                
            grads = torch.autograd.grad(
                outputs=y,
                inputs=x,
                grad_outputs=grad_outputs,
                create_graph=False,
                retain_graph=True
            )[0]
            
            jacobian[:, i, :] = grads
        
        if is_single:
            jacobian = jacobian.squeeze(0)
            
        return jacobian

def aux_function(func):
    def inner_aux(inputs):
        out = func(inputs)
        return out, out

    return inner_aux
    
def aux_function_jac(func):
    def inner_aux(inputs):
        out = func(inputs)
        return out[0], (out[0], out[1])

    return inner_aux

import torch.func as functorch
def batched_hessian(func, inputs: torch.Tensor, create_graph=False,
                    return_jacobian=False, return_func_output=False):
    r"""

    Args:
        func: a Python function that takes Tensor inputs and returns a tuple of Tensors or a Tensor.
        inputs: inputs to the function ``func``. First dimension is treated as batch dimension
        create_graph: If ``True``, the Hessian will be computed in a differentiable manner.
        return_jacobian: If ``True``, the Jacobian will be returned.
        return_func_output: If ``True``, the function output will be returned.

    Returns: Hessian

    """

    if inputs.shape[0] == 1:
        vmap_randomness = 'same'
    else:
        # https://github.com/pytorch/functorch/issues/996
        # Should be 'different'
        vmap_randomness = 'same'

    with torch.no_grad():
        if not return_func_output and not return_jacobian:
            return functorch.vmap(functorch.jacrev(functorch.jacrev(func)), randomness=vmap_randomness)(
                inputs[:, None])
        elif not return_func_output and return_jacobian:
            return functorch.vmap(functorch.jacrev(aux_function_jac(functorch.jacrev(func)), has_aux=True),
                                    randomness=vmap_randomness)(inputs[:, None])
        elif return_func_output and not return_jacobian:
            return functorch.vmap(functorch.jacrev(functorch.jacrev(aux_function(func), has_aux=True)),
                                    randomness=vmap_randomness)(inputs[:, None])
        elif return_func_output and return_jacobian:
            (hessian, (jacobian, value)) = functorch.vmap(
                functorch.jacrev(aux_function_jac(functorch.jacrev(aux_function(func), has_aux=True)),
                                    has_aux=True), randomness=vmap_randomness)(inputs[:, None])
            return hessian, jacobian, value


def linearize(neural_mpc, x0: torch.Tensor, order: int) -> List[np.ndarray]:
    """
    Linearize the neural network around the working point x0.
    Returns the linearization: y ≈ y0 + J @ (x - x0) + 0.5 * (x - x0)^T * H @ (x - x0)
    Args:
        x0: Working point tensor of shape (input_size,) or (batch_size, input_size)
        order: Order of linearization (1 for first-order Taylor expansion, 2 for second-order Taylor expansion)
    Returns:
        y0: Output at the working point
        J0: Jacobian matrix at the working point
        H0: Hessian matrix at the working point (only if order=2)
    """

    # Taylor expansion: y = y0 + J0 * (x - x0) + 0.5 * (x - x0)^T * H0 * (x - x0)
    # NOTE: y0 does NOT need to be label transformed since mlp_out is transformed inside the MPC formulation
    if order == 1:
        y0 = neural_mpc.neural_model(x0)
        J0 = compute_jacobian(x0, y0)
    elif order == 2:
        H0, J0, y0 = batched_hessian(neural_mpc.neural_model, x0, return_func_output=True, return_jacobian=True)

    # Flatten Jacobian and Hessian by appending all rows into one column
    # =============================
    # The two libraries have opposite default ordering:
    # Library           | Default order	        | Reads elements...
    # ------------------|-----------------------|-----------------
    # NumPy .reshape	| Row-major ('C')	    | row by row
    # CasADi .reshape	| Column-major ('F')	| column by column

    # Example:
    # J = [[a, b],
    #      [c, d]]
    # NumPy: J.reshape(-1)          # → [a, b, c, d]  (row by row)
    # CasADi: casadi.reshape(J, -1) # → [a, c, b, d]  (column by column)

    # => CasADi's reshape is equivalent to NumPy's reshape(..., order='F').
    # NOTE: PyTorch's reshape follows NumPy's convention
    # =============================

    y0_np = y0.detach().cpu().numpy().reshape(x0.shape[0], -1, order='F')
    J0_np = J0.detach().cpu().numpy().reshape(x0.shape[0], -1, order='F')
    if order == 2:
        # Remove zero dimensions (clean-up)
        H0 = H0.squeeze(1).squeeze(1).squeeze(3)  # shape (batch_size, output_size, input_size, input_size) NOTE: Don't call .squeeze() without specifying dimension since for sim we have batch_size=1
        # Flatten all inner input_size x input_size Hessian matrices H0_i into one column and concatenate all Hessian columns after each other
        H0_list = []
        for i in range(H0.shape[1]):
            H0_list.append(H0[:,i,:,:].cpu().numpy().reshape(x0.shape[0], -1, order='F'))
        
        H0_np = np.concatenate([*H0_list], axis=1)  # concatenate columns of all Hessian matrices into one column vector
    else:
        H0_np = np.empty((x0.shape[0], 0))

    return y0_np, J0_np, H0_np

def set_linearization_params(neural_mpc, ocp_solver: AcadosOcpSolver, order: int):
    # Set linearization point as parameters
    x0_np = np.zeros((ocp_solver.N+1, len(neural_mpc.state_feats) + len(neural_mpc.u_feats)))
    for j in range(ocp_solver.N+1):
        state_j = ocp_solver.get(j, "x")
        if j < ocp_solver.N:
            u_cmd_j = ocp_solver.get(j, "u")
        else:
            # NOTE: For the terminal node, we can only use the control input from the previous node since the control input for the terminal node is not available in the optimization scheme
            # This is an approximation/assumption that the control input is not far from the previous node's control input!!
            pass  # use u_cmd_j from previous node
        # Input transform
        if neural_mpc.mlp_metadata["ModelFitConfig"]["input_transform"]:
            v_b = v_dot_q(state_j[3:6], quaternion_inverse(state_j[6:10]))
            state_j = np.concatenate(state_j[:3], v_b, state_j[6:])
        
        x0_j_np = np.concatenate([state_j[neural_mpc.state_feats], u_cmd_j[neural_mpc.u_feats]])
        x0_np[j, :] = x0_j_np

    x0 = torch.tensor(x0_np).float().to(neural_mpc.device).requires_grad_(True)  # shape: (N, input_size)
    y0_np, J0_np, H0_np = linearize(neural_mpc, x0, order)  # shape: y0: (N, output_size), J0: (N, output_size * input_size), H0: (N, output_size * input_size * input_size)
    # NOTE: Since the control input u is not available for the terminal node, we can only linearize up to N-1 and ignore/zero-out the linearization for node N (initialization value)
    neural_mpc.acados_parameters[:, neural_mpc.linearize_start_idx : neural_mpc.linearize_end_idx] = \
        np.concatenate([x0_np, y0_np, J0_np, H0_np], axis=1)

def set_linearization_params_sim(sim_neural_mpc, state_curr_sim: np.ndarray, u_cmd: np.ndarray, order: int):
    # Input transform
    if sim_neural_mpc.mlp_metadata["ModelFitConfig"]["input_transform"]:
        v_b = v_dot_q(state_curr_sim[3:6], quaternion_inverse(state_curr_sim[6:10]))
        state_curr_sim = np.concatenate(state_curr_sim[:3], v_b, state_curr_sim[6:])
    # Set linearization point as parameters for one step in simulator
    x0_np = np.concatenate([state_curr_sim[sim_neural_mpc.state_feats], u_cmd[sim_neural_mpc.u_feats]])
    x0 = torch.tensor(x0_np).unsqueeze(0).float().to(sim_neural_mpc.device).requires_grad_(True)
    y0_np, J0_np, H0_np = linearize(sim_neural_mpc, x0, order)
    y0_np = y0_np.squeeze(0)
    J0_np = J0_np.squeeze(0)
    H0_np = H0_np.squeeze(0)

    sim_neural_mpc.acados_parameters[0, sim_neural_mpc.linearize_start_idx : sim_neural_mpc.linearize_end_idx] = \
        np.concatenate([x0_np, y0_np, J0_np, H0_np])


def set_l4casadi_params(neural_mpc, ocp_solver: AcadosOcpSolver):
    # Set l4casadi parameters
    state_over_horizon = np.zeros((0, neural_mpc.state.shape[0]))
    u_cmd_over_horizon = np.zeros((0, neural_mpc.controls.shape[0]))
    for j in range(ocp_solver.N + 1):
        state_over_horizon = np.append(state_over_horizon, ocp_solver.get(j, "x")[np.newaxis, :], axis=0)
        u_cmd_over_horizon = np.append(u_cmd_over_horizon, ocp_solver.get(j, "u")[np.newaxis, :] if j < ocp_solver.N else ocp_solver.get(ocp_solver.N - 1, "u")[np.newaxis, :], axis=0)

    mlp_in_over_horizon = np.concatenate([state_over_horizon[:, neural_mpc.state_feats], u_cmd_over_horizon[:, neural_mpc.u_feats]], axis=1)
    
    l4casadi_params = neural_mpc.learned_dyn_model.get_params(mlp_in_over_horizon)

    for j in range(ocp_solver.N + 1):
        neural_mpc.acados_parameters[j, neural_mpc.l4casadi_start_idx : neural_mpc.l4casadi_end_idx] = l4casadi_params[j].flatten()

def set_l4casadi_params_sim(sim_neural_mpc, sim_solver: AcadosOcpSolver, u_cmd: np.ndarray):
    # Set l4casadi parameters
    state_over_horizon = sim_solver.get("x")[np.newaxis, :]
    u_cmd_over_horizon = u_cmd[np.newaxis, :]

    mlp_in_over_horizon = np.concatenate([state_over_horizon[:, sim_neural_mpc.state_feats], u_cmd_over_horizon[:, sim_neural_mpc.u_feats]], axis=1)
    
    l4casadi_params = sim_neural_mpc.learned_dyn_model.get_params(mlp_in_over_horizon)

    sim_neural_mpc.acados_parameters[:, sim_neural_mpc.l4casadi_start_idx : sim_neural_mpc.l4casadi_end_idx] = l4casadi_params.flatten()


def set_delayed_states_as_params(neural_mpc, ocp_solver: AcadosOcpSolver, history_y: np.ndarray, u_cmd: np.ndarray):
    # Set previous state and control as parameters
    if u_cmd is None:
        # Take init values for all nodes at t=0
        neural_mpc.acados_parameters[:, neural_mpc.delay_start_idx : neural_mpc.delay_end_idx] = history_y.copy().flatten()
    else:
        for j in range(ocp_solver.N + 1):
            if j == 0:
                # Use all available history steps from current node with size of the delay horizon
                # Take all available history steps in order of actuality
                running_y = history_y.copy()
            else:
                # Shift history steps from current node with size of the delay horizon
                # Note, delay horizon is sorted from newest to oldest, so discard last indices first
                running_y = running_y[:-1, :]

                # Use predicted states for all previous nodes from current node to fill up delay window
                # Note, this is an approximation since we are using the state prediction from the previous optimization scheme and
                # not the best possible estimate for the previous states
                predicted_x = ocp_solver.get(j, "x")
                if j < ocp_solver.N - 1:
                    predicted_u = ocp_solver.get(j, "u")
                else:
                    # Terminal node
                    predicted_u = [None] * neural_mpc.nu
                running_y = np.append(np.append(predicted_x, predicted_u)[np.newaxis, :], running_y, axis=0)

            # Set acados parameters in OCP solver
            neural_mpc.acados_parameters[j, neural_mpc.delay_start_idx : neural_mpc.delay_end_idx] = running_y.flatten()

def set_delayed_states_as_params_sim(sim_neural_mpc, sim_solver: AcadosOcpSolver, history_y: np.ndarray, u_cmd: np.ndarray):
    raise NotImplementedError("TODO.")

def set_temporal_states_as_params(neural_mpc, ocp_solver: AcadosOcpSolver, history_y: np.ndarray, u_cmd: np.ndarray):
    raise NotImplementedError("TODO.")

def set_temporal_states_as_params_sim(sim_neural_mpc, sim_solver: AcadosOcpSolver, history_y: np.ndarray, u_cmd: np.ndarray):
    raise NotImplementedError("TODO.")


def get_output_mapping(state_dim, y_reg_dims, label_transform=False, only_vz=False):
    M = np.zeros((state_dim, len(y_reg_dims)))
    for i in range(len(y_reg_dims)):
        M[y_reg_dims[i], i] = 1

    if label_transform and only_vz:
        # Special case: MLP only predicts v_z
        # Set mapping for v_x and v_y to 1 to account for their influence from v_z
        M_v_xy = np.zeros((state_dim, 2))
        M_v_xy[3, 0] = 1
        M_v_xy[4, 1] = 1
        M = np.append(M_v_xy, M, axis=1)
    return M


def get_inverse_output_mapping(state_dim, y_reg_dims):
    O = np.eye(state_dim)
    for i in y_reg_dims:
        O[i, i] = 0
    return O


def get_device():
    if torch.cuda.is_available():
        device = "cuda"
    elif torch.backends.mps.is_available():
        device = "mps"
    else:
        device = "cpu"
    print(f"Using {device} device")
    return torch.device(device)


def load_model(model_options, sim_options, run_options, device) -> Union[MLP, VAE]:
    """
    Load a pre-trained neural network for the MPC controller.
    """
    # Load metadata from trained model
    neural_model_name = model_options["neural_model_name"]
    neural_model_instance = model_options["neural_model_instance"]
    json_file_name = os.path.join(DirectoryConfig.SAVE_DIR, "metadata.json")
    with open(json_file_name, "r") as json_file:
        metadata = json.load(json_file)[neural_model_name][neural_model_instance]

    # Cross-check simulation environment options with metadata
    if sim_options is not None and run_options is not None:
        cross_check_options(model_options, sim_options, run_options, metadata)

    # Define trained MLP model
    file_name = os.path.join(DirectoryConfig.SAVE_DIR, neural_model_name, f"{neural_model_instance}.pt")
    saved_dict = torch.load(file_name, map_location=device)

    if metadata["NetworkConfig"]["model_type"] == "MLP":
        neural_model = MLP(
            saved_dict["input_size"],
            saved_dict["hidden_sizes"],
            saved_dict["output_size"],
            activation=saved_dict["activation"],
            use_batch_norm=saved_dict["use_batch_norm"],
            dropout_p=saved_dict["dropout_p"],
            dropout_input=saved_dict["dropout_input"] if "dropout_input" in saved_dict else False,
            x_mean=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            x_std=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            y_mean=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
            y_std=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
        ).to(device)

    elif metadata["NetworkConfig"]["model_type"] == "VAE":
        neural_model = VAE(
            saved_dict["input_size"],
            saved_dict["encoder_hidden_sizes"],
            saved_dict["latent_dim"],
            saved_dict["decoder_hidden_sizes"],
            saved_dict["output_size"],
            activation=saved_dict["activation"],
            use_batch_norm=saved_dict["use_batch_norm"],
            dropout_p=saved_dict["dropout_p"],
            dropout_input=saved_dict["dropout_input"] if "dropout_input" in saved_dict else False,
            x_mean=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            x_std=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
            y_mean=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
            y_std=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
        ).to(device)

    # Load weights and biases from saved model
    neural_model.eval()
    neural_model.load_state_dict(saved_dict["state_dict"])

    for parameters in neural_model.parameters():
        parameters.requires_grad = False

    return neural_model, metadata


def cross_check_options(model_options, sim_options, run_options, metadata):
    """
    Cross-check the model options and simulation options to ensure they match the configuration
    that the neural model was trained with.
    """
    if model_options["mpc_type"] != metadata["ds_mpc_type"]:
        raise ValueError(
            "MPC type used in dataset for training the neural model doesn't match the simulation environment."
        )

    if sim_options["disturbances"]["cog_dist"]:
        if not run_options["real_machine"] and not (
            sim_options["use_nominal_simulator"] or sim_options["use_real_world_simulator"]
        ):
            for dist, value in sim_options["disturbances"].items():
                if dist not in metadata["ds_disturbances"]:
                    raise ValueError(
                        f"Disturbance '{dist}' used in simulation environment is not present in the dataset for training the neural model."
                    )
                if value != metadata["ds_disturbances"][dist]:
                    if dist == "cog_dist_factor":
                        pass
                    else:
                        raise ValueError(
                            f"Disturbance '{dist}' used in dataset for training the neural model doesn't match the simulation environment."
                        )


def cross_check_params(mpc_params, mlp_metadata):
    """
    Cross-check the MPC parameters with the metadata of the MLP model to ensure they match.
    """
    if mlp_metadata["ds_mpc_params"]:
        if mpc_params["T_samp"] != mlp_metadata["ds_mpc_params"]["T_samp"]:
            raise ValueError(
                "Sampling time used in dataset for training the neural model doesn't match the MPC parameters."
        )

        if mpc_params["T_horizon"] != mlp_metadata["ds_mpc_params"]["T_horizon"]:
            raise ValueError(
                "Prediction horizon used in dataset for training the neural model doesn't match the MPC parameters."
            )

        if mpc_params["T_step"] != mlp_metadata["ds_mpc_params"]["T_step"]:
            raise ValueError("Step time used in dataset for training the neural model doesn't match the MPC parameters.")


def sanity_check_features_and_reg_dims(model_name, state_feats, u_feats, y_reg_dims, in_dim, out_dim, delay, temporalize, N):
    """
    Simple check to ensure that the features and regression dimensions are valid.
    """
    if not isinstance(state_feats, list) or not isinstance(u_feats, list) or not isinstance(y_reg_dims, list):
        raise ValueError("state_feats, u_feats, and y_reg_dims must be lists.")

    if len(state_feats) == 0 or len(y_reg_dims) == 0:
        raise ValueError("state_feats and y_reg_dims cannot be empty lists.")

    if temporalize:
        if (len(state_feats) + len(u_feats)) * (N + 1) != in_dim:
            raise ValueError(
                f"Total number of features {(len(state_feats) + len(u_feats)) * (N + 1)} does not match input dimension {in_dim}."
            )
        if len(y_reg_dims) * N != out_dim:
            raise ValueError(
                f"Total number of regression dimensions {len(y_reg_dims) * N} does not match output dimension {out_dim}."
            )
    else:
        if (len(state_feats) + len(u_feats)) * (delay + 1) != in_dim:
            raise ValueError(
                f"Total number of features {(len(state_feats) + len(u_feats)) * (delay + 1)} does not match input dimension {in_dim}."
            )
        if len(y_reg_dims) != out_dim:
            raise ValueError(
                f"Total number of regression dimensions {len(y_reg_dims)} does not match output dimension {out_dim}."
            )

    if delay > 0:
        if "delay" not in model_name:
            raise ValueError(
                "Delay horizon is set but model name does not contain 'delay'. Please use a delayed model."
            )
    else:
        if "delay" in model_name:
            raise ValueError(
                "Delay horizon is not set but model name contains 'delay'. Please use a non-delayed model."
            )
    if temporalize:
        if "temporal" not in model_name:
            raise ValueError(
                "Temporalization is set but model name does not contain 'temporal'. Please use a temporalized model."
            )
    else:
        if "temporal" in model_name:
            raise ValueError(
                "Temporalization is not set but model name contains 'temporal'. Please use a non-temporalized model."
            )
