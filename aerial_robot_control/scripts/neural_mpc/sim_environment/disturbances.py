import numpy as np
from acados_template import AcadosSimSolver
from neural_controller import NeuralMPC


def apply_cog_disturbance(sim_solver: AcadosSimSolver, neural_mpc: NeuralMPC, cog_dist_factor, u_cmd, state):
    """
    Function to generate a random disturbance force and torque on the center of gravity (CoG).
    This is a placeholder function that can be expanded with specific disturbance parameters.
    """
    # Get average of thrust command for smoother disturbance
    # u_sequence = np.array([neural_mpc.ocp_solver.get(i, "u") for i in range(int(neural_mpc.N/5))])
    # max_thrust = np.average(u_sequence[:,:4])
    max_thrust = np.average(u_cmd[:4])
    # Ground effect increases lift the closer drone is to the ground
    # Force values behave in [-thrust_max, thrust_max]
    z = state[2]
    # force_mu_z = min(1 / (z+1)**2, 1) * 0.3 * max_thrust * 4
    force_mu_z = 1 / (abs(z) + 1) ** 2 * cog_dist_factor * max_thrust * 4
    force_std_z = 0  # 0.01 * max_thrust

    force_mu_x = 0.0
    force_mu_y = 0.0
    force_std_x = 0  # 0.05 * max_thrust
    force_std_y = 0  # 0.05 * max_thrust

    # Torque values behave in [-2, 2], with thrust_max = 30
    torque_mu = 0  # 0.5
    torque_std = 0  # 0.05 * 1/15 * max_thrust

    mu = np.array([force_mu_x, force_mu_y, force_mu_z, torque_mu, torque_mu, torque_mu])
    std = np.array([force_std_x, force_std_y, force_std_z, torque_std, torque_std, torque_std])
    cog_dist = mu  # np.random.normal(loc=mu, scale=std)
    start_idx = neural_mpc.cog_dist_start_idx
    end_idx = neural_mpc.cog_dist_end_idx
    sim_solver.acados_sim.parameter_values[start_idx:end_idx] = cog_dist


def apply_extra_mass(
    sim_solver: AcadosSimSolver,
    neural_mpc: NeuralMPC,
    extra_mass_kg: float,
    g: float = 9.81,
) -> None:
    """
    Model a fixed extra payload rigidly attached at the CoG.

    Adds a constant downward gravitational force from the extra mass into the
    CoG disturbance parameter slot.  No torque is produced because the mass
    sits exactly at the center of gravity.

    World-frame convention: z-up, so the extra gravitational force is negative
    in the z component: force_z = -extra_mass_kg * g.

    Parameters
    ----------
    sim_solver    : AcadosSimSolver — simulator solver whose parameter values
                    will be updated.
    neural_mpc    : NeuralMPC — provides cog_dist_start_idx / cog_dist_end_idx
                    to locate the disturbance slot in the parameter vector.
    extra_mass_kg : payload mass in kilograms (must be >= 0).
    g             : gravitational acceleration in m/s² (default 9.81).
    The function must be called AFTER sim_solver.set("p", ...) so that the
    disturbance is overlaid on top of the base parameters instead of being
    overwritten.  It writes directly into the sim solver's own parameter
    buffer (same pattern as apply_cog_disturbance / apply_motor_noise),
    rather than rebuilding a full parameter vector from neural_mpc — the
    controller and the simulator model can have different parameter counts
    (e.g. the controller carries MLP weight parameters, the simulator does
    not), so resending a controller-sized vector to the sim solver would
    mismatch its expected length.
    The MPC controller's acados_parameters are not modified, so the controller
    continues to plan as if no external disturbance is present.
    """
    start_idx = neural_mpc.cog_dist_start_idx
    end_idx   = neural_mpc.cog_dist_end_idx
    sim_solver.acados_sim.parameter_values[start_idx:end_idx] = 0.0
    sim_solver.acados_sim.parameter_values[start_idx + 2] = -extra_mass_kg * g  # world z-up → downward


def apply_motor_noise(sim_solver: AcadosSimSolver, neural_mpc: NeuralMPC, u_cmd):
    # Thrust noise
    if u_cmd is None:
        thrust_factor = np.ones((4,))
    else:
        # Use last thrust command for normalization
        thrust_factor = u_cmd[:4] / neural_mpc.params["thrust_max"]
    amplitude_mu = 0.06 * thrust_factor**2
    amplitude_std = 0.08
    mu = np.random.uniform(-amplitude_mu, amplitude_mu)
    std = np.abs(thrust_factor) * amplitude_std ** (1 / 4)
    rotor_noise = np.random.normal(loc=mu, scale=std)

    start_idx = neural_mpc.motor_noise_start_idx
    end_idx = neural_mpc.motor_noise_end_idx
    sim_solver.acados_sim.parameter_values[start_idx : start_idx + 4] = rotor_noise

    # Servo angle noise
    if neural_mpc.tilt:
        mu = np.zeros((4,))
        std = 0.04  # Assume constant inaccuracy for servo angles since they have low frequency
        servo_noise = np.random.normal(loc=mu, scale=std)

        sim_solver.acados_sim.parameter_values[start_idx + 4 : end_idx] = servo_noise
