import numpy as np
from acados_template import AcadosSimSolver

# NeuralMPC is referenced in the annotations below as a string rather than
# imported: both controller classes need any_cog_disturbance() from this module
# to decide whether to allocate the CoG parameter slot, and importing them back
# would close the cycle.


def apply_cog_disturbance(sim_solver: AcadosSimSolver, neural_mpc: "NeuralMPC", cog_dist_factor, u_cmd, state):
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


def _eval(value, t, default=0.0):
    """
    Resolve a possibly time-varying disturbance magnitude.

    Every disturbance magnitude may be given either as a plain number (constant,
    backward-compatible) or as a callable f(t) -> number, where t is the current
    simulation time in seconds. This lets a disturbance change over time — e.g. a
    payload that appears at t=30 s, or an oscillating crosswind — which is exactly
    the regime online learning is meant to adapt to.
    """
    if value is None:
        return default
    if callable(value):
        return float(value(t))
    return float(value)


# The disturbance sources that apply_cog_disturbances() knows how to apply.
# The simulation loop skips the call entirely when none of them is enabled, and
# it MUST test this tuple rather than its own list of names: "drag" spent its
# whole life as a configuration flag that no code path ever read, because the
# guard in the loop enumerated three sources by hand. Adding a name here now
# wires it up everywhere at once.
COG_SOURCES = ("extra_mass", "ground_effect", "wind", "drag")


def any_cog_disturbance(sim_options) -> bool:
    """True when at least one CoG-slot disturbance source is enabled."""
    d = sim_options["disturbances"]
    return any(d.get(k, False) for k in COG_SOURCES)


def _eval_vec(value, t, n=3, default=0.0):
    """
    Vector form of _eval(), for per-axis coefficients.

    Accepts a scalar (broadcast to all n axes), a sequence of n values, or a
    callable f(t) returning either. A scalar therefore means "isotropic", which
    is the common case and keeps the configuration short.
    """
    if value is None:
        return np.full(n, float(default))
    if callable(value):
        value = value(t)
    v = np.asarray(value, dtype=float).ravel()
    if v.size == 1:
        return np.full(n, float(v[0]))
    if v.size != n:
        raise ValueError(f"expected a scalar or {n} components, got {v.size}")
    return v


def apply_cog_disturbances(
    sim_solver: AcadosSimSolver,
    sim_neural_mpc: "NeuralMPC",
    sim_options: dict,
    u_cmd,
    state,
    t: float = 0.0,
    g: float = 9.81,
) -> None:
    """
    Overlay every CoG-slot force/torque disturbance and push them with a SINGLE
    set("p", ...).  Contributions are SUMMED into the same world-frame CoG force
    slot, so several sources can be active at once.

    Time dependence
    ---------------
    Every magnitude below (extra_mass_kg, ground_effect_k/z0, wind_x/wind_y) may
    be a constant OR a callable f(t) -> value evaluated at the current simulation
    time t (seconds). See _eval(). Constants keep the previous behaviour.

    Sources currently handled
    -------------------------
    extra_mass    : payload rigidly attached at the CoG.  Downward weight,
                    F_z = -extra_mass_kg(t) * g.  No torque (mass at the CoG).
    ground_effect : height-dependent extra lift near the ground (the rotor
                    downwash reflects off the floor and pushes the drone up):
                        F_z = +T_vert * k_ge(t) / (1 + (z / z0(t))^2)   (upward)
                    with
                        T_vert = Σ u_cmd[:4]  — current total rotor thrust,
                                 a near-hover proxy for the vertical thrust that
                                 produces the downwash (so the effect vanishes
                                 when the motors are idle),
                        z      = height above ground (state[2], clamped >= 0),
                        z0     = characteristic height (effect halves at z0),
                        k_ge   = strength = fraction of vertical thrust returned
                                 as extra lift at z -> 0.
    wind          : constant horizontal force in the WORLD frame, x and y only
                    (never z, to keep the simulation simple):
                        F_x = wind_x(t),   F_y = wind_y(t).
                    Set one axis to 0 to have wind on a single axis.
    drag          : aerodynamic drag, opposing the world-frame velocity, as the
                    usual linear + quadratic polynomial:
                        F = -( k1(t) * v  +  k2(t) * ||v|| * v )
                    with v = state[3:6] the world-frame velocity, k1 in
                    N/(m/s) and k2 in N/(m/s)^2, each a scalar (isotropic) or
                    one coefficient per axis. The linear term stands for rotor
                    drag and blade flapping, which dominate at low speed; the
                    quadratic term for body drag, which takes over as speed
                    rises. ||v|| (not |v_i|) multiplies the quadratic term, so
                    the force stays anti-parallel to the velocity whatever the
                    direction of travel.

                    Unlike the three above, drag is a function of the STATE, not
                    of time alone: a constant-force estimator cannot represent
                    it, which is precisely why it is worth simulating here.

    Parameters
    ----------
    sim_solver     : AcadosSimSolver — simulator solver to update (compiled C
                     solver, via set("p", ...)).
    sim_neural_mpc : NeuralMPC of the SIMULATOR — provides the base parameter
                     vector (acados_parameters[0, :], whose length matches this
                     sim solver) and cog_dist_start_idx / cog_dist_end_idx.  The
                     simulator's parameter count can differ from the controller's
                     (the online controller carries MLP-weight parameters, the
                     nominal simulator does not), so the vector MUST come from
                     the sim model.
    sim_options    : full sim_options dict (reads sim_options["disturbances"]).
    u_cmd          : current control command (rotor thrusts in u_cmd[:4]); may
                     be None on the very first step.
    state          : current simulator state (state[2] = height above ground).
    t              : current simulation time [s] (for time-varying magnitudes).
    g              : gravitational acceleration in m/s² (default 9.81).

    IMPORTANT: the disturbance only reaches the compiled integrator through
    set("p", ...).  Directly mutating sim_solver.acados_sim.parameter_values has
    NO effect — that numpy attribute is a Python-side descriptor the compiled
    simulate()/solve() never reads; only set("p", ...) ->
    <model>_acados_sim_update_params reaches the C solver.  The sim model's own
    base parameter vector is copied and re-sent here with the accumulated CoG
    force overlaid, so this call is self-contained and REPLACES the base
    set("p", ...).  The MPC controller's parameters are untouched -> it keeps
    planning as if undisturbed.
    """
    dist  = sim_options["disturbances"]
    start = sim_neural_mpc.cog_dist_start_idx
    end   = sim_neural_mpc.cog_dist_end_idx

    # Accumulated [fx, fy, fz, tau_x, tau_y, tau_z] (world force / body torque).
    f_cog = np.zeros(end - start)

    # --- payload weight, downward (magnitude may vary with time) ---
    if dist.get("extra_mass", False):
        f_cog[2] += -_eval(dist.get("extra_mass_kg"), t) * g

    # --- ground effect (upward lift, grows near the ground, scales with thrust) ---
    if dist.get("ground_effect", False):
        z      = max(float(state[2]), 0.0)
        T_vert = float(np.sum(u_cmd[:4])) if u_cmd is not None else 0.0
        k_ge   = _eval(dist.get("ground_effect_k"), t, default=0.2)
        z0     = _eval(dist.get("ground_effect_z0"), t, default=0.5)
        f_cog[2] += T_vert * k_ge / (1.0 + (z / z0) ** 2)

    # --- wind (horizontal force, world frame; x and y only, never z) ---
    if dist.get("wind", False):
        f_cog[0] += _eval(dist.get("wind_x"), t)
        f_cog[1] += _eval(dist.get("wind_y"), t)

    # --- aerodynamic drag (opposes the world-frame velocity) ---
    if dist.get("drag", False):
        v = np.asarray(state[3:6], dtype=float)
        k1 = _eval_vec(dist.get("drag_linear"), t, 3)
        k2 = _eval_vec(dist.get("drag_quadratic"), t, 3)
        f_cog[:3] += -(k1 * v + k2 * float(np.linalg.norm(v)) * v)

    p = sim_neural_mpc.acados_parameters[0, :].copy()
    p[start:end] = f_cog
    sim_solver.set("p", p)


def apply_motor_noise(sim_solver: AcadosSimSolver, neural_mpc: "NeuralMPC", u_cmd):
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
