"""
Integrate command from the actuator velocities optimized in the differential MPCs (as getCommand() of the C++)
"""
import numpy as np


def get_command_zoh_lead(x0, uo_vel, T_samp, tau, x_min, x_max):
    """
    ZOH-exact lead:
    The MPC plans the actuator state x(t) = x0 + uo_vel * t from the estimated state x0 at the actuation
    time. Therefore, the state follows exactly
      x1 - x0 = T_step * uo_vel, and
      x(T_samp) - x0 = T_samp * uo_vel,
    since the actuator states are pure integrators of uo_vel. The command uo_cmd is held for
    one control period T_samp, in which the first-order actuator
      tau * dx/dt + x = uo_cmd
    moves by
      x(T_samp) - x0 = (uo_cmd - x0) * (1 - exp(-T_samp / tau)).
    So the command
      uo_cmd = x0 + uo_vel * T_samp / (1 - exp(-T_samp / tau))
    realizes exactly the planned increment of uo_vel * T_samp.
    The lead satisfies the following approximation for small T_samp / tau:
       T_samp / (1 - exp(-T_samp / tau)) ~ tau + T_samp / 2,
    whereas the continuous-time inversion x0 + tau * uo_vel neglects the hold and realizes only 73 %
    of the planned increment (with tau = 15 ms, T_samp = 10 ms).
    :param x0: Actuator state at the actuation time, i.e., the initial state of the MPC
    :param uo_vel: Optimized actuator velocity of the first shooting interval
    :param T_samp: Control period [s]
    :param tau: Time constant of the actuator [s]
    :param x_min: Lower actuator limit
    :param x_max: Upper actuator limit
    :return: Actuator command
    """
    lead = T_samp / (1.0 - np.exp(-T_samp / tau))
    uo_cmd = x0 + lead * uo_vel
    return np.clip(uo_cmd, x_min, x_max)


def get_command_leaky_integrator(u_prev, x_meas, uo_vel, dt, tau, x_min, x_max, T_leak=0.4):
    """
    Leaky integrator:
    Integrates the optimized actuator velocity and leaks slowly to the measurement-consistent lead
        x_meas + tau * uo_vel,
    so that the open-loop drift stays bounded.
    :param u_prev: Previous actuator command
    :param x_meas: Measured actuator state
    :param uo_vel: Optimized actuator velocity of the first shooting interval
    :param dt: Control period [s]
    :param tau: Time constant of the actuator [s]
    :param x_min: Lower actuator limit
    :param x_max: Upper actuator limit
    :param T_leak: Time constant of the leak [s], must be >> tau
    :return: Actuator command
    """
    uo_cmd = u_prev + uo_vel * dt
    uo_cmd += dt / T_leak * ((x_meas + tau * uo_vel) - uo_cmd)
    return np.clip(uo_cmd, x_min, x_max)
