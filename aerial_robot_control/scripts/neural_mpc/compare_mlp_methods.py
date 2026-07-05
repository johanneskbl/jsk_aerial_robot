import copy
import numpy as np
import matplotlib.pyplot as plt

from trajectory_tracking_and_record import run_simulation
from utils.visualization_utils import plot_trajectory_comparison, plot_disturbances
from config.configurations import EnvConfig


def main(model_options, solver_options, dataset_options, sim_options, run_options):
    """
    Run Nominal MPC, Static MLP and Online MLP simulations and compare them on the same plots.
    All three runs use the same random seed (sim_options["seed"]) and a dedicated RNG for
    trajectory selection so they follow the exact same trajectory sequence.
    """
    # All runs: no file recording, populate dist_dict, no real-time plot
    run_options_cmp = run_options.copy()
    run_options_cmp["recording"]       = False
    run_options_cmp["plot_trajectory"] = True
    run_options_cmp["real_time_plot"]  = False

    print("=" * 60)
    print("Run 1/3: Nominal MPC (useMLP=False)")
    print("=" * 60)
    dataset_nominal, dist_nominal, mpc_nominal = run_simulation(
        copy.deepcopy(model_options), solver_options, dataset_options,
        sim_options, run_options_cmp,
        useMLP=False, onlineMLP=False,
    )

    print("=" * 60)
    print("Run 2/3: Static MLP (onlineMLP=False)")
    print("=" * 60)
    dataset_static, dist_static, mpc_static = run_simulation(
        copy.deepcopy(model_options), solver_options, dataset_options,
        sim_options, run_options_cmp,
        useMLP=True, onlineMLP=False,
    )

    print("=" * 60)
    print("Run 3/3: Online MLP (onlineMLP=True)")
    print("=" * 60)
    dataset_online, dist_online, mpc_online = run_simulation(
        copy.deepcopy(model_options), solver_options, dataset_options,
        sim_options, run_options_cmp,
        useMLP=True, onlineMLP=True,
    )

    print("=" * 60)
    print("Plotting comparison...")
    print("=" * 60)
    plot_trajectory_comparison(
        model_options, sim_options,
        dataset_static._rec, dataset_online._rec,
        mpc_static, mpc_online,
        rec_nominal=dataset_nominal._rec,
        neural_mpc_nominal=mpc_nominal,
        dist_dict_static=dist_static,
        dist_dict_online=dist_online,
        dist_dict_nominal=dist_nominal,
        save=run_options["save_figures"],
    )
    plt.show()

    print("Done.")
    print(f"Nominal MPC — recorded: {dataset_nominal.n_recorded:5d} steps")
    print(f"Static  MLP — recorded: {dataset_static.n_recorded:5d} steps")
    print(f"Online  MLP — recorded: {dataset_online.n_recorded:5d} steps"
          f"  | trained: {dataset_online.n_samples}/{dataset_online.buffer_size} samples")


if __name__ == "__main__":
    main(
        EnvConfig.model_options,
        EnvConfig.solver_options,
        EnvConfig.dataset_options,
        EnvConfig.sim_options,
        EnvConfig.run_options,
    )
