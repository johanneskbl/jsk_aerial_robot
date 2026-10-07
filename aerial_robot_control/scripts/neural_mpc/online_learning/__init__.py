"""
Online residual-model adaptation for the tiltrotor NMPC.

core/    what runs during flight: the training buffer and its residual labels,
         the Adam adaptation law, the safety layer bounding what it may do to
         the weights, and the MPC carrying a parametric copy of the network
         inside its acados solver.
tools/   what runs on the ground: harness.py (fly and measure), tune_online.py
         (search the hyperparameters), evaluate.py (measure named
         configurations), figures.py (draw them).
tests/   the suite, none of which needs acados or a simulation.
results/ everything this package produced: tuning runs, figures, animations.

Deliberately NOT here: neural_controller.py, the nominal and offline path, and
trajectory_tracking_and_record.py, the harness that flies all three modes so the
baselines and the adaptive runs stay comparable. Both are shared with the
offline neural MPC and live at the neural_mpc root.

Run the tools as modules, from the neural_mpc directory:

    python3 -m online_learning.tools.evaluate
    python3 -m online_learning.tools.tune_online
    python3 -m online_learning.tools.figures
"""
