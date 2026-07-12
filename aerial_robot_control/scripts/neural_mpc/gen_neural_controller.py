from config.configurations import EnvConfig
from neural_controller import NeuralMPC

model_options = EnvConfig.model_options
sim_options = EnvConfig.sim_options
sim_options["disturbances"]["cog_dist"] = False
sim_options["disturbances"]["motor_noise"] = False
sim_options["disturbances"]["drag"] = False
sim_options["disturbances"]["payload"] = False

controller_list = []
# controller_list.append("nominal")
# controller_list.append("neural")

if "nominal" in controller_list:
    print("Generating nominal controller...")
    model_options["only_use_nominal"] = True
    solver_options = EnvConfig.solver_options
    solver_options["include_delta_u"] = False
    neural_mpc = NeuralMPC(
        model_options=model_options,
        solver_options=solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    print("Successfully generated nominal controller!")
    print("========================================")

if "neural" in controller_list:
    print("Generating neural controller...")
    model_options["only_use_nominal"] = False
    neural_mpc = NeuralMPC(
        model_options=model_options,
        solver_options=EnvConfig.solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    print("Successfully generated neural controller!")
    print("========================================")
