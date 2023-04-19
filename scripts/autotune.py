PROJECT_NAME = "autotune"

import helpers
data_folder, metric_folder, figure_folder = helpers.get_folder_names(PROJECT_NAME)


import metrics as lmpcc_metrics
import json, yaml
from load_ros_data import *

import optuna

from optuna.visualization import plot_optimization_history
from optuna.visualization import plot_param_importances

import subprocess


simulations = []
simulations.append("random-8_straight")
simulations.append("random-16_straight")
simulations.append("crossing-16_random")

def define_parameters(trial):
    params = dict()
    params["prm"] = dict()
    params["prm"]["n_samples"] = trial.suggest_int("prm/n_samples", 50, 400)

    params["prm"]["weights"] = dict()
    params["prm"]["weights"]["geometric"] = trial.suggest_float("prm/weights/geometric", 40, 50.)
    params["prm"]["weights"]["smoothness"] = trial.suggest_float("prm/weights/smoothness", 60, 80.)
    params["prm"]["weights"]["collision"] = trial.suggest_float("prm/weights/collision", 2, 3.)
    params["prm"]["weights"]["velocity_tracking"] = trial.suggest_float("prm/weights/velocity_tracking", 0.4, 7.)

    params["prm"]["selection_weights"] = dict()
    params["prm"]["selection_weights"]["consistency"] = trial.suggest_float("prm/selection_weights/consistency", 0.0, 0.4)

    # params["prm"]["spline"] = dict()
    # params["prm"]["spline"]["num_points"] = trial.suggest_int("prm/spline/num_points", 15, 25)

    params["prm"]["prefer_goal_over_smoothness"] = trial.suggest_float("prm/prefer_goal_over_smoothness", 0.6, 1.0)
    # params["prm"]["view_angle_times_pi"] = trial.suggest_float("prm/view_angle_times_pi", 0.4, 1.0)
    params["prm"]["max_acceleration"] = trial.suggest_float("prm/max_acceleration", 2., 2.5)
    # params["prm"]["obstacle_radius_extension"] = trial.suggest_float("prm/obstacle_radius_extension", 0., 0.5) # Was there in the non-extended case

    params["weights"] = dict()
    params["weights"]["acceleration"] = trial.suggest_float("weights/acceleration", 0.01, 1.0)
    params["weights"]["angular velocity"] = trial.suggest_float("weights/angular_velocity", 0.01, 1.0)
    params["weights"]["contour"] = trial.suggest_float("weights/contour", 0.01, 1.0)
    params["weights"]["lag"] = trial.suggest_float("weights/lag", 0.01, 1.0)
    params["weights"]["velocity"] = trial.suggest_float("weights/velocity", 0.01, 1.0)

    return params

def run_lmpcc(simulation):
    simulation_scenario = simulation.replace("-", "/") + ".xml"
    run_command = "source ~/.zshrc && source devel/setup.zsh && roslaunch lmpcc jackalsimulator.launch"
    run_command += " overwrite_pedestrian_scenario:=true"
    run_command += " pedestrian_scenario:=" + simulation_scenario
    run_command += " project_name:=" + PROJECT_NAME

    subprocess.call(run_command, shell=True, executable='/bin/zsh')

def write_to_yaml(params):
    config_path = os.getcwd() + "/../../lmpcc/config/jackal/"
    with open(config_path + "jackalsimulator_autotune_parameters.yaml", "w") as outfile:
        yaml.dump(params, outfile, default_flow_style=False)

def get_metrics(simulation, planner):
    lmpcc_metrics.main(simulation, planner, PROJECT_NAME)

    # Load the metrics
    metric_folder_sim = metric_folder + simulation
    metric_path = metric_folder_sim + "/" + planner + ".json"

    print("Loading JSON: {}".format(str(metric_path)))
    with open(metric_path) as f:
        metrics = dict(json.load(f))

    return metrics

def objective_value(metrics):

    cost = 0.
    print(cost)
    duration = metrics["task duration"]["mean"] / lmpcc_metrics.TIMEOUT if metrics["task duration"]["mean"] != -1 else 10
    cost += duration
    print(cost)

    cost += metrics["num severe collisions"]
    print(cost)

    cost += metrics["num timeouts"]
    print(cost)

    cost += max(metrics["runtime"]["mean"]*1000. + 3*metrics["runtime"]["std"] - 50., 0) # How many times are we too slow?
    print(cost)

    return cost


def lmpcc_trial(trial):

    # simulation = "random-16_straight"
    planner = "GMPCC"

    params = define_parameters(trial)
    write_to_yaml(params)

    cost = 0
    for step, simulation in enumerate(simulations): # Stack metrics of all simulations
        run_lmpcc(simulation)
        metrics = get_metrics(simulation, planner)
        cost += objective_value(metrics)
        trial.report(cost, step)

        # Handle pruning based on the intermediate value.
        if trial.should_prune():
            raise optuna.TrialPruned()

    return cost


if __name__ == '__main__':
    study_path = "/studies/"
    os.makedirs(os.getcwd() + study_path, exist_ok=True)

    load_study = True
    delete_study = False
    # study_name = "autotune-test"
    # study_name = "autotune-spline-cost"
    # study_name = "autotune-splines"
    # study_name = "autotune-jackalsimulator" # Continue this later
    study_name = "autotune-jackalsimulator-extended" # Continue this later

    # full_study_name = study_path + study_name

    if delete_study:
        inp = input("Are you sure you want to delete the " + study_name + " study? (y/n) ")
        if inp == 'y':
            optuna.delete_study(storage="sqlite:///" + study_name,
                                study_name=study_name)
            print("Deleted study: " + study_name)
        else:
            raise IOError("Please set delete_study to False")

    study = optuna.create_study(direction="minimize",
                                storage="sqlite:///" + study_name,
                                study_name=study_name,
                                pruner=optuna.pruners.MedianPruner(),
                                load_if_exists=load_study)
    study.optimize(lmpcc_trial, n_trials=25)

    print(study.best_trial)

    plot_param_importances(study).show()
    plot_optimization_history(study).show()

