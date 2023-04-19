import sys
sys.path.append("..")

import json
import re

import numpy as np

from scripts.load_ros_data import *

from scripts.helpers import *

TIMEOUT = 30
MAX_EXPERIMENTS = 100

# trigger dvc ++

# Base

def add_mean_std_max_min(sub_metric, data_array):
    if len(data_array) > 0:
        sub_metric["mean"] = np.mean(data_array)
        sub_metric["std"] = np.std(data_array)
        sub_metric["min"] = np.min(data_array)
        sub_metric["max"] = np.max(data_array)
    else:
        sub_metric["mean"] = -1.
        sub_metric["std"] = -1.
        sub_metric["min"] = -1.
        sub_metric["max"] = -1.

def add_name(method_name, metrics):
    metrics["name"] = method_name

def add_num_experiments(experiment_data, metrics):
    metrics["num experiments"] = len(experiment_data)

def add_num_obstacles(experiment_data, metrics):

    num_obstacles = 0
    pattern = re.compile("^(obstacle_[0-9]+_pose)$")
    for key, value in experiment_data[0].items():
        if pattern.match(key):
            num_obstacles += 1

    metrics["num obstacles"] = num_obstacles


def add_horizon_length(experiment_data, metrics):
    horizon_length = 0
    pattern = re.compile("^(vehicle_plan_[0-9]+)$")
    for key, value in experiment_data[0].items():
        if pattern.match(key):
            horizon_length += 1

    metrics["horizon length"] = horizon_length

def metric_runtime(experiment_data, metrics):
    metrics["runtime"] = dict()
    # all_runtime = np.ones((0, 1))
    # for experiment in experiment_data:
    #     all_runtime = np.vstack([all_runtime, experiment["runtime_control_loop"]])

    add_mean_std_max_min(metrics["runtime"], merge_experiment_data(experiment_data, "runtime_control_loop"))


def metric_duration(experiment_data, metrics, exclude_collisions=True):

    duration = []
    for i, experiment in enumerate(experiment_data):
        if not metrics["timeouts"][i] and (not metrics["collisions"][i] or (not exclude_collisions)):
            duration.append(experiment["metric_duration"])

    metrics["task duration"] = dict()
    add_mean_std_max_min(metrics["task duration"], duration)


def metric_collisions(experiment_data, metrics):

    metrics["collisions"] = []
    metrics["severe collisions"] = []
    for experiment in experiment_data:
        metrics["collisions"].append(
            int(np.max(experiment["metric_collisions"]) > 0)
        )
        metrics["severe collisions"].append(
            int(np.max(experiment["max_intrusion"]) > 0.05)
        )

    metrics["num collisions"] = int(np.sum(metrics["collisions"]))
    metrics["num severe collisions"] = int(np.sum(metrics["severe collisions"]))

def metric_timeouts(experiment_data, metrics):
    metrics["timeouts"] = [int(exp["metric_duration"] >= TIMEOUT) for exp in experiment_data]
    # metrics["timeouts"] = [int(t >= TIMEOUT) for t in data["metric_duration"]]

    metrics["num timeouts"] = len([exp for exp in experiment_data if exp["metric_duration"] >= TIMEOUT])


def metric_infeasible(experiment_data, metrics):
    metrics["num infeasible"] = []
    for experiment in experiment_data:
        in_infeasible = False
        infeasible_stints = 0
        for t in range(len(experiment["status"])):
            if (not in_infeasible) and experiment["status"][t] == 3:
                in_infeasible = True
                infeasible_stints += 1
            elif in_infeasible and experiment["status"][t] == 2:
                in_infeasible = False
        # metrics["num infeasible"].append(len(np.where(experiment["status"] == [3])[0]))
        metrics["num infeasible"].append(infeasible_stints)

def metric_success_rate(experiment_data, metrics):
    success = [1]*len(experiment_data)
    for i in range(len(success)):
        if metrics["severe collisions"][i] or metrics["timeouts"][i]:
            success[i] = 0

    metrics["success"] = success
    metrics["num success"] = int(sum(success))
    metrics["success rate"] = float(metrics["num success"]) / len(experiment_data)


def metric_num_paths(experiment_data, metrics):
    if "n_paths" not in experiment_data[0].keys():
        return

    num_paths = merge_experiment_data(experiment_data, "n_paths")
    num_paths = [value for value in num_paths if value != 1] # If there is one path, there are likely no obstacles

    metrics["num paths"] = dict()
    add_mean_std_max_min(metrics["num paths"], num_paths)

def main(simulation, planner_name, project=PROJECT_FOLDER):
    data_folder, metric_folder, _ = get_folder_names(project)

    filename = simulation + "_" + planner_name

    print(bcolors.HEADER + "----- Metrics.py ------" + bcolors.ENDC)
    print("Computing metrics...")
    print_value("Planner", planner_name)
    print_value("Simulation", simulation)

    data_path = data_folder + filename + ".txt"

    metric_folder = metric_folder + simulation
    metric_path = metric_folder + "/" + planner_name + ".json"
    data = load_ros_data(data_path)
    experiment_data = split_ros_data(data, num_experiments=MAX_EXPERIMENTS)
    print("Data has {} fields and {} timesteps.".format(len(data.keys()), len(data["runtime_control_loop"])))

    metrics = dict()
    # All experiment wise metrics
    add_name(planner_name, metrics)
    add_num_experiments(experiment_data, metrics)
    if metrics['num experiments'] == 0:
        print("No Experiments Found")
    else:
        add_num_obstacles(experiment_data, metrics)
        metric_collisions(experiment_data, metrics)
        metric_timeouts(experiment_data, metrics)
        metric_infeasible(experiment_data, metrics)

        metric_duration(experiment_data, metrics, exclude_collisions=False) # Not sure if set to False for table
        metric_runtime(experiment_data, metrics)
        metric_success_rate(experiment_data, metrics)

        metric_num_paths(experiment_data, metrics)

    print(bcolors.OKGREEN + "[Results]:" + bcolors.ENDC)
    for key, value in metrics.items():
        print_value(key, value, tab=True)

    os.makedirs(metric_folder, exist_ok=True)
    with open(metric_path, 'w') as f:
        json.dump(metrics, f)

    print(bcolors.HEADER + "----------------------" + bcolors.ENDC)
    print(bcolors.OKGREEN + "Done!" + bcolors.ENDC + " Saved result to " + metric_path)


if __name__ == '__main__':
    simulation = sys.argv[1]
    planner_name = sys.argv[2]
    main(simulation, planner_name)