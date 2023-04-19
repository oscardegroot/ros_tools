#!/usr/bin/env python3

import sys, os
sys.path.append("..")

import rospkg
import yaml
import dvc.api
params = dvc.api.params_show()

def get_config_path():
    config_path = rospkg.RosPack().get_path('lmpcc') + "/config/jackal/"
    config_name = "experiment_parameters.yaml"

    return config_path, config_name

def remove_config():
    write_config(dict()) # Write an empty yaml
    print("Removed experiment parameters!")

def write_config(params):
    config_path, config_name = get_config_path()
    with open(config_path + config_name, "w") as outfile:
        yaml.dump(params, outfile, default_flow_style=False)

    print("Wrote experiment parameters to " + config_path + config_name)

def static():
    params = dict()
    params["prm"] = dict()
    params["prm"]["enable"] = dict()
    params["prm"]["enable"]["dynamically_propagate_nodes"] = False

    params["enable_output"] = False

    write_config(params)
    return params


def static_uvd_failure():
    params = static() # Start with the static experiment setup
    params["recording"]["enable_recording"] = False

    params["prm"]["n_paths"] = 4 # More paths
    params["prm"]["n_samples"] = 8000 # More samples
    params["prm"]["timeout"] = 50  # ms
    params["prm"]["topology_comparison"] = "UVD"  # ms

    params["recording"]["time_out"] = 60
    params["recording"]["number_of_experiments"] = 10

    params["guidance"]["constraints"]["add_original_problem"] = False
    write_config(params)
    return params


def static_homology_success():
    params = static_uvd_failure()
    params["prm"]["topology_comparison"] = "Homology"  # ms
    write_config(params)
    return params


def dynamic_uvd():
    params = dict()
    params["recording"] = dict()
    params["recording"]["number_of_experiments"] = 1
    params["recording"]["enable_recording"] = True
    params["recording"]["prepend_name"] = "UVD-"

    params["prm"] = dict()
    params["prm"]["topology_comparison"] = "UVD"  # ms
    params["prm"]["n_paths"] = 7

    params["guidance"] = dict()
    params["guidance"]["constraints"] = dict()
    params["guidance"]["constraints"]["add_original_problem"] = False
    write_config(params)
    return params

def dynamic_homology():
    params = dynamic_uvd()
    params["prm"]["topology_comparison"] = "Homology"  # ms
    params["recording"]["prepend_name"] = "Homology-"

    write_config(params)
    return params


if __name__ == '__main__':
    experiment = sys.argv[1]

    if experiment == "remove":
        remove_config()
    else:
        try:
            func_to_run = globals()[experiment]
            if len(sys.argv) > 2:
                func_to_run(sys.argv[2:])
            else:
                func_to_run()
        except KeyError:
            raise IOError("Experiment configuration does not exist!")

    # simulation = sys.argv[1]
    # planner_name = sys.argv[2]
    # with warnings.catch_warnings():
    #     warnings.simplefilter("ignore")
    #     main(simulation, planner_name)
