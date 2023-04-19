#!/usr/bin/env python3

import os, sys
sys.path.append("..")
print(os.getcwd())

import json

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import pandas as pd
import seaborn as sns

from scripts.load_ros_data import *
from scripts.helpers import *
from scripts.metrics import add_num_obstacles, add_horizon_length

plt.rcParams['text.usetex'] = True
font = {'family' : 'computer modern',
        'weight' : 'bold',
        'size'   : 22}

plt.rc('font', **font)
fig_width = 7.02625 # /textwidth in latex in inches

cmap_viridis = matplotlib.cm.get_cmap('viridis')
cmap_viridis_r = matplotlib.cm.get_cmap('inferno')

COMPARE_VISUAL = True

def save_figure_as(fig, figure_folder, figure_name, ratio=1.0):
    fig.set_size_inches(fig_width, fig_width*ratio)

    os.makedirs(figure_folder, exist_ok=True)
    fig.savefig(f'{figure_folder}/{figure_name}', bbox_inches='tight')
    print(f'Saving figure as: {figure_folder}/{figure_name}')

def subplot_signal(ax, signal, **kwargs):

    indices = range(0, len(signal))
    ax.step(indices, signal, **kwargs)

    ax.set_aspect('auto', adjustable='box')

def subplot_experiment_signal(ax, experiment_data, signal, **kwargs):

    d = merge_experiment_data(experiment_data, signal)
    indices = range(0, len(d))

    ax.step(indices, d, **kwargs)  # , marker=m[i % len(m)],markersize=15,markevery=5)

    ax.set_aspect('auto', adjustable='box')
    # ax.set_ylim([-0.01, 40])
    # ax.set_xlim([0, 300])
    # ax.xlabel('Iteration')
    # ax.set_ylabel('Objective Value')
    return ax


def compare_signals(experiment_data, signal_list, highlight_signal_list=[], filter=lambda value:False):
    fig = plt.figure()
    ax = plt.gca()
    m = ['.', 'x', '*']

    for i, signal in enumerate(signal_list):
        d = merge_experiment_data(experiment_data, signal)
        indices = range(0, len(d))

        # Filter if necessary
        indices = [index for index in indices if (not filter(d[index]))]
        d = [value for value in d if (not filter(value))]

        plt.step(indices, d, color='C0', linewidth=3, alpha=0.9)#, marker=m[i % len(m)],markersize=15,markevery=5)

    # HIGHLIGHTED
    for signal in highlight_signal_list:
        d = merge_experiment_data(experiment_data, signal)
        indices = range(0, len(d))

        # Filter if necessary
        indices = [index for index in indices if (not filter(d[index]))]
        d = [value for value in d if (not filter(value))]
        # plt.step(indices, d, color='C3', linestyle='--', linewidth=2)
        plt.step(indices, d, color='C3', linestyle='-', linewidth=2)

    ax.set_aspect('auto', adjustable='box')
    plt.xlabel('Iteration')
    plt.ylabel('Objective Value')

    return fig
    # sns.boxplot(data=df) #y='{} (ms)'.format(signal_list[0])

# https://stackoverflow.com/questions/44552489/plotting-multiple-boxplots-in-seaborn
def signal_boxplot(data_list, data_name, range_list, range_name):
    fig = plt.figure()

    dfs = []
    for i, data_item in enumerate(data_list):
        dfs.append(pd.DataFrame({data_name:data_item, range_name:range_list[i]}))
        # dfs[-1][range_name] = range_list[i]

    cdf = pd.concat(dfs)
    print(cdf.head())
    # mdf = pd.melt(cdf, id_vars=['Trial'], var_name=['Number'])  # MELT

    # sns.set_style("whitegrid")
    sns.boxplot(x=range_name, y=data_name, data=cdf)  # RUN PLOT
    return fig
    # plt.show()
    # sns.boxplot(y='runtime (ms)', data=df)

def plot_plan(data, t, plan_list, highlighted_plan_list=[]):

    fig = plt.figure(tight_layout=True)
    ax = plt.gca()

    # Choose which plan to plot
    experiment_data = data[0]
    time_instance = t

    plan_name = lambda name, k: name + str(k)

    for index, current_plan_name in enumerate(plan_list):
        # Plot the plan along the horizon
        horizon = 0
        for k in range(0, 1000):
            if not plan_name(current_plan_name, k) in experiment_data:
                horizon = k
                break

        plan = np.zeros((2, horizon))
        for k in range(0, horizon):
            plan[:, k] = experiment_data[plan_name(current_plan_name, k)][time_instance, :]

        plt.plot(plan[0, :], plan[1, :], color='C0', linewidth=3.0, alpha=1.0)
        # plt.scatter(plan[0, :], plan[1, :], color='C0', s=1200, alpha=0.4, label='_nolegend_')
        plt.scatter(plan[0, :], plan[1, :], color='C0', s=200, alpha=1.0, marker='.', label='_nolegend_')

    for index, current_plan_name in enumerate(highlighted_plan_list):
        # Plot the plan along the horizon
        horizon = 0
        for k in range(0, 1000):
            if not plan_name(current_plan_name, k) in experiment_data:
                horizon = k
                break

        plan = np.zeros((2, horizon))
        for k in range(0, horizon):
            plan[:, k] = experiment_data[plan_name(current_plan_name, k)][time_instance, :]

        color_index = index + len(plan_list)
        plt.plot(plan[0, :], plan[1, :], linestyle='--', color='C' + str(color_index), alpha=1.0, linewidth=3.0)
        plt.scatter(plan[0, :], plan[1, :], color='C' + str(color_index), s=100, alpha=1.0, linewidths=8.0, marker='x')

    # Plot the obstacle position
    add_num_obstacles(data, experiment_data)
    for obstacle in range(experiment_data["num obstacles"]):
        pose_name = f"disc_{obstacle}_pose"
        plt.plot(experiment_data[pose_name][time_instance, 0], experiment_data[pose_name][time_instance, 1],
                 marker='.', markersize=15, linewidth=3.0,
                 color='k')
        ax.annotate("Obstacle", xy=(experiment_data[pose_name][time_instance, 0]+0.25, experiment_data[pose_name][time_instance, 1]),
                    horizontalalignment='left',
                    verticalalignment='center',
                    fontsize=16
                    )

    ax.set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    return fig

def plot_linearized_constraint_regions(data, t, plan_list, color_start=0):
    subfigure_mode = True
    if subfigure_mode:
        fig, axs = plt.subplots(2, tight_layout=True)
    else:
        fig = plt.figure(tight_layout=True)
        ax = plt.gca()
        axs = [ax]

    experiment_data = data[0]
    add_horizon_length(data, experiment_data)

    beta = 0.25
    r_obs = 0.5
    r = 0.325 + r_obs

    for k in range(experiment_data["horizon length"]):

        for index, plan in enumerate(plan_list):
            if subfigure_mode:
                ax = axs[index]

            if k == 0 and index == 0:
                plan_positions = np.zeros((2, 2, experiment_data["horizon length"]))

            pos = experiment_data[plan + str(k)][t, :]
            obs = experiment_data["disc_0_pose"][t + k*4, :] # one obstacle
            plan_positions[index, :, k] = pos
            color_idx = (index + color_start) * 2

            # Plot a polygon
            coords, points_on_line = get_points_in_polygon(pos, obs, beta, r_obs, r)
            if len(coords) == 0:
                continue
            p = Polygon(coords, facecolor='C' + str(color_idx), alpha=0.1, zorder=0)
            ax.add_patch(p)

            # Plot the constraint
            ax.plot([points_on_line[0][1], points_on_line[1][1]], [points_on_line[0][0], points_on_line[1][0]],
                     color='C' + str(color_idx), linewidth=1.0, alpha=0.6, linestyle='--', label='_nolegend_')

    for index, plan in enumerate(plan_list):
        if subfigure_mode:
            ax = axs[index]
        color_idx = (index + color_start) * 2

        ax.plot(plan_positions[index, 0, :], plan_positions[index, 1, :], color='C' + str(color_idx),
                 linewidth=3.0, alpha=1.0, zorder=0.1)
        ax.scatter(plan_positions[index, 0, :], plan_positions[index, 1, :], color='C' + str(color_idx),
                    marker='.', s=200, alpha=1.0, label='_nolegend_', zorder=0.1)


    for i, ax in enumerate(axs):
        for k in range(experiment_data["horizon length"]):
            ax.plot(experiment_data["disc_0_pose"][t + k*4, 0], experiment_data["disc_0_pose"][t + k*4, 1],
                 marker='.', markersize=15, linewidth=3.0, alpha=1-k/experiment_data["horizon length"],
                 color='k')
        ax.annotate("Obstacle",
                    xy=(experiment_data["disc_0_pose"][t, 0]+0.2, experiment_data["disc_0_pose"][t, 1]-0.025),
                    horizontalalignment='left',
                    verticalalignment='center',
                    fontsize=16
                    )
        ax.set_aspect('equal', adjustable='box')
        if i == len(axs) - 1:
            ax.set_xlabel('X (m)')
        else:
            ax.set_xticks([])
        ax.set_ylabel('Y (m)')

    return fig


def runtime_boxplot(data, save_path):
    # Plot
    fig_comp_time_box, ax_comp_time_box = plt.subplots()
    fig_comp_time_box.tight_layout(rect=[0, 0.03, 1, 0.95])
    runtime_data = [float(i) for i in data["runtime_control_loop"] if i != -1.]
    ax_comp_time_box.boxplot(runtime_data, showfliers=True, whis=[5, 95])

    # Set & save
    fig_comp_time_box.suptitle('Computation times boxplot')
    ax_comp_time_box.set_title('')
    ax_comp_time_box.set_xlabel('Number of Obstacles')
    ax_comp_time_box.set_ylabel('Time [ms]')
    ax_comp_time_box.grid(color='lightgray', linestyle='-', linewidth=0.1)
    fig_comp_time_box.savefig(f'{save_path}/runtime_boxplot.svg', bbox_inches='tight', format="svg")

def runtime_visual(experiment_data, metrics, save_path):
    runtime_data = merge_experiment_data(experiment_data, "runtime_control_loop")
    df = pd.DataFrame(runtime_data)
    sns.set_style("whitegrid")

    sns.boxplot(y='runtime (ms)', data=df)


def trajectory_visual(experiment_data, save_path, color_index=0):

    min_x = 1e3
    max_x = -1e3
    min_y = 1e3
    max_y = -1e3

    for experiment in experiment_data:
        max_x = max(max_x, np.max(experiment["vehicle_pose"][:, 0]))
        min_x = min(min_x, np.min(experiment["vehicle_pose"][:, 0]))
        max_y = max(max_y, np.max(experiment["vehicle_pose"][:, 1]))
        min_y = min(min_y, np.min(experiment["vehicle_pose"][:, 1]))

        fig = plt.scatter(experiment["vehicle_pose"][:, 0], experiment["vehicle_pose"][:, 1],
                    c=color_index*np.ones((len(experiment["vehicle_pose"]), )),
                    vmin=0,
                    vmax=5,
                    alpha=0.15,
                    cmap='viridis'
                    )

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    min_x = 0
    max_x = 30
    min_y = -3
    max_y = 3

    plt.xlim(min_x, max_x)
    plt.ylim(min_y, max_y)
    # print(f"{min_x}, {max_x} | {min_y}, {max_y}")
    return fig


def visualize_simulation_results(simulation):
    metric_folder = os.getcwd() + "/metrics/" + simulation

    print(bcolors.OKGREEN + "Loading Planners..." + bcolors.ENDC)
    data = []
    metrics = []
    save_paths = []
    for i, filename in enumerate(os.scandir(metric_folder)):
        if filename.is_file():
            if filename.name.endswith(".json"):
                print_value("Planner", str(filename), tab=True)
                planner_name = filename.name.split(".")[0]

                experiment_data, metrics_single, save_path = load_simulation_and_planner(simulation, planner_name)
                save_path += simulation
                os.makedirs(save_path, exist_ok=True)

                data.append(experiment_data)
                metrics.append(metrics_single)
                save_paths.append(save_path)

        if i == 0:
            break

    # Plots per planner
    runtimes = []
    method_names = []
    for i in range(len(data)):
        runtimes.append(merge_experiment_data(data[i], "runtime_control_loop"))

        method_names.append(metrics[i]["name"])
        # metrics[i]["name"]

    plot_plan(experiment_data, ["solver99_plan", "solver0_plan", "solver1_plan", "vehicle_plan_"])
    # plot_plan(experiment_data, ["solver0_plan", "solver1_plan", "solver99_plan"])
    # plot_plan(experiment_data, ["solver0_plan", "solver1_plan", "solver2_plan", "solver3_plan", "solver99_plan"])

    # compare_signals(data, ["best_planner_idx"], save_path, "planner_idx")
    # compare_signals(data, ["objective_1", "objective_2", "objective_3", "objective_99", "objective"], save_path, "objectives")
    # compare_signals(data, ["objective_1", "objective_99", "objective"], save_path, "objectives")

    # df = pd.DataFrame({'runtimes': runtimes, 'method_names': method_names})

    sns.set_style("whitegrid")
    # sns.boxplot(x="method_names", y="runtimes", data=df)#, data=runtimes)

    # fig, axs = plt.subplots(len(data), 1)
    # for i in range(len(data)):
    #     plt.sca(axs[i])
    #     trajectory_fig = trajectory_visual(data[i], save_paths[i], i)
    #     plt.title(metrics[i]["name"])
    #
    # if trajectory_fig is not None:
    #     plt.savefig(f'{save_paths[0]}/trajectory_comparison.png', bbox_inches='tight', format="png")


def load_simulation_and_planner(simulation, planner_name, **kwargs):

    data_folder, metric_folder, figure_folder = get_folder_names(PROJECT_FOLDER)

    # metric_folder = os.getcwd() + "/metrics/" + simulation
    metric_path = metric_folder + simulation + "/" + planner_name + ".json"

    filename = simulation + "_" + planner_name
    data_path = data_folder + filename + ".txt"
    data = load_ros_data(data_path, verbose=False)
    experiment_data = split_ros_data(data, **kwargs)

    # print("Loading JSON: {}".format(str(metric_path)))
    with open(metric_path) as f:
        metrics = dict(json.load(f))

    # save_path = os.getcwd() + "/figures/" + simulation + "/"
    os.makedirs(figure_folder, exist_ok=True)

    return experiment_data, metrics, figure_folder


def main(simulation, planner_name):

    visualize_simulation_results(simulation)

    # runtime_boxplot(data, save_path)
    # if COMPARE_VISUAL:
    #     compare_trajectory_visual(simulation)
    # else:
    #     trajectory_visual(experiment_data, save_path)


    plt.show()

if __name__ == '__main__':
    simulation = sys.argv[1]
    planner_name = sys.argv[2]
    main(simulation, planner_name)