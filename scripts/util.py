import sys, os, shutil

import helpers
from helpers import print_value, PROJECT_FOLDER

backup_simulations = [
    "random-16_straight",
    "random-12_straight",
    "random-8_straight",
    "random-4_straight",
    "corridor-16_random",
    "crossing-16_random-sidewalks",
]

def backup_data(simulation):
    print("Making a backup of simulation data...")
    print_value("Simulation", simulation)

    data_folder, metrics_folder, figure_folder = helpers.get_folder_names()

    backup_folder = helpers.get_backup_folder()
    os.makedirs(backup_folder, exist_ok=True)

    for filename in os.scandir(data_folder):
        if filename.is_file():
            if filename.name.endswith(".txt") and filename.name.startswith(simulation + "_"):
                print_value("Planner", str(filename), tab=True)
                shutil.copyfile(data_folder + filename.name, backup_folder + filename.name)

    print("Backup completed, see: " + backup_folder)


if __name__ == '__main__':
    run_what = sys.argv[1]

    if run_what == "backup":
        for simulation in backup_simulations:
            backup_data(simulation)
