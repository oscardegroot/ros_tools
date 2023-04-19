import helpers
import metrics, compare, specific_visuals

if __name__ == '__main__':
    # simulation = "random-8_straight"
    # simulation = "crossing-16_random-sidewalks"
    # simulation = "corridor-16_random"
    for simulation in helpers.SIMULATIONS_IN_TABLE:

    # simulation = "test"
        planner = "GMPCC"
    # planner = "frenet-planner"

        metrics.main(simulation, planner)

    compare.main()
    # specific_visuals.main(simulation, planner)
