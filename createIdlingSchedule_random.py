from createIdlingSchedule_functions import *
import csv 
import random

def main():
    filename = sys.argv[1]
    num_vehicles = int(sys.argv[2])
    num_cells = int(sys.argv[3])
    idle_max = int(sys.argv[4])

    f = open(filename,'r').read()

    directionsRuleset = get_directions(f)
    initPoses = defaultdict(int)
    deliveryReq = defaultdict(list)
    idleTime = defaultdict(list)
    initPoses = random.sample(range(1, num_cells+1), num_vehicles)
    for vehicle in range(1, num_vehicles+1):
        deliveryReq[vehicle] = [random.randrange(1, num_cells+1) for i in range(2)]
        idleTime[vehicle] = [random.randrange(1, idle_max) for i in range(2)]

    vehicleSchedules, curSch_lens = get_initial_schedules(deliveryReq, initPoses, idleTime, directionsRuleset)

    collisions, vehicle_combs = get_collision(vehicleSchedules)

    vehicleSchedules = solve_collisions(collisions, vehicle_combs, vehicleSchedules, curSch_lens, idleTime)
    
    alternative_paths = defaultdict(dict) # for storing alternative roads to use in case we are in a "dead end"
    for vehicle in vehicleSchedules:
        alternative_paths[vehicle]={} # for each time step p
    vehicleSchedules, max_p = traverse_others(alternative_paths, vehicleSchedules, directionsRuleset, initPoses, deliveryReq)
    
    statesD = defaultdict(dict)
    for vehicle in vehicleSchedules:
        for p in range(len(vehicleSchedules[vehicle])):
            statesD[vehicle][p] = vehicleSchedules[vehicle][p]

    count_all_transitions, count_edge_transitions = check_transitions_legality(statesD, directionsRuleset, max_p)

    count_vehicle_transitions, count_exclusive_transitions = check_exclusiveness(statesD, max_p)

    count_requestchecks, count_requestsatisfactions, max_ps = check_requestsatisfaction(statesD, deliveryReq, idleTime, max_p, initPoses)
    # check that everything is legal and request satisfiable
    if (count_all_transitions != count_edge_transitions):
        print("Not all transitions are physically possible")
    if (count_vehicle_transitions != count_exclusive_transitions):
        print("Not all transitions are exclusive")
    if (count_requestchecks != count_requestsatisfactions):
        print("Not all requests are satisfied")

    # recording the ps to a file
    f = open("perf_IdlingAlgorithm.csv", "a")
    with f:
        writer = csv.writer(f)
        writer.writerow(max_ps)

if __name__ == "__main__":
    main()