### Libraries
import re # for finding schedule states and graph rules
import ast # for turning string into python objects (i.e. lists)
from collections import defaultdict # for storing schedule states and graph rules
import itertools # for grouping cells a vehicle travelled to identify ones it idled at
import sys # for getting filename from the user

def get_schedule(f):
    """search for the schedule of the vehicles in the input file and output it
    keys represent the number of the vehicle
    values are dictionaries with keys as time units p and values as cell numbers"""
    states = [(s.start(), s.end()) for s in re.finditer('u\[([0-9]|,|\s)+\]', f)]
    statesD = defaultdict(dict)
    for state in states:
        state_text = f[state[0]+1:state[1]]
        state_list = ast.literal_eval(state_text)
        statesD[state_list[0]][state_list[2]] = state_list[1]
    return statesD

def get_initPoses(f):
    """search for the initial positions of the vehicles and output them
    keys represent the number of the vehicle
    values represent the number of the cell"""
    initPoses_idx = [(s.start(), s.end()) for s in re.finditer('yi\[([0-9]|,|\s)+\]', f)]
    initPoses = defaultdict(int)
    for idxs in initPoses_idx:
        idxs_text = f[idxs[0]+2:idxs[1]]
        idxs_list = ast.literal_eval(idxs_text)
        initPoses[idxs_list[0]] = idxs_list[1]
    return initPoses

def get_Req(f):
    """search for the delivery requests to vehicles and output them
    keys represent the number of the vehicle
    values represent a list of the cell to load at and the cell to unload at"""
    load_cells_idxs = [(s.start(), s.end()) for s in re.finditer('yf\[([0-9]|,|\s)+\]', f)]
    load_cells_dict = defaultdict(list)
    for idxs in load_cells_idxs:
        load_cell_text = f[idxs[0]+2:idxs[1]]
        load_cell_list = ast.literal_eval(load_cell_text)
        load_cells_dict[load_cell_list[0]].append([load_cell_list[1],load_cell_list[2]])
    
    unload_cells_idxs = [(s.start(), s.end()) for s in re.finditer('yt\[([0-9]|,|\s)+\]', f)]
    unload_cells_dict = defaultdict(list)
    for idxs in unload_cells_idxs:
        unload_cell_text = f[idxs[0]+2:idxs[1]]
        unload_cell_list = ast.literal_eval(unload_cell_text)
        unload_cells_dict[unload_cell_list[0]].append([unload_cell_list[1],unload_cell_list[2]])
    
    deliveryReq = defaultdict(list)
    idleTime = defaultdict(list)
    for key in load_cells_dict.keys():
        deliveryReq[key] = [load_cells_dict[key][0][0],unload_cells_dict[key][0][0]]
        idleTime[key] = [len(load_cells_dict[key]),len(unload_cells_dict[key])]
    return deliveryReq, idleTime

def get_directions(f):
    """search for the edges of the graph of movement from one cell to another and output them
    keys represent cells
    values are lists of cells where the cell can move"""
    directionsText_idxs = [(s.start(), s.end()) for s in re.finditer('E\s=\s({|}|,|\s|[0-9])+', f)]
    directionsText = f[directionsText_idxs[0][0]:directionsText_idxs[0][1]]
    rules = ["["+directionsText[s.start()+1: s.end()-1]+"]" for s in re.finditer('\{([0-9]|,|\s)+\}', directionsText)]
    directionsRuleset = defaultdict(list)
    for rule in rules:
        rule_list = ast.literal_eval(rule)
        directionsRuleset[rule_list[0]].append(rule_list[1])
    return directionsRuleset

def find_max_p(statesD):
    """find the maximum p for measuring optimality and later calculations"""
    max_p = 0
    for vehicle in statesD:
        for p in statesD[vehicle]:
            if p > max_p:
                max_p = p
    max_p += 1
    return max_p

def check_transitions_legality(statesD, directionsRuleset, max_p):
    """legality 1: checking if transition to another cell is legal"""
    count_all_transitions = 0
    count_edge_transitions = 0
    for vehicle in statesD:
        for i in range(1,max_p):
            count_all_transitions += 1
            # checking if the current cell is appropriate based on the previous
            if i in statesD[vehicle]: # because some vehicles might finish before others
                if (statesD[vehicle][i] in directionsRuleset[statesD[vehicle][i-1]]) or (statesD[vehicle][i-1] == statesD[vehicle][i]):
                    count_edge_transitions += 1
                else:
                    print("vehicle: {}, time: {}, cell: {}".format(vehicle, i, statesD[vehicle][i]))
    return count_all_transitions, count_edge_transitions

def check_exclusiveness(statesD, max_p):
    """legality 2: checking if no vehicles are at the same position at the same time"""
    count_vehicle_transitions = 0
    count_exclusive_transitions = 0
    for i in range(1,max_p):
        count_vehicle_transitions += 1
        if i in statesD[1] and i in statesD[2] and i in statesD[3]:
            if (statesD[1][i] != statesD[2][i]) and (statesD[1][i] != statesD[3][i]) and (statesD[2][i] != statesD[3][i]):
                count_exclusive_transitions += 1
        elif i in statesD[1] and i in statesD[2]:
            if (statesD[1][i] != statesD[2][i]):
                count_exclusive_transitions += 1
        elif i in statesD[1] and i in statesD[3]:
            if (statesD[1][i] != statesD[3][i]):
                count_exclusive_transitions += 1
        elif i in statesD[2] and i in statesD[3]:
            if (statesD[2][i] != statesD[3][i]):
                count_exclusive_transitions += 1
    return count_vehicle_transitions, count_exclusive_transitions

def check_requestsatisfaction(statesD, deliveryReq, idleTime, max_p, initPoses):
    """request-satisfiability 1: followed the delivery instructions 
    idling time, loading, unloading cells, loading cell before unloading cell"""
    count_requestchecks = 0
    count_requestsatisfactions = 0
    max_ps = []
    for vehicle in statesD:
        if vehicle in deliveryReq: # if the vehicle has delivery requests
            count_requestchecks += 3 # number of checks to be made later in the code
            for_time = idleTime[vehicle][0] # amount of time the vehicle should stay at loading cell
            to_time = idleTime[vehicle][1]  # amount of time the vehicle should stay at unloading cell
            for_cell = deliveryReq[vehicle][0] # number of the loading cell
            to_cell = deliveryReq[vehicle][1] # number of the unloading cell
            for_time_now = 0 # amount of time the vehicle stays at loading cell
            to_time_now = 0 # amount of time the vehicle stays at unloading cell
            for_end = 0 # time at which left the loading cell
            to_end = 0 # time at which left the unloading cell
            to_time_idx = 0
            for_time_idx = 0
            after_finished = 0 # how much time passed after satisfication of the delivery requirements
            positions = statesD[vehicle].values()
            consecutive_cells = [(i, len(list(c))) for i, c in itertools.groupby(positions)]
            for_time_now = max([count[1] for count in consecutive_cells if count[0]==for_cell], default=0)
            to_time_now = max([count[1] for count in consecutive_cells if count[0]==to_cell], default=0)
            # checking that idling is appropriate
            if for_time_now >= for_time:
                count_requestsatisfactions += 1
            else:
                print("for not enough for vehicle {}".format(vehicle))
            if to_time_now >= to_time:
                count_requestsatisfactions += 1
            else:
                print("to not enough for vehicle {}".format(vehicle))
            # checking that the vehicle loads before unloading
            for idx, cell in enumerate(consecutive_cells):
                if cell[0] == for_cell and cell[1] == for_time_now:
                    for_time_idx = idx
                elif cell[0] == to_cell and cell[1] == to_time_now:
                    to_time_idx = idx
            if to_time_idx > for_time_idx:
                count_requestsatisfactions += 1
            else:
                print("for before to for vehicle {}".format(vehicle))
            if for_time_now >= for_time and to_time_now >= to_time: # if the delivery instructions were satisfied
                for cell in consecutive_cells[to_time_idx+1:]:
                    after_finished += cell[1]
                after_finished+=to_time_now - to_time
                max_ps.append(len(statesD[vehicle])-after_finished)
            elif for_time_now >= for_time: # if the vehicle only unloaded
                max_ps.append(len(statesD[vehicle])+to_time)
            elif to_time_now >= to_time: # if the vehicle only loaded
                max_ps.append(len(statesD[vehicle])+for_time)
            else: # if the vehicle satisfied neither of the requirements
                max_ps.append(len(statesD[vehicle])+for_time+to_time)
        else:
            max_ps.append(len(statesD[vehicle]))
    return count_requestchecks, count_requestsatisfactions, max_ps
