from ratingSchedule_functions import *
from itertools import combinations

def bfs_paths(o, d, directionsRuleset):
    """to get the shortest path from the origin to the destination
    TO DO: modify to output a list of all possible paths (or at least a certain number)"""
    explored = []
    queue = [[o]]
    if o == d:
        return [o]
    
    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node not in explored:
            neighbors = directionsRuleset[node]
            
            for neighbour in neighbors:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                if neighbour == d:
                    return new_path
                     
            explored.append(node)

def get_initial_schedules(deliveryReq, initPoses, idleTime, directionsRuleset):
    """to get schedules without consideration of other vehicles, i.e. ignore collisions"""
    vehicleSchedules = defaultdict(list)
    curSch_lens = defaultdict(list) # storing number of steps for each destination
    for vehicle in deliveryReq:
        # dictionary --> list for each vehicle --> schedule as a list
        vehicleSchedules[vehicle] = []
        # dictionary --> list for each vehicle --> list for each request --> schedule as a list
        curSch_lens[vehicle] = []
    # storing info on origin and destination cells
    org_dest = defaultdict(list)
    for vehicle in deliveryReq:
        org_dest[vehicle] = [(initPoses[vehicle],deliveryReq[vehicle][0])]
        org_dest[vehicle].append((deliveryReq[vehicle][0],deliveryReq[vehicle][1]))

    ## getting the schedules for each vehicle without consideration of other vehicles
    for vehicle in deliveryReq:
        for req_idx, req in enumerate(org_dest[vehicle]):
            path = bfs_paths(req[0], req[1], directionsRuleset)
            # remove the unnecessary step (e.g. 6 idles instead of 5)
            if req_idx == 1:
                path = path[1:]

            vehicleSchedules[vehicle].extend(path)
            curSch_lens[vehicle].append(len(path))

            # adding idling Time for unloading cells
            idlingSteps = [req[1]]*(idleTime[vehicle][req_idx]-1)
            vehicleSchedules[vehicle].extend(idlingSteps)
    
    return vehicleSchedules, curSch_lens

def get_collision(vehicleSchedules):
    # finding collisions for vehicles with delivery requests
    vehicle_combs = list(combinations([k for k in vehicleSchedules], 2))
    collisions = []
    for comb in vehicle_combs:
        collisions.append(None)
        min_len = min(len(vehicleSchedules[comb[0]]), len(vehicleSchedules[comb[1]]))
        for step in range(1, min_len):
            if vehicleSchedules[comb[0]][step] == vehicleSchedules[comb[1]][step]:
                collisions[-1] = step
                break
    return collisions, vehicle_combs

def solve_collisions(collisions, vehicle_combs, vehicleSchedules, curSch_lens, idleTime):
    """prevent collisions by idling a vehicle that is not idling"""
    dealtVehicles = []
    for col_idx, collision in enumerate(collisions):
        if collision != None and all(vehicle not in dealtVehicles for vehicle in vehicle_combs[col_idx]):
            ## TODO: if there is a another possibleSchedule
            ## if we only can idle the vehicle that is not idling
            # check if paths of vehicles were dealt with before
            # if yes one of them might have already transformed
            # and thus we would not solve the collision correctly
            # check if either of the vehicles is idling
            if_idling = [False, False]
            try:
                same_as_next1 = vehicleSchedules[vehicle_combs[col_idx][0]][collision] == vehicleSchedules[vehicle_combs[col_idx][0]][collision+1]
            except:
                same_as_next1 = False
            try:
                same_as_next2 = vehicleSchedules[vehicle_combs[col_idx][1]][collision] == vehicleSchedules[vehicle_combs[col_idx][1]][collision+1]
            except:
                same_as_next2 = False
            if vehicleSchedules[vehicle_combs[col_idx][0]][collision] == vehicleSchedules[vehicle_combs[col_idx][0]][collision-1] or same_as_next1:
                if_idling[0] =True
            if vehicleSchedules[vehicle_combs[col_idx][1]][collision] == vehicleSchedules[vehicle_combs[col_idx][1]][collision-1] or same_as_next2:
                if_idling[1] = True
            if if_idling[0] == True and if_idling[1] == True:
                # check how much is left for both of them to idle
                first_stage_idx1 = curSch_lens[vehicle_combs[col_idx][0]][0]-1
                first_break_idx1 = first_stage_idx1 + idleTime[vehicle_combs[col_idx][0]][0]
                second_stage_idx1 = first_break_idx1 + curSch_lens[vehicle_combs[col_idx][0]][1]-1
                second_break_idx1 = second_stage_idx1 + idleTime[vehicle_combs[col_idx][0]][1]
                first_stage_idx2 = curSch_lens[vehicle_combs[col_idx][1]][0]-1
                first_break_idx2 = first_stage_idx2 + idleTime[vehicle_combs[col_idx][1]][0]
                second_stage_idx2 = first_break_idx2 + curSch_lens[vehicle_combs[col_idx][1]][1]-1
                second_break_idx2 = second_stage_idx2 + idleTime[vehicle_combs[col_idx][1]][1]
                # find out which delivery requirement the vehicles were doing
                idlingLefts = defaultdict(int)
                idlingStarts = defaultdict(int)
                schToChange = defaultdict(int) # to change the count in curSch_lens appropriately
                # recording how much time step is left for each of the vehicles to idle
                if collision in range(first_stage_idx1, first_break_idx1):
                    idlingLefts["first"] = first_break_idx1 - collision
                    idlingStarts["first"] = first_stage_idx1
                    schToChange["first"] = 1
                elif collision in range(second_stage_idx1, second_break_idx1):
                    idlingLefts["first"] = second_break_idx1 - collision
                    idlingStarts["first"] = second_stage_idx1
                    schToChange["first"] = 2
                if collision in range(first_stage_idx2, first_break_idx2):
                    idlingLefts["second"] = first_break_idx2 - collision
                    idlingStarts["second"] = first_stage_idx2
                    schToChange["second"] = 1
                elif collision in range(second_stage_idx2, second_break_idx2):
                    idlingLefts["second"] = second_break_idx2 - collision
                    idlingStarts["second"] = second_stage_idx2
                    schToChange["second"] = 2
                # making wait for the vehicle which started idling earlier
                if idlingStarts["first"] > idlingStarts["second"]:
                    # if the first of the 2 vehicles started idling later it has to wait to avoid collision
                    vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                    vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:idlingStarts["first"]]+[vehicle1[idlingStarts["first"]-1]]*idlingLefts["second"]+vehicle1[idlingStarts["first"]:]
                    dealtVehicles.append(vehicle_combs[col_idx][0])
                    if schToChange["first"] == 1:
                        curSch_lens[vehicl_combs[col_idx][0]][0] += idlingLefts["second"]
                    elif schToChange["first"] == 2:
                        curSch_lens[vehicl_combs[col_idx][0]][1] += idlingLefts["second"]
                elif idlingStarts["second"] > idlingStarts["first"]:
                    # if the second of the 2 vehicles started idling later it has to wait to avoid collision
                    vehicle2 = vehicleSchedules[vehicle_combs[col_idx][1]]
                    print(idlingLefts["first"])
                    vehicleSchedules[vehicle_combs[col_idx][1]] = vehicle2[:idlingStarts["second"]]+[vehicle2[idlingStarts["second"]-1]]*idlingLefts["first"]+vehicle2[idlingStarts["second"]:]
                    dealtVehicles.append(vehicle_combs[col_idx][1])
                    if schToChange["second"] == 1:
                        curSch_lens[vehicl_combs[col_idx][1]][0] += idlingLefts["first"]
                    elif schToChange["second"] == 2:
                        curSch_lens[vehicl_combs[col_idx][1]][1] += idlingLefts["first"]
                else:
                    print("Both vehicles that started idling at the same time collided (which I think is impossible to have as non-idling collisions would first be outputed and solved")
            elif if_idling[0] == True and if_idling[1] == False: # if the 1st of the vehicles is idling
                # find out when the idling vehicle stops idling
                first_stage_idx = curSch_lens[vehicle_combs[col_idx][0]][0]-1
                first_break_idx = first_stage_idx + idleTime[vehicle_combs[col_idx][0]][0]
                second_stage_idx = first_break_idx + curSch_lens[vehicle_combs[col_idx][0]][1]-1
                second_break_idx = second_stage_idx + idleTime[vehicle_combs[col_idx][0]][1]
                if collision in range(first_stage_idx, first_break_idx):
                    idlingLeft = first_break_idx - collision 
                    schToChange = 1
                elif collision in range(second_stage_idx, second_break_idx):
                    idlingLeft = second_break_idx - collision
                    schToChange = 2
                # make the other vehicle wait
                vehicle2 = vehicleSchedules[vehicle_combs[col_idx][1]]
                vehicleSchedules[vehicle_combs[col_idx][1]] = vehicle2[:collision]+[vehicle2[collision-1]]*idlingLeft+vehicle2[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][1])
                if schToChange == 1:
                    curSch_lens[vehicl_combs[col_idx][1]][0] += idlingLeft
                elif schToChange == 2:
                    curSch_lens[vehicl_combs[col_idx][1]][1] += idlingLeft
            elif if_idling[0] == False and if_idling[1] == True: # if the 2nd of the vehicles is idling
                # find out when the idling vehicle stops idling
                first_stage_idx = curSch_lens[vehicle_combs[col_idx][1]][0]-1
                first_break_idx = first_stage_idx + idleTime[vehicle_combs[col_idx][1]][0]
                second_stage_idx = first_break_idx + curSch_lens[vehicle_combs[col_idx][1]][1]-1
                second_break_idx = second_stage_idx + idleTime[vehicle_combs[col_idx][1]][1]
                if collision in range(first_stage_idx, first_break_idx):
                    idlingLeft = first_break_idx - collision 
                    schToChange = 1
                elif collision in range(second_stage_idx, second_break_idx):
                    idlingLeft = second_break_idx - collision
                    schToChange = 2
                # make the other vehicle wait
                vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:collision]+[vehicle1[collision-1]]*idlingLeft+vehicle1[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][0])
                if schToChange == 1:
                    curSch_lens[vehicl_combs[col_idx][0]][0] += idlingLeft
                elif schToChange == 2:
                    curSch_lens[vehicl_combs[col_idx][0]][1] += idlingLeft
            else: # if neither of the vehicles is idling
                vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:collision]+[vehicle1[collision-1]]+vehicle1[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][0])
                
                # recording changes in schedule to curSch_lens
                first_stage_idx = curSch_lens[vehicle_combs[col_idx][0]][0]-1
                first_break_idx = first_stage_idx + idleTime[vehicle_combs[col_idx][0]][0]
                second_stage_idx = first_break_idx + curSch_lens[vehicle_combs[col_idx][0]][1]-1
                second_break_idx = second_stage_idx + idleTime[vehicle_combs[col_idx][0]][1]
                if collision in range(first_stage_idx, first_break_idx):
                    schToChange = 1
                elif collision in range(second_stage_idx, second_break_idx):
                    schToChange = 2
                else:
                    schToChange = 0
                if schToChange == 1:
                    curSch_lens[vehicl_combs[col_idx][0]][0] += 1
                elif schToChange == 2:
                    curSch_lens[vehicl_combs[col_idx][0]][1] += 1
    return vehicleSchedules, curSch_lens

def traverse_others(alternative_paths, vehicleSchedules, directionsRuleset, initPoses, deliveryReq):
    # getting the maximum p taken by any vehicle
    max_p = max([len(vehicleSchedules[vehicle]) for vehicle in vehicleSchedules])
    # appending additional traversing cells with no collisions
    for vehicle in initPoses:
        if vehicle not in deliveryReq or len(vehicleSchedules[vehicle]) != max_p:
            # if the vehicle just has to traverse around without requrements
            if vehicle not in deliveryReq:
                vehicleSchedules[vehicle] = [initPoses[vehicle]]
                prevCell = initPoses[vehicle]
            
            prevCell = vehicleSchedules[vehicle][-1]
            for p in range(len(vehicleSchedules[vehicle]),max_p):
                possible_cells = [prevCell] + directionsRuleset[prevCell]
                for cell_idx, targetCell in enumerate(possible_cells):
                    isColliding = False
                    for another_v in deliveryReq:
                        if another_v != vehicle and len(vehicleSchedules[another_v])-1 >= p:
                            if vehicleSchedules[another_v][p] == targetCell:
                                isColliding = True # because this cell results in a collision
                                break
                    if isColliding == False:
                        if len(possible_cells[cell_idx+1:]) != 0:
                            alternative_paths[vehicle][p] = possible_cells[cell_idx+1:]
                        break # because the current cell is good enough, we don't have to check for others
                if isColliding == True:
                    print("A path could not be found for vehicle {} in time {}".format(vehicle, p))
                    print("Schedule for vehicle {}: {}".format(vehicle, vehicleSchedules[vehicle]))
                    get_alternative_path(directionsRuleset, initPoses, vehicle, alternative_paths, deliveryReq, vehicleSchedules)
                else:
                    vehicleSchedules[vehicle].append(targetCell)
                    prevCell = targetCell
    return vehicleSchedules, max_p

def get_alternative_path(directionsRuleset, initPoses, vehicle, alternative_paths, deliveryReq, vehicleSchedules):
    # if it's the case when we have go back to alternatives because of dead end
    isColliding = True
    while isColliding == True or not alternative_paths[vehicle]:
        print(alternative_paths[vehicle])
        foundAlternative = False
        #while foundAlternative == False:
        p = sorted(list(alternative_paths[vehicle].keys()))[-1]
        #    if len(alternative_paths[vehicle][p]) == 0:
        #        del alternative_paths[vehicle][p]
        #    else:
        #        foundAlternative = True
        isColliding = False
        for targetCell in alternative_paths[vehicle][p]:
            alternative_paths[vehicle][p].pop(0)
            for another_v in deliveryReq:
                if another_v != vehicle and len(vehicleSchedules[another_v])-1 >= p:
                    if vehicleSchedules[another_v][p] == targetCell:
                        isColliding = True
                        break # because this cell results in a collision
            if isColliding == False:
                vehicleSchedules[vehicle] = vehicleSchedules[vehicle][:p]
                print("p solved: {}".format(p))
                print("alternative schedule {}".format(vehicleSchedules[vehicle]))
                vehicleSchedules[vehicle].append(targetCell)
                traverse_others(alternative_paths, vehicleSchedules, directionsRuleset, initPoses, deliveryReq)
                break # because the current cell is good enough, we don't have to check for others
        del alternative_paths[vehicle][p]
    if not alternative_paths:
        print("Could not find a path for vehicle {} at time unit {}".format(vehicle, p))
