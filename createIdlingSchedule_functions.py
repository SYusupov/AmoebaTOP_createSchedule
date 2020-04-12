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
            if req_idx == 1 and (deliveryReq[vehicle][0] != deliveryReq[vehicle][1]): 
                # if the deliveryReqs are the same cells, the additional one is needed:
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

def addTo_idlingForSolving(vehicle, idlingForSolving, idlingStart_idx, idlingTime, schToChange, curSch_lens):
    """adding the idling made for solving the collisions for later possible use (when the vehicle collides at the time step of the idling)
    the function also adds any effects of the current idling to previous made idlings
    used in solve_collisions()"""
    if idlingForSolving[vehicle]:
        idling_len = len(idlingForSolving[vehicle])
        idling_record_count = 0 # to avoid multiple recordings for each idling in else below
        print(idling_len)
        for idx, idling in enumerate(idlingForSolving[vehicle][:idling_len]):
            if idlingStart_idx in range(idling[0],idling[1]+1): # if it is in the range
                print("ran the range vehicle {}".format(vehicle))
                # append the current idling to the previous one as the same number
                idlingForSolving[vehicle][idx][1] += idlingTime
                print(idlingForSolving[vehicle], "if")
            else:
                if idling_record_count == 0:
                    # if the idling was not recorded in a previous else statement
                    print(idlingStart_idx, idlingStart_idx+idlingTime, vehicle)
                    idlingForSolving[vehicle].append([idlingStart_idx, idlingStart_idx+idlingTime])
                    # adding info about what stage this idling to later be able to edit curSch_lens if a time step in range of this idling collides
                    if schToChange == 1:
                        idlingForSolving[vehicle][-1].extend([1])
                    elif schToChange == 2:
                        idlingForSolving[vehicle][-1].extend([2])
                    else:
                        print("schToChange is not defined for vehicle {}".format(vehicle))
                    idling_record_count += 1
                # in case the idling affects the previous idling
                if idling[0] > idlingStart_idx:
                    idling[0] += idlingTime
                    idling[1] += idlingTime
                print(idlingForSolving[vehicle], "else")
    else:
        # if it is a new idling for the vehicle no problem in affecting previous idlings
        idlingForSolving[vehicle].append([idlingStart_idx, idlingStart_idx+idlingTime])
        if schToChange == 1:
            idlingForSolving[vehicle][-1].extend([1])
        elif schToChange == 2:
            idlingForSolving[vehicle][-1].extend([2])
        else:
            print("schToChange is not defined for vehicle {}".format(vehicle))
    if schToChange == 1:
        curSch_lens[vehicle][0] += idlingTime
    elif schToChange == 2:
        curSch_lens[vehicle][1] += idlingTime
    else:
        print("schToChange is not defined for vehicle {}".format(vehicle))
    return idlingForSolving, curSch_lens

def solve_collisions(collisions, vehicle_combs, vehicleSchedules, curSch_lens, idleTime, idlingForSolving):
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
            # check how improtant cell indeces
            first_stage_idx1 = curSch_lens[vehicle_combs[col_idx][0]][0]-1 # when we reached for-cell
            first_break_idx1 = first_stage_idx1 + idleTime[vehicle_combs[col_idx][0]][0] # when we finished idling in for-cell
            second_stage_idx1 = first_break_idx1 + curSch_lens[vehicle_combs[col_idx][0]][1]-1 # when we reached to-cell
            second_break_idx1 = second_stage_idx1 + idleTime[vehicle_combs[col_idx][0]][1] # when we finished idling in to-cell
            first_stage_idx2 = curSch_lens[vehicle_combs[col_idx][1]][0]-1
            first_break_idx2 = first_stage_idx2 + idleTime[vehicle_combs[col_idx][1]][0]
            second_stage_idx2 = first_break_idx2 + curSch_lens[vehicle_combs[col_idx][1]][1]-1
            second_break_idx2 = second_stage_idx2 + idleTime[vehicle_combs[col_idx][1]][1]
            if_idling = [False, False]
            idlingLefts = defaultdict(int)
            idlingStarts = defaultdict(int)
            schToChange = defaultdict(int) # to change the count in curSch_lens appropriately
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
                # recording how much time step is left for each of the vehicles to idle
                if collision in range(first_stage_idx1, first_break_idx1):
                    idlingLefts["first"] = first_break_idx1 - collision
                    idlingStarts["first"] = first_stage_idx1
                    schToChange["first"] = 1
                elif collision in range(second_stage_idx1, second_break_idx1):
                    idlingLefts["first"] = second_break_idx1 - collision
                    idlingStarts["first"] = second_stage_idx1
                    schToChange["first"] = 2
                else:
                    if idlingForSolving[vehicle_combs[col_idx][0]]:
                        for idling in idlingForSolving[vehicle_combs[col_idx][0]]:
                            if collision in range(idling[0],idling[1]+1):
                                idlingLefts["first"] = idling[1]+1 - collision
                                idlingStarts["first"] = idling[0]
                                schToChange["first"] = idling[2]
                                break
                    else:
                        print("vehicle {} is not idling but marked as idling".format(vehicle_combs[col_idx][0]))
                if collision in range(first_stage_idx2, first_break_idx2):
                    idlingLefts["second"] = first_break_idx2 - collision
                    idlingStarts["second"] = first_stage_idx2
                    schToChange["second"] = 1
                elif collision in range(second_stage_idx2, second_break_idx2):
                    idlingLefts["second"] = second_break_idx2 - collision
                    idlingStarts["second"] = second_stage_idx2
                    schToChange["second"] = 2
                else:
                    if idlingForSolving[vehicle_combs[col_idx][1]]:
                        for idling in idlingForSolving[vehicle_combs[col_idx][1]]:
                            if collision in range(idling[0],idling[1]+1):
                                idlingLefts["second"] = idling[1]+1 - collision
                                idlingStarts["second"] = idling[0]
                                schToChange["second"] = idling[2]
                                break
                    else:
                        print("vehicle {} is not idling but marked as idling".format(vehicle_combs[col_idx][0]))
                # making wait for the vehicle which started idling earlier
                if idlingStarts["first"] > idlingStarts["second"]:
                    # if the first of the 2 vehicles started idling later it has to wait to avoid collision
                    vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                    vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:idlingStarts["first"]]+[vehicle1[idlingStarts["first"]-1]]*idlingLefts["second"]+vehicle1[idlingStarts["first"]:]
                    dealtVehicles.append(vehicle_combs[col_idx][0])
                    # editting previous recorded idlings in idlingForSolving if this one affects them
                    idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][0], idlingForSolving, idlingStarts["first"]-1, idlingLefts["second"], schToChange["first"], curSch_lens)
                elif idlingStarts["second"] > idlingStarts["first"]:
                    # if the second of the 2 vehicles started idling later it has to wait to avoid collision
                    vehicle2 = vehicleSchedules[vehicle_combs[col_idx][1]]
                    vehicleSchedules[vehicle_combs[col_idx][1]] = vehicle2[:idlingStarts["second"]]+[vehicle2[idlingStarts["second"]-1]]*idlingLefts["first"]+vehicle2[idlingStarts["second"]:]
                    dealtVehicles.append(vehicle_combs[col_idx][1])
                    # adding the idling added to later use if needed for finding idlingLefts
                    idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][1], idlingForSolving, idlingStarts["second"]-1, idlingLefts["first"], schToChange["second"], curSch_lens)
                else:
                    vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                    vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:idlingStarts["first"]]+[vehicle1[idlingStarts["first"]-1]]*idlingLefts["second"]+vehicle1[idlingStarts["first"]:]
                    dealtVehicles.append(vehicle_combs[col_idx][0])
                    # adding the idling added to later use if needed for finding idlingLefts
                    idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][0], idlingForSolving, idlingStarts["first"]-1, idlingLefts["second"], schToChange["first"], curSch_lens)
            elif if_idling[0] == True and if_idling[1] == False: # if the 1st of the vehicles is idling
                # find out when the idling vehicle stops idling
                if collision in range(first_stage_idx1, first_break_idx1):
                    idlingLeft = first_break_idx1 - collision 
                elif collision in range(second_stage_idx1, second_break_idx1):
                    idlingLeft = second_break_idx1 - collision
                else:
                    if idlingForSolving[vehicle_combs[col_idx][0]]:
                        for idling in idlingForSolving[vehicle_combs[col_idx][0]]:
                            if collision in range(idling[0],idling[1]+1):
                                idlingLeft = idling[1]+1 - collision
                                idlingStarts["first"] = idling[0]
                                break
                    else:
                        print("vehicle {} is not idling but marked as idling".format(vehicle_combs[col_idx][0]))
                if collision in range(1, first_stage_idx2+1):
                    schToChange = 1
                elif collision in range(first_break_idx2, second_stage_idx2):
                    schToChange = 2
                else:
                    print("Collision for vehicle {} is in either cell 1 or while it is idling but it is marked as not idling".format(vehicle_combs[col_idx][1]))
                # make the other vehicle wait
                vehicle2 = vehicleSchedules[vehicle_combs[col_idx][1]]
                vehicleSchedules[vehicle_combs[col_idx][1]] = vehicle2[:collision]+[vehicle2[collision-1]]*idlingLeft+vehicle2[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][1])
                idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][1], idlingForSolving, collision-1, idlingLeft, schToChange, curSch_lens)
            elif if_idling[0] == False and if_idling[1] == True: # if the 2nd of the vehicles is idling
                # find out when the idling vehicle stops idling
                if collision in range(first_stage_idx2, first_break_idx2):
                    idlingLeft = first_break_idx2 - collision 
                elif collision in range(second_stage_idx2, second_break_idx2):
                    idlingLeft = second_break_idx2 - collision
                else:
                    if idlingForSolving[vehicle_combs[col_idx][1]]:
                        for idling in idlingForSolving[vehicle_combs[col_idx][1]]:
                            if collision in range(idling[0],idling[1]+1):
                                idlingLeft = idling[1]+1 - collision
                                idlingStarts["second"] = idling[0]
                                break
                    else:
                        print("vehicle {} is not idling but marked as idling".format(vehicle_combs[col_idx][0]))
                if collision in range(1, first_stage_idx1+1):
                    schToChange = 1
                elif collision in range(first_break_idx1, second_stage_idx1):
                    schToChange = 2
                else:
                    print("Collision for vehicle {} is in either cell 1 or while it is idling but it is marked as not idling".format(vehicle_combs[col_idx][0]))
                # make the other vehicle wait
                vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:collision]+[vehicle1[collision-1]]*idlingLeft+vehicle1[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][0])
                idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][0], idlingForSolving, collision-1, idlingLeft, schToChange, curSch_lens)
            else: # if neither of the vehicles is idling
                vehicle1 = vehicleSchedules[vehicle_combs[col_idx][0]]
                vehicleSchedules[vehicle_combs[col_idx][0]] = vehicle1[:collision]+[vehicle1[collision-1]]+vehicle1[collision:]
                dealtVehicles.append(vehicle_combs[col_idx][0])
                # recording changes in schedule to curSch_lens
                if collision in range(1, first_stage_idx1+1):
                    schToChange = 1
                elif collision in range(first_break_idx1, second_stage_idx1):
                    schToChange = 2
                else:
                    print("Collision for vehicle {} is in either cell 1 or while it is idling but it is marked as not idling".format(vehicle_combs[col_idx][0]))
                idlingForSolving, curSch_lens = addTo_idlingForSolving(vehicle_combs[col_idx][0], idlingForSolving, collision-1, 1, schToChange, curSch_lens)
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
