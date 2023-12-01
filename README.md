# AmoebaTOP_createSchedule
Our solution to creating a legal schedule of multiple robots in a factory for **Vehicle Routing Problems**. The algorithm involves idling the vehicles when needed.

The files:  
- createIdlingSchedule_random.py - Creating schedules for vehicles with the specified # of vehicles, # of cells, max idle time. It involves: 
  1. get_initial_schedules(): getting the shortest paths for the vehicles, independent of other vehicles
  2. get_collision(): finding the first collisions for each pair of vehicles
  3. solve_collisions(): for each collision, solve it by rerouting or idling either of the vehicles (chosen based on which change to the schedule leads to the least number of additional steps)
  4. Confirm the change is legal using check_transition_legality(), check_exclusiveness() and check_requestsatisfaction()
  
- createIdlingSchedule_functions.py - functions for running the algorithm

- ratingSchedule_functions.py - functions for performing the legality checks

- problemA.txt, problemB.txt - sample input of the environment. In particular, it consists of the following variables
  - E represents the edges between the positions in the factory
  - varYiTrueA - the initial positions of the vehicles
  - varYfTrueA - the final positions of the vehicles
  - varYtTrueA - idling time needed for the vehicles at certain positions
