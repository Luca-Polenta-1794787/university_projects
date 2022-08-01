# coding=utf-8

from unified_planning.shortcuts import *
#from unified_planning.solvers import PlanGenerationResultStatus
from unified_planning.engines import PlanGenerationResultStatus

#NOTE: To disable printing of planning engine credits:
up.shortcuts.get_env().credits_stream = None
    
def ourSolver(problem):
    for planner_name in ['pyperplan']: #, 'fast_downward'
      with OneshotPlanner(name=planner_name) as planner:
        result = planner.solve(problem)
        if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
          #print(f'pyperplan found a plan.\nThe plan is: {result.plan}')
          print('pyperplan found a plan.\nThe plan is: %s' %(result.plan))
          completePlan = (str(result.plan)[1:-1])          # Lo trasformo in una stringa
          firstMovePlan = completePlan.split(', move')[0]  # Estraggo solo la prima mossa, cioè:        move(disk_i, loc_from, loc_to)
          print('The first move to do is: %s' %(firstMovePlan))
          firstMovePlan = firstMovePlan[5:-1] 	           # Estraggo solo il contenuto utile, cioè:    disk_i, loc_from, loc_to
          return firstMovePlan
        else:
          noPlan = "No plan found"
          print(noPlan)
          return noPlan
	        
	  

def initProblem(num_disks, leftStaff, centerStaff, rightStaff):
    
    # Saving initial and goal location names in python variables
    INIT = 'loc_1'  #loc_2  loc_3
    DEST = 'loc_3'
    

    Items_list = ['loc_1', 'loc_2', 'loc_3']
    for i in range(num_disks):
      Items_list.append('disk_%d' %(i+1))
    # ONLY FOR DEBUG: print(Items_list)

    # First, we declare a "Location" type 
    #Location = UserType('Location')
    Item = UserType('Item')
    
    # We create a new problem
    problem = Problem('towersHanoi')


    # Declare the fluents:
    # - `robot_at` is a predicate modeling the robot position,
    # - `connected` is a static fluent for modeling the graph connectivity relation
    is_disk = Fluent('is_disk', BoolType(), disk_i=Item)
    clear = Fluent('clear', BoolType(), disk_i=Item)
    on = Fluent('on', BoolType(), disk_i=Item, disk_pos=Item)
    smaller = Fluent('smaller', BoolType(), disk_i=Item, disk_j=Item)
    #robot_at = Fluent('robot_at', BoolType(), position=Location)
    #connected = Fluent('connected', BoolType(), l_from=Location, l_to=Location)

    # Add the fluents to the problem, a Fluent can be resused in many problems
    # The default values are optional and can be any value (not forcing closed-world assumption) 
    #problem.add_fluent(robot_at, default_initial_value=False)
    #problem.add_fluent(connected, default_initial_value=False)
    problem.add_fluent(is_disk, default_initial_value=False)
    problem.add_fluent(clear, default_initial_value=False)
    problem.add_fluent(on, default_initial_value=False)
    problem.add_fluent(smaller, default_initial_value=False)

    # Create a simple `move` action  
    move = InstantaneousAction('move', disk=Item, l_from=Item, l_to=Item)
    disk = move.parameter('disk')
    l_from = move.parameter('l_from')
    l_to = move.parameter('l_to')
    move.add_precondition(is_disk(disk))
    move.add_precondition(smaller(disk, l_to))
    move.add_precondition(on(disk, l_from))
    move.add_precondition(clear(disk))
    move.add_precondition(clear(l_to))
    move.add_effect(clear(l_from), True)
    move.add_effect(on(disk, l_to), True)
    move.add_effect(on(disk, l_from), False)
    move.add_effect(clear(l_to), False)
    problem.add_action(move)

    # Programmatically create a map from location name to a new `Object` of type `Location`
    items = {str(i) : Object(str(i), Item) for i in Items_list}
    #locations = {str(l) : Object(str(l), Location) for l in location_map.nodes}

    # Add all the objects to the problem
    problem.add_objects(items.values())

    # Setting the initial location
    #problem.set_initial_value(robot_at(locations[INIT]), True)
    for i in range(num_disks):
      problem.set_initial_value(is_disk(items['disk_%d' %(i+1)]), True)
      ############ problem.set_initial_value(on(items[f'disk_{i+1}'], items['loc_1']), True)
      if i!=num_disks-1:
	############ problem.set_initial_value(on(items[f'disk_{i+1}'], items[f'disk_{i+2}']), True)
        for j in range(i+1, num_disks):
          problem.set_initial_value(smaller(items['disk_%d' %(i+1)], items['disk_%d' %(j+1)]), True)

      for k in range(3): #3 is the num_locs
        problem.set_initial_value(smaller(items['disk_%d' %(i+1)], items['loc_%d' %(k+1)]), True)


    # For the Left Staff
    precValue=""
    for i, value in enumerate(leftStaff):
      problem.set_initial_value(on(items['disk_'+value], items['loc_1']), True)
      if i!=0:
        problem.set_initial_value(on(items['disk_%s' %(value)], items['disk_%s' %(precValue)]), True)
      if i==len(leftStaff)-1:
        problem.set_initial_value(clear(items['disk_%s' %(value)]), True)
      precValue = value
    # For the Center Staff
    precValue=""
    for i, value in enumerate(centerStaff):
      problem.set_initial_value(on(items['disk_%s' %(value)], items['loc_2']), True)
      if i!=0:
        problem.set_initial_value(on(items['disk_%s' %(value)], items['disk_%s' %(precValue)]), True)
      if i==len(centerStaff)-1:
        problem.set_initial_value(clear(items['disk_%s' %(value)]), True)
      precValue = value
    # For the Right Staff
    precValue=""
    for i, value in enumerate(rightStaff):
      problem.set_initial_value(on(items['disk_%s' %(value)], items['loc_3']), True)
      if i!=0:
        problem.set_initial_value(on(items['disk_%s' %(value)], items['disk_%s' %(precValue)]), True)
      if i==len(rightStaff)-1:
        problem.set_initial_value(clear(items['disk_%s' %(value)]), True)
      precValue = value    
	    
    if(len(leftStaff)==0):
        problem.set_initial_value(clear(items['loc_1']), True)
    if(len(centerStaff)==0):
        problem.set_initial_value(clear(items['loc_2']), True)
    if(len(rightStaff)==0):
        problem.set_initial_value(clear(items['loc_3']), True)



    # Initializing the connectivity relations by iterating over the graph edges
    #for (f, t) in location_map.edges:
    #  problem.set_initial_value(connected(locations[str(f)], locations[str(t)]), True)
    #  problem.set_initial_value(connected(locations[str(t)], locations[str(f)]), True)

    # Setting the goal
    for i in range(num_disks-1):
      problem.add_goal(on(items['disk_%d' %(i+1)], items['disk_%d' %(i+2)]))
    problem.add_goal(on(items['disk_%d' %(num_disks)], items[DEST]))

    # Printing the problem data structure in human-readable form
    # (We can also print in PDDL and ANML)
    # ONLY FOR DEBUG: print(problem)
    
    print("Made the request for a plan. In processing:")
    # Call the Solver
    moveToDo = ourSolver(problem)
    
    return moveToDo
    
