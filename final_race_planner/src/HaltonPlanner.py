import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list 
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    
    while len(self.open) != 0:
        current = min(self.open, key=self.open.get)  # Get the node with the lowest fScore
	
	if current == self.tid:
	    return self.get_solution(self.tid)

	if((self.parent[current] is None) or (self.planningEnv.manager.get_edge_validity(self.planningEnv.get_config(current),
											    self.planningEnv.get_config(self.parent[current])))):
	   
	    self.closed[current] = self.open[current]
	    del self.open[current]
	else:
 	    del self.open[current]
    	    continue

	for neighbor in self.planningEnv.get_successors(current):
	    if neighbor in self.closed:
		continue  # Ignore already evaluated nodes

	    tentative_g = self.gValues[current] + self.planningEnv.get_distance(current, neighbor)
	   	
	    if neighbor not in self.open or tentative_g < self.gValues[neighbor]:
		self.parent[neighbor] = current
		self.gValues[neighbor] = tentative_g
		self.open[neighbor] = tentative_g + self.planningEnv.get_heuristic(neighbor, self.tid)

    # If the open set is empty but the goal is not reached

    return []

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # Pick random indices i and j
      i = random.randint(0, len(plan) - 1)
      j = random.randint(0, len(plan) - 1)

      # Redraw if i == j
      if i == j:
          continue

      # Switch i and j if i > j
      if i > j:
          i, j = j, i

      # Get the configurations for i and j
      start_config_i = plan[i][0]
      goal_config_j = plan[j][-1]

      # Check if there is a path between i and j
      if self.planningEnv.manager.is_edge_valid(start_config_i, goal_config_j):
          # Get the new path between i and j
          new_path = self.planningEnv.manager.get_edge_path(start_config_i, goal_config_j)

          # Reformat the plan to insert the new path and remove the old section between i and j
          plan = plan[:i] + new_path + plan[j+1:]
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
     
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly


      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
