# Aidan Rohm
# Midterm Exam Question 1 Part A
# Professor Lyons
# March 19, 2026

import math
import numpy as np

#
# selects a map with a global variable
#
gMap = np.full((10, 10), 1)  # cost is just one

#
# Successor fn: generate list of next grid cells
# assumes the map is a numpy array, and state is (x,y)
#
def successor(themap, state):
	"""generate list of successors of state in the map"""
	
	h, w = np.shape(themap)
	successorList = []
	for mx in [-1, 0, 1]:
		for my in [-1, 0, 1]:
			if state[0] + mx in range(0, w) and state[1] + my in range(0, h):
				successorList.append((state[0] + mx, state[1] + my))
	return successorList

#
# calculates straight line distance as a heuristic for ASTAR
#
def sld(loc1, loc2):
	"""return the straight line distance from loc1 to loc2"""
	xdel = loc1[0] - loc2[0]
	ydel = loc1[1] - loc2[1]
	dist = math.sqrt(xdel * xdel + ydel * ydel)
	return dist

#
# Implementing A* search as requested by the problem,
# only changing what is necessary from the provided bfs
#
def astar(themap, frontier, goal):
	"""Carrying out an A* search tree for goal from node in frontier"""
	
	searched = []
	expanded = 0
	
	while len(frontier) > 0:
		# This is the only change from the provided bfs, we sort the frontier by cost + heuristic
		# We use a lambda function to calculate the sorting key for each node in the frontier, 
		# which is the sum of the cost to reach that node and the straight line distance from that node to the goal.
		# This is effectively f(n) = g(n) + h(n) where g(n) is the actual cost so far and h(n) is the heuristic estimate cost to reach the goal.
		frontier.sort(key=lambda n: n[1] + sld(n[0], goal))
		location, cost, path = frontier.pop(0)
		
		if location in searched:
			continue
		
		# This is how we keep track of the number of nodes expanded.
		# We increment the expanded counter each time we pop a node from the frontier and process it, 
		# which indicates that we are expanding that node.
		expanded += 1
		
		if location == goal:
			print(f"Found goal {location}")
			print(f"Nodes expanded: {expanded}")
			return path + [goal]
		
		succList = successor(themap, location)
		searched.append(location)
		frontierLocs = [f[0] for f in frontier]
		
		for next_location in succList:
			x, y = int(next_location[0]), int(next_location[1])
			stepcost = themap[x][y]
			
			if next_location not in searched and stepcost > 0:
				newnode = [next_location, cost + stepcost, path + [location]]
				
				if next_location in frontierLocs:
					idx = frontierLocs.index(next_location)
					if newnode[1] < frontier[idx][1]:
						frontier[idx] = newnode
				else:
					frontier.append(newnode)
	
	print(f"No route to {str(goal)}")
	print(f"Nodes expanded: {expanded}")
	return []

#
# the search function that is exported from this module
#
def search(themap, frontier, goal):
	return astar(themap, frontier, goal)

#
# test
#
def test():
	start = (0, 0)
	startnode = [start, 0, []]
	frontier = [startnode]
	path = astar(gMap, frontier, (5, 5))
	print(path)
	return

if __name__ == "__main__":
	test()