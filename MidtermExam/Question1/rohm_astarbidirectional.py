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
# calculates straight line distance as a heuristic for astar_bidirectional
#
def sld(loc1, loc2):
	"""return the straight line distance from loc1 to loc2"""
	xdel = loc1[0] - loc2[0]
	ydel = loc1[1] - loc2[1]
	dist = math.sqrt(xdel * xdel + ydel * ydel)
	return dist

# A simple function to reconstruct the path, combining the two paths generated
def reconstruct_path(meet, f_best, b_best):
    """Combine the forward path -> meet with backward path -> goal"""
    forward_path = f_best[meet][2] + [meet]
    backward_path = b_best[meet][2] + [meet]

    # The backward path is the goal to the meet, so we need to reverse it
    backward_path.reverse()

    # Avoid repeating the meeting point in the combined path
    return forward_path + backward_path[1:]

# Bidirectional A* search implementation
def astar_bidirectional(themap, frontier, goal):
    """Carrying out a bidirectional A* search for a goal from a node in the frontier"""
    start = frontier[0][0]              # Assuming frontier is initialized with the start node
    forward_frontier = [[start, 0, []]]       # Forward frontier: (location, cost, path)
    backward_frontier = [[goal, 0, []]]        # Backward frontier: (location, cost, path)

    forward_closed = []
    backward_closed = []

    forward_best = {start: [start, 0, []]}      # Best known cost and path to each node in forward search
    backward_best = {goal: [goal, 0, []]}       # Best known cost and path to each node in backward search

    forward_expanded = 0            # Counter for nodes expanded in the forward search
    backward_expanded = 0           # Counter for nodes expanded in the backward search

    while len(forward_frontier) > 0 and len(backward_frontier) > 0:
        
        # -------- Forward Search Step --------
        forward_frontier.sort(key=lambda n: n[1] +sld(n[0], goal))
        f_location, f_cost, f_path = forward_frontier.pop(0)

        if f_location in forward_closed:
            continue
        
        forward_expanded += 1
        forward_closed.append(f_location)
        forward_best[f_location] = [f_location, f_cost, f_path]

        if f_location in backward_best:
            fullpath = reconstruct_path(f_location, forward_best, backward_best)
            print(f"Searches met at {f_location}")
            print(f"Forward nodes expanded: {forward_expanded}")
            print(f"Backward nodes expanded: {backward_expanded}")
            return fullpath

        f_succlist = successor(themap, f_location)

        for next_location in f_succlist:
            x, y = int(next_location[0]), int(next_location[1])
            stepcost = themap[x][y]

            if stepcost > 0 and next_location not in forward_closed:
                newcost = f_cost + stepcost
                newnode = [next_location, newcost, f_path + [f_location]]

                if next_location not in forward_best or newcost < forward_best[next_location][1]:
                    forward_best[next_location] = newnode

                    replaced = False
                    for i in range(len(forward_frontier)):
                        if forward_frontier[i][0] == next_location:
                            if newcost < forward_frontier[i][1]:
                                forward_frontier[i] = newnode
                            replaced = True
                            break
                    if not replaced:
                        forward_frontier.append(newnode)
        
        # -------- Backward Search Step --------
        backward_frontier.sort(key=lambda n: n[1] + sld(n[0], start))
        b_location, b_cost, b_path = backward_frontier.pop(0)

        if b_location in backward_closed:
            continue
        
        backward_expanded += 1
        backward_closed.append(b_location)
        backward_best[b_location] = [b_location, b_cost, b_path]

        if b_location in forward_best:
            fullpath = reconstruct_path(b_location, forward_best, backward_best)
            print(f"Searches met at {b_location}")
            print(f"Forward nodes expanded: {forward_expanded}")
            print(f"Backward nodes expanded: {backward_expanded}")
            return fullpath
        
        b_succlist = successor(themap, b_location)

        for next_location in b_succlist:
            x, y = int(next_location[0]), int(next_location[1])
            stepcost = themap[x][y]

            if stepcost > 0 and next_location not in backward_closed:
                newcost = b_cost + stepcost
                newnode = [next_location, newcost, b_path + [b_location]]

                if next_location not in backward_best or newcost < backward_best[next_location][1]:
                    backward_best[next_location] = newnode

                    replaced = False
                    for i in range(len(backward_frontier)):
                        if backward_frontier[i][0] == next_location:
                            if newcost < backward_frontier[i][1]:
                                backward_frontier[i] = newnode
                            replaced = True
                            break
                    if not replaced:
                        backward_frontier.append(newnode)
            
    print(f"No route to {str(goal)}")
    print(f"Forward nodes expanded: {forward_expanded}")
    print(f"Backward nodes expanded: {backward_expanded}")
    print(f"Total nodes expanded: {forward_expanded + backward_expanded}")
    return []

#
# the search function that is exported from this module
#
def search(themap, frontier, goal):
	return astar_bidirectional(themap, frontier, goal)

#
# test
#
def test():
	start = (0, 0)
	startnode = [start, 0, []]
	frontier = [startnode]
	path = astar_bidirectional(gMap, frontier, (5, 5))
	print(path)
	return

if __name__ == "__main__":
	test()