#
# BFS implementation
# for occupancy grid
#
# c 2016 dml
#
import math
import numpy as np
#
#
# selects a map with a global variable
#
gMap = np.full((10,10),1) # cost is just one

#
# Successor fn:generate list of next grid cells
# assumes the map is an numpy array, and state is (x,y)
#
def successor(themap,state):
    """generate list of successors of state in the map"""
    
    h,w=np.shape(themap)
    successorList=[]
    for mx in [-1,0,1]:
        for my in [-1,0,1]:
            if state[0]+mx in range(0,w) and state[1] +my in range(0,h):
                successorList.append( (state[0]+mx, state[1]+my) )
    return successorList

#
# calculates straight line distance as a heuristic for ASTAR

def sld(loc1,loc2):
    """return the straight line distance from loc1 to loc2"""
    xdel = loc1[0]-loc2[0]
    ydel = loc1[1]-loc2[1]
    dist = math.sqrt( xdel*xdel + ydel*ydel )
    return dist 



#
# BFS  on the map given a frontier
#                 [(xstart,ystart),cost,[]] and
#  goal = (xgoal,ygoal)
#
#
def bfsGrid(themap, frontier, goal):
    """carry out a search tree for goal from node in frontier"""
    searched = list() # list of already seached locations
    while len(frontier)>0:
        location,cost,path= frontier.pop(0)
        # 'pop' minimum f(n) node from frontier
        #print(f"bfsGRID Expands {location} f={cost}")
        if location == goal:
            print (f'Found goal {location}')
            return path+[goal]
        succList = successor(themap,location) # get successors
        searched.append(location)             # mark this location
        # break out just the locations in the frontier
        frontierLocs=[ f[0] for f in frontier ]
        # add sucessors to the frontier
        for next_location in succList:
            xf,yf=next_location
            x,y=int(xf),int(yf)
            stepcost=themap[x][y] # cost of this cell, <0 means obstacle
            if (not next_location in searched) and \
               (not next_location in frontierLocs) and stepcost>0:
                newnode=[next_location,cost+stepcost,path+[location]]
                # expand the path by one city
                frontier.append(newnode)# put them at the end
    print(f"No route to {str(goal)}")
    return []

# the search function that is exported from this module
def search(themap, frontier, goal):
  return bfsGrid(themap, frontier, goal)

# test
def test():
  start = (0,0)
  startnode= [start,0,[]]
  frontier = [ startnode ]
  path=bfsGrid(gMap,frontier,(5,5))
  print(path)
  return

if __name__=="__main__":
    test()
    

