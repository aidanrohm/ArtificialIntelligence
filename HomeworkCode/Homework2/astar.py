#
# ASTAR implementation
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
gMap = np.zeros((10,10))

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
    return dist # factor to scale close to text numbers



#
# A* search on the map given a fringe
#                 [(xstart,ystart),cost,[]] and
#  goal = (xgoal,ygoal)
#
#

def ASTARsearch(themap, fringe, goal):
    """carry out a tree search for goal from node in fringe"""
    searched = list() # list of already seached locations
    while len(fringe)>0:
        fmin = 10E6 #big number
        for node in fringe:
            g = node[1]
            h = sld(node[0],goal)
            f = g + h
            if f<fmin:
                rootnode = node # find smallest
                fmin = f
        fringe.remove(rootnode) # 'pop' minimum f(n) node from fringe
        root = rootnode[0]
        #print("ASTAR Expands ",root," f=",fmin)
        if root == goal:
            print ('Found goal',root)
            return rootnode[2]+[goal]
        
        succList = successor(themap,root) 
        searched.append(root)

        #Splitting the fringe into two for ease of testing for duplicates
        fringeLocs=[]
        locCosts={}
        for f in fringe:
            fringeLocs.append( f[0] )  # coordinates of cell on fringe
            locCosts[ str(f[0]) ]=f[1] # fringe cost of this cell

        for mapentry in succList:
            loc = mapentry
            x,y=int(loc[0]),int(loc[1])
            stepcost=themap[x][y] # cost of this cell
            if (not loc in searched) \
            and (not loc in fringeLocs or (loc in fringeLocs and locCosts[str(loc)]>stepcost+rootnode[1])) \
            and stepcost>0:
                newnode=[loc,rootnode[1]+stepcost,rootnode[2]+[root]]
                # expand the path by one grid cell
                fringe.append(newnode)
    print("No route to "+str(goal))
    return []

# function exported from the module

def search(themap, fringe, goal):
    return ASTARsearch(themap, fringe, goal)

#
#fringe=[ [(xstart,ystart),0, []] ]

