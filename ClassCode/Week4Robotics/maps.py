
# Helper functions for maps
#
# All maps are 10x10 arrays
#
# Cell content value represents terrain:
# Easy=1, Rough=5, impossible=-1
#
#

import copy
#import astar as s 
import bfsGrid as s
import numpy as np
import matplotlib.pyplot as plt

map0 = [[1,1,1,1,1,1,1,1,1,3],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[2,1,1,1,1,1,1,1,1,1]]

map1 = [[1,1,1,1,1,1,1,1,1,3],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,1,1,1,1],
[1,1,1,1,1,1,-1,-1,-1,1],
[-1,-1,-1,-1,-1,-1,-1,-1,-1,1],
[-1,-1,-1,-1,-1,-1,-1,-1,-1,1],
[2,1,1,1,1,1,1,1,1,1]]

map2 = [[-1,-1,-1,-1,-1,-1,1,1,1,3],
[-1,1,1,1,1,-1,1,1,1,1],
[-1,1,1,1,1,-1,1,1,1,1],
[-1,1,-1,1,1,-1,1,1,1,1],
[-1,1,-1,1,1,1,1,1,1,1],
[-1,1,-1,-1,-1,-1,-1,-1,-1,-1],
[-1,1,-1,1,1,1,1,1,1,1],
[-1,1,1,1,1,1,1,1,1,1],
[-1,-1,-1,-1,-1,-1,-1,-1,1,1],
[2,1,1,1,1,1,1,1,1,1]]


# Display a map array symbolically
#

def display(themap):
    mymap=np.array(themap)
    plt.pcolor(mymap.transpose())
    plt.show()
    return


# show the path symbolically on the map
#
def showpath(themap,path):
    mymap=np.array(themap) 
    for x,y in path:
        mymap[x][y]=20
    display(mymap)
    return

# helper function to make a block in a map
# make sure the x,y start indices and the
# length l and width w are in your map!
#
def makeBlock(thismap,x,y,l,w):
    for i in range(l):
        for j in range(w):
            thismap[x+i][y+j]=-1
    return

# helper function to do pathplanning for
# all the globally defined maps

def runMapTests(thismap):
    start=(0,9)
    goal=(9,0)
    startnode = [start,0,[]]
    frontier = [ startnode ]
    path=s.search(np.array(thismap),frontier,goal)
    display(thismap)
    showpath(thismap,path)
    return

#---------------------
