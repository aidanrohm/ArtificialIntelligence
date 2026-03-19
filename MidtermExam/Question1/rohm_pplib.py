#
# Helper functions for maps
# CISC 3060 dml c2020
#

import copy
import rohm_astar as s  # A* search
import numpy as np
import matplotlib.pyplot as plt

# previously stored and dilated occupancy map
# for the enclosed.world gazebo world
#
gMap = np.loadtxt(r'enclosed_ocmap.txt')

# Display a map 
#
def display(themap):
    mymap=np.array(themap)
    plt.pcolor(np.flip(mymap,0))
    plt.show()
    return



# show the path symbolically on the map
#
def showpath(themap,path):
    mymap=np.array(themap) 
    for pt in path:
        #print(pt[0], pt[1])
        mymap[int(pt[0])][int(pt[1])]=20
    display(mymap)

# helper function to do pathplanning for
#  globally defined map gMap
#
def planPath(start,goal):
    # translate robot coordinates to occ map = 10*coordinat+50
    start=(10*start[0]+50,10*start[1]+50)
    goal=(10*goal[0]+50,10*goal[1]+50)
    print("Path planning from:",start," to ",goal)
    # call the path planner on the occ map
    path=s.search(gMap,[ [start,0,[]] ],goal)
    if len(path)==0: # no path!! terminate
        exit()
    showpath(gMap,path)
    # translate the loccupancy coordinate back to robot coordinates
    newpath=[]
    for loc in path:
        for i in range(0,2):
            newpath.append( ( (loc[0]-50)/10.0,  (loc[1]-50)/10.0 ) )
    return newpath

#---------------------

