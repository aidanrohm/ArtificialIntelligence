# CISC 4597
# BFS algorithm
# dml 2023

#
#Part of the romania roadmapp
#
romania = [("arad","zerind",75), ("arad","sibiu",140),("arad","timisoara",118),("zerind","oradea",71),
       ("oradea","sibiu",151),("timisoara","lugoj",111),("lugoj","mehadia",70),("mehadia","drobeta",75),
       ("drobeta","craiova",120),("craiova","rimnicu",146),("rimnicu","sibiu",80),
       ("craiova","pitesti",138),("rimnicu","pitesti",97),("sibiu","fagaras",99),
       ("pitesti","bucharest",101), ("fagaras","bucharest",211), ("giurgiu","bucharest",90)]



#
# Successors fn: successor(state)=set of states arrived at for any action in state
#
def successors(roadmap,city):
    """generate list of successors of startcity in roadmap"""
    # assumes format of roadmap and that city is a strong
    successorList=[]
    for from_city,to_city,dist in roadmap:
        if from_city==city:
            successorList.append( (to_city, dist) )
        elif to_city==city:
            successorList.append( (from_city, dist) )
    return successorList



# test the arad, sibiu, fagaras route to bucharest

#successors(romania,'arad')

#
# BF treesearch on the roadmap given a frontier =['name of start city'] and goal = 'name of goal city'
# Repors found when it has found a path
#
def BFSearch1(roadmap,frontier,goal):
    """carry out a tree search for goal from this frontier (set of unexpanded nodes to search)"""
    # assumes that the start state is in the frontier
    # will not return the path
    while len(frontier)>0:
        city = frontier.pop(0) # this is the 'strategy' - just pick the first one
        if city == goal:
            print ('Found goal city!!')
            return city
        nextcitylist = successors(roadmap,city) # all the cities accessible from here
        # print(nextcitylist) # uncomment to see search in action
        for nextcity,dist in nextcitylist:
            if not nextcity in frontier:
                frontier.append(nextcity) # put them at the end
    return "No route to "+goal




#frontier=['arad'] 
#BFSearch1(romania,frontier,'bucharest') # works

#BFSearch1(romania,frontier,'berlin') # does not end, why?

