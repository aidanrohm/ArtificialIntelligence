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
        for nextcity,dist in nextcitylist:
            if not nextcity in frontier:
                frontier.append(nextcity) # put them at the end
    return "No route to "+goal




#frontier=['arad'] 
#BFSearch3(romania,frontier,'bucharest') # works

#BFSearch3(romania,frontier,'berlin') # does not end, why?


#
# BF treesearch on the roadmap given a frontier =['name of start city'] and goal = 'name of goal city'
#
def BFSearch2(roadmap,frontier,goal):
    """carry out a tree search for goal from this frontier (set of unexpanded nodes to search)"""
    # assumes that the start state is in the frontier
    # does not return path
    # works for cities not in list
    searched = [] # list of already seached cities
    while len(frontier)>0:
        city = frontier.pop(0) # thisis the strategy - just pick the first one
        if city == goal:
            print ('Found goal')
            return city
        nextcitylist = successors(roadmap,city) # all the cities accessible from here
        searched.append(city)

        for nextcity,dist in nextcitylist:
            if not nextcity in frontier and not nextcity in searched:
                frontier.append(nextcity) # put them at the end
    return "No route to "+goal


#
# treesearch on the roadmap given a frontier =['name of start city',[]] and goal = 'name of goal city'
#
# Search node = [ name of city, [ list of city names so far ] ]
# frontier = list of search nodes
# goal = name of city
#
# frontier = [ ['arad',[]] ] # start with one searchnode 
#
def BFSearch3(roadmap,frontier,goal):
    """carry out a tree search for goal from this frontier (set of unexpanded nodes to search)"""
    # assumes that the start state is in the frontier
    # returns the path if found
    searched = list() # list of already seached cities
    while len(frontier)>0:
        city,path = frontier.pop(0) # this is the strategy - just pick the first one
        # this city is the root for further searching
        if city == goal:
            print ('Found goal')
            return path+[goal]
        nextcitylist = successors(roadmap,city) # all the cities accessible from here
        searched.append(city)
        #
        frontiercity = [ fcity for fcity,fpath in frontier]
        #frontiercity=list() # can't check frontier directly for duplicates so extract cities to list
        #for city,path in frontier:
        #    frontiercity.append(city)
        #
        for nextcity,dist in nextcitylist:
            if not nextcity in frontiercity and not nextcity in searched:
                newnode=[nextcity,path+[city]] # expand the path by one city
                frontier.append(newnode)# put them at the end
    return "No route to "+goal


#
#

#
# treesearch on the roadmap given a frontier =['name of start city',[]] and goal = 'name of goal city'
#
# Search node = [ name of city, [ list of city names so far ] , cumulative cost]
# frontier = list of search nodes
# goal = name of city
#
# frontier = [ ['arad,[],0] ] # start with one searchnode 
#
def BFSearch(roadmap,frontier,goal):
    """carry out a tree search for goal from this frontier (set of unexpanded nodes to search)"""
    # assumes that the start state is in the frontier
    # returns path if found
    # calculates cost
    searched = list() # list of already seached cities
    while len(frontier)>0:
        city,path,dist = frontier.pop(0) # ordered list, first is least cost
        # this city is the root for further searching
        if city == goal:
            print ('Found goal')
            return path+[goal], dist # the path and the overall length
        nextcitylist = successors(roadmap,city) # all the cities accessible from here
        searched.append(city)
        #
        frontiercity = [ fcity for fcity,fpath,fdist in frontier]

        for nextcity,stepdist in nextcitylist:
            if not nextcity in frontiercity and not nextcity in searched:
                newnode=[nextcity,path+[city], dist+stepdist] # expand the path by one city
                frontier.append(newnode)# put them at the end
        frontier.sort(key=lambda node : node[2])
    return "No route to "+goal


