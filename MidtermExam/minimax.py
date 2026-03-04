#
# Minimax search for the TTT game
# c dml 2019
#
#
# State[i][j] = i,j element of board ="X", "O" or " "
# Assumes O is the agent and X the opponent
#

# global variable to hold nodes expanded and list of all
gNodesList=[]
gNodes=0

import copy

# is this a win/lose/draw
def terminal(s):
    '''determine if s is a win/lose/draw and return T/F and 1/-1/0'''
    if wins(s,'O'):
        return True,1
    elif wins(s,'X'):
        return True,-1
    elif draw(s):
        return True,0
    #
    return False,0


# check for winning config of symbol c
def wins(s,c):
    ''' return true if any winning config of c'''
    w=c+c+c
    return s[0][0]+s[0][1]+s[0][2]==w or s[1][0]+s[1][1]+s[1][2]==w or \
       s[2][0]+s[2][1]+s[2][2]==w or s[0][0]+s[1][0]+s[2][0]==w or \
       s[0][1]+s[1][1]+s[2][1]==w or s[0][2]+s[1][2]+s[2][2]==w or \
       s[0][0]+s[1][1]+s[2][2]==w or s[0][2]+s[1][1]+s[2][0]==w

# check for a draw
def draw(s):
    '''return True iff all positions are non empty'''
    for i in range(0,3):
        for j in range(0,3):
            if s[i][j]==" ":
                return False
    return True

#expand a board position with symbol c
def expand(s,c):
    '''returns a list of all next move by c game states'''
    nextList=[]
    # your expansion code goes here
    return nextList


#extract the move that was made
def extractmove(a,b):
    '''find ther one move difference and return the indices'''
    for i in range(0,3):
        for j in range(0,3):
            if a[i][j]!=b[i][j]:
                return (i,j)

#minimax MAX step

def maxval(s):
    '''do max step on s and return value'''
    global gNodes
    isTerminal,util = terminal(s)
    if isTerminal:
        return util
    v = -100 # -ve infinity
    for g in expand(s,'O'):
        mv = minval(g)
        if mv>v:
            v = mv
    gNodes = gNodes+1
    return v

#minimmax MIN step

def minval(s):
    '''do min step on s and return value'''
    global gNodes
    isTerminal,util = terminal(s)
    if isTerminal:
        return util
    v = 100 # +ve infinity
    for g in expand(s,'X'):
        mv = maxval(g)
        if mv<v:
            v = mv
    gNodes = gNodes+1
    return v

#minimax decision procedure
# should not already be a winning board
#
def minimax(s):
    '''will return the best move for 'O' in current state'''
    global gNodes,gNodesList
    
    gNodes = 0
    v = -100 # -ve infinity
    move = s
    for g in expand(s,'O'):
        mv = minval(g)
        if mv>v:
            v = mv
            move = g
    action = extractmove(s,move)
    
    print("Nodes expanded in this move: ",gNodes)
    gNodesList.append(gNodes)
    
    return action
