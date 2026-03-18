# 
# Aidan Rohm - Artificial Intelligence
# Midterm Exam Question 2
# Minimax Search for Tic Tac Toe
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
    for i in range(0,3):
        for j in range(0,3):
            if s[i][j]==" ":
                next = copy.deepcopy(s)
                next[i][j]=c
                nextList.append(next)
    return nextList


#extract the move that was made
def extractmove(a,b):
    '''find ther one move difference and return the indices'''
    for i in range(0,3):
        for j in range(0,3):
            if a[i][j]!=b[i][j]:
                return (i,j)

# Check whether X has an inevitable win within 2 ply (O -> X)
# Based on the pseudocode developed from Parts A and B
#   Ply 1: O plays somewhere (all possible moves are checked)
#   Ply 2: X responds -> does X have a winning follow up for every O move?
# Returns True if X wins inevidably, False otherwise
def win_inevitable_2ply(s):
    '''Return True if X has an inevitable win within 2 ply (O -> X), False otherwise'''

    # Ply 1: try ALL possible O moves on the current board
    # Assuming that X can win no matter what
    x_wins_all_responses = True

    for board_after_o in expand(s, 'O'):

        # If O wins immediately after ply 1 -> X cannot win
        if wins(board_after_o, 'O'):
            wins_x_all_responses = False
            break
        
        # Ply 2: check if ANY of X's responses win
        # A default flag, assuming that X cannot win
        x_can_win = False

        for board_after_x in expand(board_after_o, 'X'):

            # Check whether X wins in this resulting board state
            if wins(board_after_x, 'X'):
                x_can_win = True
                break # X has a winning reply to this O move

        # Only executes if there is no win for X after ply 2
        if not x_can_win:
            x_wins_all_responses = False
            break # X does not have a winning reply to this O move
    
    return x_wins_all_responses

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
    
    # Check for terminal state first before proceeding
    isTerminal, util = terminal(s)
    if isTerminal:
        print("Terminal state reached")
        gNodesList.append(gNodes)
        return None
    
    # Check if X has an inevitable win within 2 ply (O -> X)
    # If so, print a warning - no O move can prevent X from winning
    if win_inevitable_2ply(s):
        print("2-ply check: X has an inevitable win regardless of O's response")

    # Fall through to full minimax search regardless
    v = -100 # -ve infinity
    move = s

    for g in expand(s,'O'):
        mv = minval(g)
        if mv>v:
            v = mv
            move = g
    action = extractmove(s,move)

    print("Nodes expanded in this move: ", gNodes)
    gNodesList.append(gNodes)

    return action
