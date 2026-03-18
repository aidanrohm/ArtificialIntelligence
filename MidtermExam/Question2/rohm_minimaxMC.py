#
# Aidan Rohm - Artificial Intelligence
# Midterm Exam Question 2
# Modified Minimax for Tic Tac Toe
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

def win_inevitable_2ply(s, player):
    '''Return True if player has an inevitable win within the tested ply pattern'''

    if player == 'X':
        opponent = 'O'
    else:
        opponent = 'X'

    # Try every legal move for the current player (ply 1)
    for g in expand(s, player):

        # If the current player wins immediately after ply 1, this is already a forced win
        if wins(g, player):
            return True

        # Assume this move is forcing unless an opponent reply breaks it
        player_forces_win = True

        # Try every legal response by the opponent (ply 2)
        for g2 in expand(g, opponent):

            # If the opponent wins immediately after ply 2, this move is not a forced win
            if wins(g2, opponent):
                player_forces_win = False
                break

            # After the opponent reply, the current player must still have
            # at least one immediate winning reply
            player_has_winning_reply = False

            # This is an essential check to see if the opponent can win on the next move
            # The third ply does mean that there is an inevitable win through the second ply
            # Or in other words, an inevitable win 2 ply
            for g3 in expand(g2, player):
                if wins(g3, player):
                    player_has_winning_reply = True
                    break

            if not player_has_winning_reply:
                player_forces_win = False
                break

        # If every opponent reply still allows the current player
        # to win on the following move, then this move is inevitable
        if player_forces_win:
            return True

    # There is no inevitable win
    return False

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
        print("Terminal state reached with utility: ", util)
        gNodesList.append(gNodes)
        return None

    # Check if O has inevitable win within the tested ply pattern
    # If so, return that move immediately without further search
    if win_inevitable_2ply(s, 'O'):
        for g in expand(s, 'O'):

            if wins(g, 'O'):
                action = extractmove(s, g)
                print("2-ply check: O has an inevitable win. Choosing move: ", action)
                print("Nodes expanded in this move: ", gNodes)
                gNodesList.append(gNodes)
                return action

            move_is_forcing = True

            for g2 in expand(g, 'X'):

                if wins(g2, 'X'):
                    move_is_forcing = False
                    break

                o_has_winning_reply = False

                for g3 in expand(g2, 'O'):
                    if wins(g3, 'O'):
                        o_has_winning_reply = True
                        break

                if not o_has_winning_reply:
                    move_is_forcing = False
                    break

            if move_is_forcing:
                action = extractmove(s, g)
                print("2-ply check: O has an inevitable win. Choosing move: ", action)
                print("Nodes expanded in this move: ", gNodes)
                gNodesList.append(gNodes)
                return action

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