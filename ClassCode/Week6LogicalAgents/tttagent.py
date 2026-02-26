#
#
#
import  minimax as mx
#


def search(state):
    for i in range(0,3):
        for j in range(0,3):
            if state[i][j]==" ":
                return (i,j),True
    return (0,0),False # no move

#
def mmsearch(state):
    act = mx.minimax(state)
    print ("Minimax chooses ",act)
    if act!=None:
    	return act,True
    return (0,0),False # no move
#
