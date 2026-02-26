#
# Check prop
#
# dml c 2016
#
# representing propositional logic - PL
#

# from Wumpus world, p11=pit in room 1,1 etc
#
symbols = ['p11', 'p12', 'p21', 'p22' ]
#

# one example model

model = [('p11',True), ('p12',False), ('p21', True), ('p22',False)]

# a statement in PL

alpha = [ 'p11', 'or', 'p12' ] # p11 or p12



#
def isTrue(prop,model):
    '''Check whether prop is true in model'''
    # assumes prop and model use the list/logic notation
    if isinstance(prop,str):
        return (prop,True) in model
    if prop[0]=='not':
        return not isTrue(prop[1],model)
    elif prop[1]=='and':
        return isTrue(prop[0],model) and isTrue(prop[2],model)
    elif prop[1]=='or':
        return isTrue(prop[0],model) or isTrue(prop[2],model)
    elif prop[1]=='implies':
        return (not isTrue(prop[0],model)) or isTrue(prop[2],model)
    elif prop[1]=='iff':
        left = (not isTrue(prop[0],model)) or isTrue(prop[2],model)
        right= (not isTrue(prop[2],model)) or isTrue(prop[0],model)
        return (left and right)
    return False


#
# Enumerate all possible models
#

def ttc(symbols,model):
    if len(symbols)==0:
        print("model ",model)
        return 
    else:
        p = symbols[0]
        rest = list(symbols[1:len(symbols)])
        ttc(rest,model+[(p,True)])
        ttc(rest,model+[(p,False)])
        return
#
#