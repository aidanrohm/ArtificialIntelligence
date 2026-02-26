#
# resolution theorem proving
# dml c 2018
#

import copy
import itertools

import findpairs as fp

#
# disjunctive clauses in the knowledge base
#
c1 = set(['np21','b11'])
c2 = set(['nb11','p12','p21'])
c3 = set(['np12','b11'])
c4 = set(['b11'])
c5 = set(['np12'])


#
# KB is a conjunct of all these clauses
#
KB  = [c1,c2,c3,c4,c5]


#
# resolution asks: is KB and not Query satusfiable
#
NotQuery = set(['p21'])
#
ArgumentToResolution=list(KB)
ArgumentToResolution.append(NotQuery)


#
# Given two clauses find a comp pair of literals
#
# def findCompPairs(w1,w2):
# returns either True,prop1,prop2
# or false,'',''
#
# e.g., True,'np11','p11'
#
def findCompPairs(w1,w2):
    for p1 in w1:
        for p2 in w2:
            if p1[0]=='n' and p1[1:]==p2:
                return True, p1, p2
            if p2[0]=='n' and p2[1:]==p1:
                return True, p1, p2
            
    return False,'',''

#
# Resolve two clauses on a comp pair p1,p2
# returns the resultant clause
#
def resolvePair(c1,c2,p1,p2):
    # the order is important - do the diff BEFORE the union
    cc1 = copy.deepcopy(c1)
    cc2 = copy.deepcopy(c2)
    cc1.remove(p1)
    cc2.remove(p2)
    clause =cc1.union(cc2)
    return clause

# Resolution theorem prover
# assumes the input is CNF of a list of sets
# and that it contains the negated query clause
# returns whether the query is true or not

def resolve(KBandNotQuery):
    '''apply resolution theorem proving to clauses in argument'''
    iter=0
    theKB=copy.deepcopy(KBandNotQuery)
    while True:
        print("Iteration ",iter," KB size=",len(theKB))
        newKB = copy.deepcopy(theKB)
        for cpair in itertools.product(theKB,theKB):
            foundPair,prop1,prop2 = fp.findCompPairs(cpair[0],cpair[1])
            if foundPair:
                clause = resolvePair(cpair[0],cpair[1],prop1,prop2)
                if len(clause)==0:
                    print(cpair[0]," with ",cpair[1]," makes empty")
                    print("Empty set generated: => Query is True")
                    return True
                if not clause in newKB:
                    print(cpair[0]," with ",cpair[1]," makes ",clause)
                    newKB.append(clause)
        if theKB==newKB:
            print("No new clauses generated: Query is False")
            return False
        theKB = newKB
        iter +=1
    

#
#
resolve(ArgumentToResolution)
