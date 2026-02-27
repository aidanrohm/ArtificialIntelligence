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

w1 = ['np11', 'b11', 'nb12']
w2 = ['p13','b12']