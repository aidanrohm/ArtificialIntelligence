# Aidan Rohm
# Forward chaining of Horn Clauses
# Corresponds to Homework 3 Question 3 --> My code
# DUE: March 5th, 2026

from collections import deque

def fchain(PROPS, RULES):
    '''Function for computing forward chaining of Horn Clauses'''
    # Initialize the VALS array, where propositions are initially False
    VALS = [False] * len(PROPS)

    # CTRS[i] = the number of premises in RULES[i]
    CTRS = [len(rule[0]) for rule in RULES]

    # Finding propositions that are facts --> The LHS is empty
    agenda = deque()
    for i, count in enumerate(CTRS):
        if count == 0:
            rhs_index = RULES[i][1]
            if not VALS[rhs_index]:
                VALS[rhs_index] = True
                agenda.append(rhs_index)

    # Process the agenda
    while agenda:
        p_index = agenda.popleft()

        # Checking every rule to see if p_index is in its LHS
        for i, rule in enumerate(RULES):
            lhs, rhs = rule
            if p_index in lhs:
                CTRS[i] -= 1

                # If all premises are now True, the RHS becomes True
                if CTRS[i] == 0:
                    if not VALS[rhs]:
                        VALS[rhs] = True
                        agenda.append(rhs)

    # Formatting the output
    truth_values = ['T' if v else 'F' for v in VALS]
    print(PROPS)
    print(truth_values)

# ------------------- EXAMPLE USAGE -------------------
# PROPS = ['A', 'B', 'Q', 'R']
# rule1 = [[0,1],2] : A and B => Q
# rule2 = [[2], 0]  : Q => A
# rule3 = [[], 0]   : A
# rule4 = [[], 1]   : B
# RULES = [rule1, rule2, rule3, rule4]

def main():
    '''Main function for setting parameters and calling the forward chaining function'''

    # Based on the homework example usage
    rule1 = [[0,1],2]
    rule2 = [[2], 0]
    rule3 = [[], 0]
    rule4 = [[], 1]

    PROPS = ['A', 'B', 'Q', 'R']
    RULES = [rule1, rule2, rule3, rule4]

    fchain(PROPS, RULES)

if __name__ == "__main__":
    main()
