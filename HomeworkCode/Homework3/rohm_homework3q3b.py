# Aidan Rohm
# Forward chaining of Horn Clauses
# Corresponds to Homework 3 Question 3 --> Chat GPT Code
# DUE: March 5th, 2026

# Chat GPT was given the exact question from the homework as a prompt and it created this solution:

from collections import deque
from typing import List, Tuple

# A rule is: [LHS_indices_list, RHS_index]
Rule = Tuple[List[int], int]

def fchain(PROPS: List[str], RULES: List[Rule]) -> List[bool]:
    """
    Forward chaining for positive (definite) Horn clauses.

    PROPS: list of proposition names, e.g. ['A','B','Q','R']
    RULES: list of rules, where each rule is ([lhs_indices...], rhs_index)
           Facts are rules with empty LHS, e.g. ([], 0) means A is true.
    Returns: VALS (list of booleans) where VALS[i] is truth of PROPS[i].
    """
    n = len(PROPS)

    # VALS[i] is truth of proposition i (initially all false)
    VALS = [False] * n

    # CTRS[r] = number of unsatisfied premises remaining for rule r
    CTRS = [len(lhs) for (lhs, rhs) in RULES]

    # For each proposition p, store which rules have p in their LHS
    occurs_in = [[] for _ in range(n)]
    for r_idx, (lhs, rhs) in enumerate(RULES):
        for p in lhs:
            occurs_in[p].append(r_idx)

    # Agenda holds proposition indices that just became true
    agenda = deque()

    # Initialize with all facts (empty LHS): they can fire immediately
    for r_idx, (lhs, rhs) in enumerate(RULES):
        if CTRS[r_idx] == 0 and not VALS[rhs]:
            VALS[rhs] = True
            agenda.append(rhs)

    # Main forward-chaining loop
    while agenda:
        the_proposition_index = agenda.popleft()

        # Any rule that needs this proposition gets closer to being satisfied
        for r_idx in occurs_in[the_proposition_index]:
            if CTRS[r_idx] > 0:
                CTRS[r_idx] -= 1
                if CTRS[r_idx] == 0:
                    _, rhs = RULES[r_idx]
                    if not VALS[rhs]:
                        VALS[rhs] = True
                        agenda.append(rhs)

    # Print in the requested format
    print(PROPS)
    print(['T' if v else 'F' for v in VALS])

    return VALS


# ------------------- Example from the prompt -------------------
if __name__ == "__main__":
    PROPS = ['A', 'B', 'Q', 'R']

    rule1 = ([0, 1], 2)   # A and B => Q
    rule2 = ([2], 0)      # Q => A
    rule3 = ([], 0)       # A
    rule4 = ([], 1)       # B

    RULES = [rule1, rule2, rule3, rule4]

    fchain(PROPS, RULES)