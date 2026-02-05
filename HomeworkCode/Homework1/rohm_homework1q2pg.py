# Aidan Rohm
# Uniform Cost Search program for classic jug problem
# Corresponds to question 2 in Homework 1
# DUE: February 5th, 2026

# This code demonstrates Chat GPT's response to the following prompt:
# There are three jugs. the first has a capacity of 12 gallons, the next has a capacity of 8 gallons, 
# and the last has a capacity of 3 gallons. There are three actions you can take: fill a gallon from a 
# tap all the way to its capacity, empty a jug to nothing, or empty as much as you can from one jug to another. 
# Write a python program to complete a uniform cost search. Include some basic analysis in the code such search 
# execution time, total states visited, solution path length and states explored per second.

import time
import heapq

# Jug capacities
CAPACITIES = (12, 8, 3)

START_STATE = (0, 0, 0)

def is_goal(state):
    """Goal: any jug contains exactly 1 gallon."""
    return 1 in state

def get_successors(state):
    """
    Generate all valid successor states from the current state.
    Actions:
      1) Fill a jug to capacity
      2) Empty a jug
      3) Pour from one jug to another
    """
    successors = []
    a, b, c = state
    jugs = [a, b, c]

    # --- Fill actions ---
    for i in range(3):
        if jugs[i] < CAPACITIES[i]:
            new_state = list(jugs)
            new_state[i] = CAPACITIES[i]
            successors.append(tuple(new_state))

    # --- Empty actions ---
    for i in range(3):
        if jugs[i] > 0:
            new_state = list(jugs)
            new_state[i] = 0
            successors.append(tuple(new_state))

    # --- Pour actions ---
    for i in range(3):
        for j in range(3):
            if i != j and jugs[i] > 0 and jugs[j] < CAPACITIES[j]:
                new_state = list(jugs)
                amount = min(jugs[i], CAPACITIES[j] - jugs[j])
                new_state[i] -= amount
                new_state[j] += amount
                successors.append(tuple(new_state))

    return successors

def uniform_cost_search():
    start_time = time.time()

    # Priority queue: (cost, state, path)
    frontier = []
    heapq.heappush(frontier, (0, START_STATE, [START_STATE]))

    visited = set()
    states_explored = 0

    while frontier:
        cost, state, path = heapq.heappop(frontier)

        if state in visited:
            continue

        visited.add(state)
        states_explored += 1

        if is_goal(state):
            end_time = time.time()
            return {
                "solution_path": path,
                "path_length": len(path) - 1,
                "states_visited": len(visited),
                "states_explored": states_explored,
                "execution_time": end_time - start_time
            }

        for successor in get_successors(state):
            if successor not in visited:
                heapq.heappush(
                    frontier,
                    (cost + 1, successor, path + [successor])
                )

    return None

# ------------------- Run the search -------------------

result = uniform_cost_search()

if result:
    exec_time = result["execution_time"]
    states_per_sec = result["states_explored"] / exec_time if exec_time > 0 else 0

    print("=== Uniform Cost Search Results ===")
    print(f"Solution path length: {result['path_length']}")
    print(f"Total states visited: {result['states_visited']}")
    print(f"Execution time (s): {exec_time:.6f}")
    print(f"States explored per second: {states_per_sec:.2f}")
    print("\nSolution path:")
    for state in result["solution_path"]:
        print(state)
else:
    print("No solution found.")
