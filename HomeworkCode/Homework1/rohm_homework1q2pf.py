# Aidan Rohm
# Uniform Cost Search program for classic jug problem
# Corresponds to question 2 in Homework 1
# DUE: February 5th, 2026

import time  # Added import for timing
from queue import PriorityQueue
from typing import Tuple, List, Optional, Dict

class WaterJugProblem:
    def __init__(self):
        self.capacities = (12, 8, 3)
        self.start_state = (0, 0, 0)
        self.visited = set()

    def is_goal_state(self, state: Tuple[int, int, int]) -> bool:
        """Check if any jug contains exactly 1 gallon -> this is the goal."""
        return state[0] == 1 or state[1] == 1 or state[2] == 1
    
    def get_actions(self) -> List[str]:
        """Return all possible actions."""
        return [
            'Fill1', 'Fill2', 'Fill3',
            'Empty1', 'Empty2', 'Empty3',
            'Pour1->2', 'Pour1->3', 'Pour2->1',
            'Pour2->3', 'Pour3->1', 'Pour3->2'
        ]

    def apply_action(self, state: Tuple[int, int, int], action: str) -> Tuple[int, int, int]:
        """Apply an action to a state and return the resulting state from this action"""
        
        x1, x2, x3 = state

        # Fill states
        if action == 'Fill1':
            return (self.capacities[0], x2, x3)
        elif action == 'Fill2':
            return (x1, self.capacities[1], x3)
        elif action == 'Fill3':
            return (x1, x2, self.capacities[2])
        # Empty states
        elif action == 'Empty1':
            return (0, x2, x3)
        elif action == 'Empty2':
            return (x1, 0, x3)
        elif action == 'Empty3':
            return (x1, x2, 0)
        # Pour states
        elif action == 'Pour1->2':
            t = min(x1, self.capacities[1] - x2)
            return (x1 - t, x2 + t, x3)
        elif action == 'Pour1->3':
            t = min(x1, self.capacities[2] - x3)
            return (x1 - t, x2, x3 + t)
        elif action == 'Pour2->1':
            t = min(x2, self.capacities[0] - x1)
            return (x1 + t, x2 - t, x3)
        elif action == 'Pour2->3':
            t = min(x2, self.capacities[2] - x3)
            return (x1, x2 - t, x3 + t)
        elif action == 'Pour3->1':
            t = min(x3, self.capacities[0] - x1)
            return (x1 + t, x2, x3 - t)
        elif action == 'Pour3->2':
            t = min(x3, self.capacities[1] - x2)
            return (x1, x2 + t, x3 - t)

        return state  # This should never happen because you can always do an action and be brought to a state

    def get_successors(self, state: Tuple[int, int, int]) -> List[Tuple[Tuple[int, int, int], str, int]]:
        """Get all the valid successor states from a given state."""
        
        successors = []

        for action in self.get_actions():
            new_state = self.apply_action(state, action)
            # Only add if the state actually changed
            if new_state != state:
                # Uniform cost where each action costs 1
                successors.append((new_state, action, 1))

        return successors

    def uniform_cost_search(self) -> Optional[Tuple[List[Tuple[int, int, int]], List[str], int]]:
        """
        Performing Uniform Cost Search to find the shortest sequence of actions to reach the goal state
        
        Returns: (path_states, path_actions, total_cost) if a solution is found, if no solution is found it returns None
        """

        # Priority queue stores the (cumulative_cost, state, path_states, path_actions)
        frontier = PriorityQueue()
        frontier.put((0, self.start_state, [self.start_state], []))

        # Keeping track of the visited states and their corresponding costs
        visited_costs = {self.start_state: 0}

        while not frontier.empty():
            current_cost, current_state, path_states, path_actions = frontier.get()

            # Check if we have already visited this state with a lower cost
            if current_state in visited_costs and visited_costs[current_state] < current_cost:
                continue

            # Mark the state as visited with this cost
            visited_costs[current_state] = current_cost
            
            # Add to visited set for statistics
            self.visited.add(current_state)

            # Check to see if this is one of the goal states
            if self.is_goal_state(current_state):
                return path_states, path_actions, current_cost

            # Generate the successors
            for next_state, action, action_cost in self.get_successors(current_state):
                new_cost = current_cost + action_cost

                # Only add to the frontier if
                #   (1). We have not visited this state before OR
                #   (2). We just found a cheaper path to this state
                if next_state not in visited_costs or new_cost < visited_costs[next_state]:
                    visited_costs[next_state] = new_cost
                    new_path_states = path_states + [next_state]
                    new_path_actions = path_actions + [action]
                    frontier.put((new_cost, next_state, new_path_states, new_path_actions))
        
        return None

    def print_solution(self, solution: Tuple[List[Tuple[int, int, int]], List[str], int]):
        """Printing the solution in a readable format."""
        path_states, path_actions, total_cost = solution

        print("Water Jug Problem Solution using Uniform Cost Search")
        print("*" * 60)
        print(f"Start State: {self.start_state}")
        print(f"Jug Capacities: {self.capacities}")
        print(f"Total Actions: {total_cost}")
        print("*" * 60)

        print("\nStep by step solution:")
        print(f"Step 0: state {self.start_state} (Start)")

        for i, (state, action) in enumerate(zip(path_states[1:], path_actions), 1):
            print(f"Step {i}: {action:10} -> State {state}")

        print("\n" + "=" * 60)
        print(f"GOAL REACHED! Jug states: {path_states[-1]}")
        print(f"One gallon is in {'jug 1' if path_states[-1][0] == 1 else 'jug 2' if path_states[-1][1] == 1 else 'jug 3'}")
        print(f"Total cost: {total_cost} actions")

def format_time(seconds: float) -> str:
    """Format time in a human-readable way."""
    if seconds < 0.001:  # Less than 1 millisecond
        return f"{seconds * 1_000_000:.2f} μs (microseconds)"
    elif seconds < 1:  # Less than 1 second
        return f"{seconds * 1000:.2f} ms (milliseconds)"
    elif seconds < 60:  # Less than 1 minute
        return f"{seconds:.3f} seconds"
    else:
        minutes = int(seconds // 60)
        remaining_seconds = seconds % 60
        return f"{minutes} minutes {remaining_seconds:.2f} seconds"

def main():
    """Main function to run the water jug problem and solution generation."""

    print("Solving Water Jug Problem with Uniform Cost Search")
    print("Jug capacities: 12, 8, 3 gallons respectively")
    print("Goal: Get exactly 1 gallon in any of the jugs")
    print("\nSearching for solution...\n")
    
    # Start stopwatch
    start_time = time.time()
    
    problem = WaterJugProblem()
    solution = problem.uniform_cost_search()
    
    # End stopwatch
    end_time = time.time()
    elapsed_time = end_time - start_time

    if solution:
        problem.print_solution(solution)

        # Additional statistics
        print("\n" + "=" * 60)
        print("ALGORITHM PERFORMANCE STATISTICS:")
        print(f"Search execution time: {format_time(elapsed_time)}")
        print(f"Total states visited during search: {len(problem.visited)}")
        print(f"Solution path length: {len(solution[0]) - 1} steps")
        print(f"States explored per second: {len(problem.visited) / elapsed_time:.0f} states/sec")
    else:  # Failsafe though it may never print
        print("No solution found!")
        print(f"Search execution time: {format_time(elapsed_time)}")

if __name__ == "__main__":
    main()