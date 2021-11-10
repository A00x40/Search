from collections import deque
from dungeon import DungeonProblem, DungeonState
from mathutils import Direction, Point, euclidean_distance, manhattan_distance
from helpers import utils
from problem import Problem

# This heuristic returns the distance between the player and the exit as an estimate for the path cost
# While it is consistent, it does a bad job at estimating the actual cost thus the search will explore a lot of nodes before finding a goal
def weak_heuristic(problem: DungeonProblem, state: DungeonState):
    return euclidean_distance(state.player, problem.layout.exit)

#TODO: Import any modules and write any functions you want to use

def H_BFS(problem, initial_state, goal_state, state_dist):
    
    if initial_state == goal_state:
        state_dist[(initial_state, goal_state)] = 0
        return 

    frontier = deque()
    frontier.append(initial_state)

    explored = set()

    parent = dict()
    parent[initial_state] = None

    while frontier:
        state = frontier.popleft()
        explored.add(state)

        for action in problem.get_actions(state):
            child = problem.get_successor(state, action)
            
            if child not in frontier and child not in explored:
                if child == goal_state:
                    #
                    goal = child
                    
                    path = []
                    while goal != None:
                        path.insert(0, goal)
                        goal = parent[goal]

                    for i in range(len(path)):
                        state_dist[(path[i], goal_state)] = len(path[i:]) - 1

                    return 

                frontier.append(child)
                parent[child] = state
                
def strong_heuristic(problem: DungeonProblem, state: DungeonState) -> float:
    #TODO: ADD YOUR CODE HERE
    #IMPORTANT: DO NOT USE "problem.is_goal" HERE.
    # Calling it here will mess up the tracking of the explored nodes count
    # which is considered the number of is_goal calls during the search
    #NOTE: you can use problem.cache() to get a dictionary in which you can store information that will persist between calls of this function
    # This could be useful if you want to store the results heavy computations that can be cached and used across multiple calls of this function
    state_dist = problem.cache()

    for coin in state.remaining_coins:
        if (state.player, coin) not in state_dist:
            H_BFS(problem, state, coin, state_dist)
    
    dist = manhattan_distance(state.player, problem.layout.exit)
    for coin in state.remaining_coins:
        dist = max(dist, state_dist[(state.player, coin)])

    return dist
    