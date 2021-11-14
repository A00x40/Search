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

def strong_heuristic(problem: DungeonProblem, state: DungeonState) -> float:
    #TODO: ADD YOUR CODE HERE
    #IMPORTANT: DO NOT USE "problem.is_goal" HERE.
    # Calling it here will mess up the tracking of the explored nodes count
    # which is considered the number of is_goal calls during the search
    #NOTE: you can use problem.cache() to get a dictionary in which you can store information that will persist between calls of this function
    # This could be useful if you want to store the results heavy computations that can be cached and used across multiple calls of this function
    
    
    exit_dist = manhattan_distance(state.player, problem.layout.exit)

    max_coin_dist = 0
    furthest_coin = None

    max_coin_dist2 = 0
    furthest_coin2 = None

    # Get furthest coin
    # And the Distance from state to furthest coin then to exit
    for coin in state.remaining_coins:
        coin_dist = manhattan_distance(state.player, coin) + manhattan_distance(coin, problem.layout.exit)
        if coin_dist > max_coin_dist:
            max_coin_dist = coin_dist
            furthest_coin = coin

    # Get the second furthest coin
    for coin in state.remaining_coins:
        coin_dist = manhattan_distance(state.player, coin) + manhattan_distance(coin, problem.layout.exit)
        if coin_dist > max_coin_dist2:
            max_coin_dist2 = coin_dist
            furthest_coin2 = coin

    '''
    If 2 or more remaining
    Hueristic = distance(state, second furthest coin) 

        + distance(second furthest coin, furthest coin)

        + distance(furthest coin, dungeon exit)
    '''
    if len(state.remaining_coins) > 1:
        return manhattan_distance(state.player, furthest_coin2) + manhattan_distance(furthest_coin2, furthest_coin) + manhattan_distance(furthest_coin, problem.layout.exit)

    # One remaining coin
    # Distance from state to furthest coin then to exit
    elif len(state.remaining_coins) == 1:
        return max_coin_dist

    # No coins
    # Distance from state to exit
    return exit_dist