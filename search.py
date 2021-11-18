from typing import Any, Iterable, Set
from problem import HeuristicFunction, Problem, S, A, Solution
from collections import deque
from queue import PriorityQueue
from helpers import utils

#TODO: Import any modules or write any helper functions you want to use

# All search functions take a problem and a state
# If it is an informed search function, it will also receive a heuristic function
# S and A are used for generic typing where S represents the state type and A represents the action type

# All the search functions should return one of two possible type:
# 1. A list of actions which represent the path from the initial state to the final state
# 2. None if there is no solution
def GetPath(parent, initial_state, state_Action):
    goal, goal_action = state_Action
                    
    path = []

    # Move up in the path dictionary until we reach initial_state
    while goal_action != initial_state:

        # Append to the left each action while going up in the dictionary
        path.insert(0, goal_action)
        goal, goal_action = parent[goal]

    # Return list of actions which represent the path from the initial state to the final state
    return path

def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE

    # Check if initial node is goal
    if problem.is_goal(initial_state): 
        return [initial_state]

    # FIFO Queue with initial_state as first element
    frontier = deque()
    frontier.append(initial_state)

    # Empty Set
    explored: Set = set()

    # Path Dictionary to store actions from start to goal
    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    while frontier:

        # Pop shalowest node in frontier
        state: S = frontier.popleft()

        # Add node state to explored
        explored.add(state)

        # Getting successors of shalowest node
        for action in problem.get_actions(state):
            child: S = problem.get_successor(state, action)
            
            # Check Child not already visited and not in expanded frontier nodes
            if child not in frontier and child not in explored:

                # Check if child node is goal before expansion
                if problem.is_goal(child):
                    return GetPath(parent, initial_state, [state, action])

                # Add child to frontier
                frontier.append(child)

                # Add state, action to the path dictionary
                parent[child] = [state, action]
    return None

def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    
    # Empty Set
    explored: Set = set()

    frontier = deque()
    frontier.append(initial_state)

    # Path Dictionary to store actions from start to goal
    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    def dfs(node: list([S, A]), path: list):
        state, goal_action = node
        path.append(goal_action)
        
        explored.add(state)
        
        # Check if goal
        if problem.is_goal(state):
            return path

        # Getting successors of left most node   
        for action in problem.get_actions(state):
            child: S = problem.get_successor(state, action)

            # If not explored visit its subtrees from left to right
            if child not in explored:
                result = dfs([child, action], path)
                if result is not None:
                    return result

        path.pop()
        # Reached right most node in the subtree and goal not found
        return None
    path = dfs([initial_state, initial_state], [])

    # Return path of actions from start to goal or none if not found
    if path: return path[1:]
    return None

'''
Class Node for Uniform Cost, Astar and Best first search

Overloads comparison operator to compare by less cost( gn + hn ) then by time entered t
'''
class Node(object):
    def __init__(self, state: S, gn: float, hn: float, time: int):
        self.state = state
        self.gn = gn
        self.hn = hn
        self.t = time
    
    def __lt__(self, other):
        return (self.gn + self.hn) < (other.gn + other.hn) or ( (self.gn + self.hn) == (other.gn + other.hn)  and (self.t < other.t) )

def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE

    # Priority Queue to compare by path cost (gn) then by time entered t
    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0.0, 0.0, 0))

    # Empty Set
    explored: Set = set()

    # Path Dictionary to store actions from start to goal
    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():

        # Get node with lowest cost or by time if costs are equal
        node: Node = frontier.get()

        state: S = node.state
        child_gn: float  = node.gn
        # Check if goal     
        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])

        # Add node state to explored
        explored.add(state)

        # Getting successors of cheapest node   
        actions = problem.get_actions(state)
        for action in actions:
            child: S = problem.get_successor(state, action)

            '''
            Loop on the frontier to check child not in
            If child in frontier store its node to check if it has a higher cost and replace it
            '''
            in_frontier = False
            higher_cost_state = None
            for vertex in frontier.queue:
                if child == vertex.state:
                    in_frontier = True
                    higher_cost_state = vertex
                    break

            # Check Child not already visited and not in expanded frontier nodes
            if child not in explored and not in_frontier:
                # Increment time entered
                time_entered += 1
                
                # Add state, action to the path dictionary
                frontier.put(Node(child, child_gn + problem.get_cost(state, action), 0.0, time_entered))
               
                # Add child to frontier
                parent[child] = [state, action]

            # If Child already in frontier but with higher cost
            # Modify Child cost
            elif in_frontier and higher_cost_state.gn > child_gn + problem.get_cost(state, action):
                # Increment time entered
                time_entered += 1

                # Modify child cost, time and path action in the dictionary
                higher_cost_state.gn = child_gn + problem.get_cost(state, action)
                higher_cost_state.t = time_entered
                parent[child] = [state, action]
    return None

def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE

    # Priority Queue to compare by total path cost (gn + hn) then by time entered t
    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0.0, 0.0, 0))
    
    # Empty Set
    explored: Set = set()

    # Path Dictionary to store actions from start to goal
    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():

        # Get node with lowest total cost or by time if costs are equal
        node: Node = frontier.get()

        state: S = node.state   
        # Check if goal     
        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])

        # Add node state to explored
        explored.add(state)

        # Getting successors of cheapest node        
        actions = problem.get_actions(state)
        for action in actions:
            child = problem.get_successor(state, action)
            
            # Child Cost and heuristic
            child_gn = node.gn + problem.get_cost(state, action)           
            child_hn = heuristic(problem, child)

            '''
            Loop on the frontier to check child not in
            If child in frontier store its node to check if it has a higher cost and replace it
            '''
            in_frontier = False
            higher_cost_state = None
            for vertex in frontier.queue:
                if child == vertex.state:
                    in_frontier = True
                    higher_cost_state = vertex
                    break

            # Check Child not already visited and not in expanded frontier nodes
            if child not in explored and not in_frontier: 

                # Increment time entered
                time_entered += 1
                
                # Add state, action to the path dictionary
                parent[child] = [state, action]

                # Add child to frontier
                frontier.put(Node(child, child_gn, child_hn, time_entered))

            # If Child already in frontier but with higher cost
            # Modify Child cost
            elif in_frontier and higher_cost_state.gn > child_gn:
                # Increment time entered
                time_entered += 1

                # Modify child cost, time and path action in the dictionary
                parent[child] = [state, action]
                node.gn = child_gn
                node.t = time_entered
    return None

def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE

    # Priority Queue to compare by heuristic (hn) then by time entered t
    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0.0, 0.0, 0))
    
    # Empty Set
    explored: Set = set()

    # Path Dictionary to store actions from start to goal
    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():

        # Get node with lowest heuristic by time if costs are equal
        node: Node = frontier.get()

        state: S = node.state   
        # Check if goal     
        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])

        # Add node state to explored
        explored.add(state)

        # Getting successors of cheapest node 
        for action in problem.get_actions(state):
            child = problem.get_successor(state, action)
            
            # Loop on the frontier to check child not in
            in_frontier = False
            for vertex in frontier.queue:
                if child == vertex.state :
                    in_frontier = True
                    break

            if child not in explored and in_frontier == False: 
                # Increment time entered
                time_entered += 1

                # Add state, action to the path dictionary
                parent[child] = [state, action]

                # Add child to frontier
                # Here we're only interested in heuristic so set path cost = 0
                frontier.put(Node(child, 0.0, heuristic(problem, child),  time_entered))
                
    return None
    