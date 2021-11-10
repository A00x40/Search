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
    while goal_action != initial_state:
        
        path.insert(0, goal_action)
        goal, goal_action = parent[goal]
    return path

def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    if problem.is_goal(initial_state): 
        return [initial_state]

    frontier = deque()
    frontier.append(initial_state)

    explored: Set = set()

    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    while frontier:
        state: S = frontier.popleft()
        explored.add(state)

        for action in problem.get_actions(state):
            child: S = problem.get_successor(state, action)
            
            if child not in frontier and child not in explored:
                if problem.is_goal(child):
                    return GetPath(parent, initial_state, [state, action])

                frontier.append(child)
                parent[child] = [state, action]
    return None

def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    explored: Set = set()

    frontier = deque()
    frontier.append(initial_state)

    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    def dfs(node: list([S, A]), path: list):
        state, goal_action = node
        path.append(goal_action)
        
        explored.add(state)

        if problem.is_goal(state):
            return path

        for action in problem.get_actions(state):
            child: S = problem.get_successor(state, action)
            if child not in explored:
                result = dfs([child, action], path)
                if result is not None:
                    return result
        path.pop()
        return None
    path = dfs([initial_state, initial_state], [])
    if path: return path[1:]
    return None

def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    class Node(object):
        def __init__(self, state: S, cost: float, time: int):
            self.state = state
            self.cost = cost
            self.t = time
        def __lt__(self, other):
            return (self.cost < other.cost) or (self.cost == other.cost and self.t < other.t)

    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0.0, 0))

    explored: Set = set()

    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():
        node: Node = frontier.get()

        state: S = node.state
        cost: float  = node.cost

        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])

        explored.add(state) 

        actions = problem.get_actions(state)
        for action in actions:
            child: S = problem.get_successor(state, action)

            in_frontier = False
            higher_cost_state = None
            for vertex in frontier.queue:
                if child == vertex.state:
                    in_frontier = True
                    higher_cost_state = vertex
                    break

            if child not in explored and not in_frontier:
                time_entered += 1
                frontier.put(Node(child, cost + problem.get_cost(state, action), time_entered))
                parent[child] = [state, action]

            elif in_frontier and higher_cost_state.cost > cost + problem.get_cost(state, action):
                time_entered += 1
                higher_cost_state.cost = cost + problem.get_cost(state, action)
                higher_cost_state.t = time_entered
                parent[child] = [state, action]
    return None

def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE
    class Node(object):
        def __init__(self, state: S, gn: float, time: int):
            self.state = state
            self.gn = gn
            self.hn = heuristic(problem, state)
            self.t = time
        
        def __lt__(self, other):
            return (self.gn + self.hn) < (other.gn + other.hn) or ( (self.gn + self.hn) == (other.gn + other.hn)  and (self.t < other.t) )

    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0.0, 0))
    
    explored: Set = set()

    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():
        node: Node = frontier.get()

        state: S = node.state
        
        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])

        explored.add(state)
        
        actions = problem.get_actions(state)
        for action in actions:
            child = problem.get_successor(state, action)
            
            child_gn = node.gn + problem.get_cost(state, action)           

            in_frontier = False
            higher_cost_state = None
            for vertex in frontier.queue:
                if child == vertex.state:
                    in_frontier = True
                    higher_cost_state = vertex
                    break

            if child not in explored and not in_frontier: 
                time_entered += 1
                parent[child] = [state, action]
                frontier.put(Node(child, child_gn, time_entered))

            elif in_frontier and higher_cost_state.gn > child_gn:
                time_entered += 1
                parent[child] = [state, action]
                node.gn = child_gn
                node.t = time_entered
    return None

def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE
    class Node(object):
        def __init__(self, state: S, time: int):
            self.state = state
            self.hn = heuristic(problem, state)
            self.t = time
        
        def __lt__(self, other):
            return (self.hn < other.hn) or ( (self.hn == other.hn) and (self.t < other.t) )

    frontier = PriorityQueue()
    frontier.put(Node(initial_state, 0))
    
    explored: Set = set()

    parent = dict()
    parent[initial_state] = [initial_state, initial_state]

    time_entered = 1
    while not frontier.empty():
        node: Node = frontier.get()

        state: S = node.state
        
        if problem.is_goal(state):
            return GetPath(parent, initial_state, parent[state])


        explored.add(state)

        for action in problem.get_actions(state):
            child = problem.get_successor(state, action)
            
            in_frontier = False
            for vertex in frontier.queue:
                if child == vertex.state :
                    in_frontier = True
                    break

            if child not in explored and in_frontier == False: 
                time_entered += 1
                parent[child] = [state, action]
                frontier.put(Node(child, time_entered))
                
    return None
    