# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """

    open = util.Stack()     # open is a stack for DFS
    start = ((problem.getStartState(), None, 0), None)
    open.push(start)
    closed = util.Stack()
    visited = []
    actions = util.Stack()

    while not open.isEmpty():
        curr_state = open.pop()  # (x,y)
        closed.push(curr_state)
        visited.append(curr_state[0][0])
        if problem.isGoalState(curr_state[0][0]):
            break
        for succ in problem.getSuccessors(curr_state[0][0]):
            if not succ[0] in visited:
                open.push((succ,curr_state[0][0]))  # Push the newly expanded elements into open list
            else:
                pass
    # Finding path from the goal node
    path = []
    state = closed.pop()
    while not state[1] == None:
        for states in closed.list:
            if state[1] == states[0][0]:
                actions.push(state[0][1])
                state = states
    while not actions.isEmpty():
        path.append(actions.pop())
    return path


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    open = util.Queue()  # open is a Queue for BFS
    start = ((problem.getStartState(), None, 0), None)  #((x,y), direction, cost) ,parent
    open.push(start)
    closed = util.Stack()
    visited = []
    actions = util.Stack()

    while (open.isEmpty() == False):
        curr_state = open.pop()  # (x,y)
        closed.push(curr_state)
        # Full cycle check

        if curr_state[0][0] in visited:
            continue
        else:
            visited.append(curr_state[0][0])

        if problem.isGoalState(curr_state[0][0]):
            break

        for succ in problem.getSuccessors(curr_state[0][0]):
            if not succ[0] in visited:
                open.push((succ, curr_state[0][0]))  # Push the newly expanded elements&it's parent into open list

    # Finding path from the goal node
    path = []
    state = closed.pop()
    while (state[1] != None):
        for states in closed.list:
            if (state[1] == states[0][0]):
                actions.push(state[0][1])
                state = states
            else:
                continue
    while actions.isEmpty() == False:
        path.append(actions.pop())
    return path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    open = util.PriorityQueue()  # open is a Queue for BFS
    start = ((problem.getStartState(), None, 0), None)  #((x,y), direction, cost) ,parent
    cost = 0
    open.push(start,cost)      # state, cost(as priority)
    closed = util.Stack()
    visited = []
    actions = util.Stack()

    while not open.isEmpty():
        curr_state = open.pop()  # (x,y)
        cost = curr_state[0][2]
        closed.push(curr_state)
        # Full cycle check
        if curr_state[0][0] in visited:
            continue
        else:
            visited.append(curr_state[0][0])
        if problem.isGoalState(curr_state[0][0]):
            break
        for succ in problem.getSuccessors(curr_state[0][0]):
            if not succ[0] in visited:
                new_succ = (succ[0],succ[1],succ[2]+cost)
                open.push((new_succ, curr_state[0][0]), succ[2]+cost)  # Push the newly expanded elements&it's parent into open list
    # Finding path from the goal node
    path = []
    state = closed.pop()
    while not state[1] == None:
        for states in closed.list:
            if state[1] == states[0][0]:
                actions.push(state[0][1])
                state = states
            else:
                continue
    # Adjust order of actions into correct order
    while not actions.isEmpty():
        path.append(actions.pop())
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    open = util.PriorityQueue()  # open is a Queue for BFS

    cost = 0
    h = heuristic(problem.getStartState(), problem)
    eval = cost + h
    start = ((problem.getStartState(), None, cost, h), None)  # ((x,y), direction, cost, heuristic) ,parent
    open.push(start, eval)      # state, eval (as priority)

    closed = util.Stack()
    visited = []
    actions = util.Stack()

    while not open.isEmpty():
        curr_state = open.pop()  # (x,y)
        cost = curr_state[0][2]
        closed.push(curr_state)

        # Full cycle check
        if curr_state[0][0] in visited:
            continue
        else:
            visited.append(curr_state[0][0])

        if problem.isGoalState(curr_state[0][0]):
            break

        for succ in problem.getSuccessors(curr_state[0][0]):
            if not succ[0] in visited:
                h = heuristic(succ[0], problem)
                eval = succ[2]+cost + h
                new_succ = (succ[0],succ[1],succ[2]+cost,h)
                open.push((new_succ, curr_state[0][0]), eval)  # Push the newly expanded elements&it's parent into open list

    # Finding path from the goal node
    path = []
    state = closed.pop()
    while not state[1] == None:
        for states in closed.list:
            if state[1] == states[0][0]:
                actions.push(state[0][1])
                state = states
            else:
                continue
    # Adjust order of actions into correct order
    while not actions.isEmpty():
        path.append(actions.pop())
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
