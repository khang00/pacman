"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from __future__ import division
from collections import defaultdict
from game import Directions
from util import PriorityQueue
from util import manhattanDistance
import util
n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


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


def depthFirstSearch(problem):
    return [Directions.EAST, Directions.EAST]


def breadthFirstSearch(problem):
    return [Directions.EAST, Directions.EAST]


def uniformCostSearch(problem):
    frontier = PriorityQueue()
    frontier.push((problem.getStartState(), Directions.STOP, 0), 0)
    expanded = []
    current_cost = 0
    path = defaultdict(list)
    while not frontier.isEmpty():
        current_state = frontier.pop()
        expanded.append(current_state)
        current_cost = current_cost + current_state[2]
        if problem.isGoalState(current_state[0]):
            return compute_actions(path, current_state)
        successors = problem.getSuccessors(current_state[0])
        for state in successors:
            if state not in expanded and not frontier.does_contain(state):
                frontier.push(state, current_cost + state[2])
                path[state] = current_state
    return path


def compute_actions(path, goal):
    if not path:
        return [Directions.STOP]
    actions = []
    i = goal
    actions.insert(0, goal[1])
    while path[i][1] != Directions.STOP:
        actions.insert(0, path[i][1])
        i = path[i]
    return actions


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic):
    frontier = PriorityQueue()
    expanded = []
    path = defaultdict(list)
    current_cost = 0
    start_state = (problem.getStartState(), Directions.STOP, 0)
    frontier.push(start_state, heuristic(start_state, problem) + current_cost)
    while not frontier.isEmpty():
        current_state = frontier.pop()
        expanded.append(current_state)
        current_cost = current_cost + current_state[2]
        if heuristic(current_state, problem) == 0:
            return compute_actions(path, current_state)
        successors = problem.getSuccessors(current_state[0])
        for state in successors:
            if state not in expanded and not frontier.does_contain(state):
                frontier.push(state, heuristic(state, problem) + current_cost)
                path[state] = current_state
    return [Directions.STOP]

def aStarSearchSwitching(problem, heuristic):
    ghostEvadePath = aStarSearch(problem, ghostHeuristic)
    if ghostEvadePath != [Directions.STOP]:
        print 'ghost evade mode'
        return ghostEvadePath
    else:
        print 'find food mode'
        return aStarSearch(problem, heuristic)

def ghostHeuristic(state, problem):
    heuristic = manhattanDistance(state[0][0], problem.ghostPositions[0])
    for ghostPos in problem.ghostPositions:
        heuristic = min(heuristic, manhattanDistance(state[0][0], ghostPos))

    detect_range = 3
    if heuristic == 0:
        return 9999999

    if heuristic > detect_range:
        return 0
    else:
        return 1 / heuristic

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
