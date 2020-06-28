"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from __future__ import division
from collections import defaultdict
from util import Stack
from time import sleep

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
    frontier = util.Stack()
    expanded = []
    path = []
    frontier.push((problem.getStartState(), []))
    while not frontier.isEmpty():
        node, path = frontier.pop()
        if problem.isGoalState(node):
            return path
        else:
            if node not in expanded:
                expanded.append(node)
                successors = problem.getSuccessors(node)
                for successor in successors:
                    child = successor[0]
                    child_path = successor[1]
                    full_path = path + [child_path]
                    frontier.push((child, full_path))
    return path


def breadthFirstSearch(problem):
    frontier = util.Queue()
    expanded = []
    path = []
    frontier.push((problem.getStartState(), path))
    while not frontier.isEmpty():
        node, path = frontier.pop()
        if problem.isGoalState(node):
            return path
        else:
            if node not in expanded:
                expanded.append(node)
                successors = problem.getSuccessors(node)
                for successor in successors:
                    child = successor[0]
                    child_path = successor[1]
                    full_path = path + [child_path]
                    frontier.push((child, full_path))
    return path


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
                # For evading of ghosts
                if is_next_move_die(state[0][0], problem.ghostPositions):
                    frontier.push(state, current_cost + state[2] + 9999999)
                else:
                    frontier.push(state, current_cost + state[2])
                path[state] = current_state
    return path


def is_next_move_die(next_position, ghost_positions):
    for position in ghost_positions:
        x, y = position
        if next_position == (x + 1, y) or next_position == (x - 1, y):
            return True
        if next_position == (x, y - 1) or next_position == (x, y + 1):
            return True


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


def gradientDescent(problem, heuristic):
    frontier = PriorityQueue()
    expanded = []
    path = defaultdict(list)
    current_cost = 0
    start_state = (problem.getStartState(), Directions.STOP, 0)
    frontier.push(start_state, heuristic(start_state, problem) + current_cost)
    current_state = frontier.pop()
    expanded.append(current_state)
    current_cost = current_cost + current_state[2]
    successors = problem.getSuccessors(current_state[0])
    for state in successors:
        if state not in expanded and not frontier.does_contain(state):
            frontier.push(state, heuristic(state, problem) + current_cost)
            path[state] = current_state
    return compute_actions(path, frontier.pop())

def ghostHeuristic(state, problem):
    ghostHeuristics = problem.heuristicInfo.getDijkstraDistance(state[0][0], problem.ghostPositions[0])
    for ghostPos in problem.ghostPositions:
        ghostHeuristics = min(ghostHeuristics, problem.heuristicInfo.getDijkstraDistance(state[0][0], ghostPos))

    detect_range = 2
    if ghostHeuristics == 0:
        return 9999999

    if ghostHeuristics > detect_range:
        return 0
    else:
        return 1 / ghostHeuristics

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
