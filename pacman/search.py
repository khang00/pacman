"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
from util import Queue
from util import PriorityQueue
from collections import defaultdict
from util import Stack
from time import sleep

from game import Directions

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
                # For evading of ghosts
                if is_next_move_die(state[0][0], problem.ghostPositions):
                    frontier.push(state, current_cost + state[2] + 999999)
                else:
                    frontier.push(state, current_cost + state[2])
                path[state] = current_state
    return path

def is_next_move_die(next_position, ghost_positions):
    for postion in ghost_positions:
        x, y = postion
        if next_position == (x + 1, y) or next_position == (x - 1, y):
            return True
        if next_position == (x, y - 1) or next_position == (x, y + 1):
            return True

def compute_actions(path, goal):
    actions = []
    i = goal
    actions.insert(0, goal[1])
    while path[i][1] != Directions.STOP:
        actions.insert(0, path[i][1])
        i = path[i]
    return [actions[0]]


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    return [Directions.EAST, Directions.EAST]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch