from __future__ import division
from game import Directions
from game import Agent
from game import Actions
from util import manhattanDistance
from collections import defaultdict
from APSP import Graph, Coordinate
import sys
import util
import time

import search
import random

class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    def __init__(self, fn='uniformCostSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        if fn not in dir(search):
            raise AttributeError, fn + ' is not a search function in search.py.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' is not a function in searchAgents.py or search.py.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            self.searchFunction = lambda x: func(x, heuristic=heur)

        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' is not a search problem type in SearchAgents.py.'
        self.searchType = globals()[prob]
        self.heuristicInfo = None
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """]
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        problem = self.searchType(state)
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            self.registerInitialState(state)
            self.actionIndex = 0
            i = self.actionIndex
            self.actionIndex += 1
            return self.actions[i]
       

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """

    def __init__(self, startingGameState, heuristicFlag=False, heuristicInfo=None):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.ghostPositions = [(int(x), int(y)) for (x, y) in startingGameState.getGhostPositions()]
        self.startingGameState = startingGameState
        self._expanded = 0 # DO NOT CHANGE
        self.heuristicInfo = heuristicInfo  # A dictionary for the heuristic to store information
        self.foodPosition = []
        self.computeFoodPositions()
        if heuristicInfo is None and heuristicFlag:
            self.computeHeuristicInfo()

    def computeHeuristicInfo(self):
        graph = Graph(height=self.walls.height, width=self.walls.width)
        for (x, row) in enumerate(self.walls.data):
            for (y, value) in enumerate(self.walls.data[x]):
                currentCoordinate = Coordinate(x, y)
                if value:
                    graph.addEdge(currentCoordinate, currentCoordinate, 0)
                else:
                    for neighborCoordinate in self.getAllNeighbors(currentCoordinate):
                        graph.addEdge(currentCoordinate, neighborCoordinate, 1)
        graph.computeAllPairShortestPath()
        self.heuristicInfo = graph

    def getAllNeighbors(self, coordinate):
        neighbors = []
        (x, y) = coordinate
        if x + 1 < self.walls.width and not self.walls.data[x + 1][y]:
            neighbors.append(Coordinate(x + 1, y))
        if x - 1 >= 0 and not self.walls.data[x - 1][y]:
            neighbors.append(Coordinate(x - 1, y))
        if y + 1 < self.walls.height and not self.walls.data[x][y + 1]:
            neighbors.append(Coordinate(x, y + 1))
        if y - 1 >= 0 and not self.walls.data[x][y - 1]:
            neighbors.append(Coordinate(x, y - 1))
        return neighbors

    def computeFoodPositions(self):
        for x in range(0, self.start[1].width):
            for y in range(0, self.start[1].height):
                if self.start[1].data[x][y]:
                    self.foodPosition.append((x, y))

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1 # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x,y= self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class BFSFoodSearchAgent(SearchAgent):
    def __init__(self, fn='breadthFirstSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)


class DFSFoodSearchAgent(SearchAgent):
    def __init__(self, fn='depthFirstSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)


class UCSFoodSearchAgent(SearchAgent):
    def __init__(self, fn='uniformCostSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def getAction(self, state):
        self.registerInitialState(state)
        return self.actions[0]


class AStarFoodSearchAgent(SearchAgent):
    def __init__(self, fn='aStarSearch', prob='FoodSearchProblem', heuristic='foodHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

class AStarGhostEvadeAgent(SearchAgent):
    def __init__(self, fn='aStarSearch', prob='FoodSearchProblem', heuristic='ghostHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def getAction(self, state):
        self.registerInitialState(state)
        return self.actions[0]

class AStarFoodSearchAndGhostEvadeAgent(SearchAgent):
    def __init__(self, fn='aStarSearch', prob='FoodSearchProblem', heuristic='ghostEvadeAndFoodSearchHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def getAction(self, state):
        self.registerInitialState(state)
        return self.actions[0]


class AStarSwitchingModeAgent(SearchAgent):
    def __init__(self, fn='aStarSearchSwitching', prob='FoodSearchProblem', heuristic='foodHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def getAction(self, state):
        self.registerInitialState(state)
        return self.actions[0]


class GradientDescentAgent(SearchAgent):
    def __init__(self, fn='aStarSearchSwitching', prob='FoodSearchProblem', heuristic='foodHeuristicDijkstraDistance'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def registerInitialState(self, state):
        if self.searchFunction == None: raise Exception, "No search function provided for SearchAgent"
        starttime = time.time()
        if self.heuristicInfo is None:
            problem = self.searchType(state, True, self.heuristicInfo)  # Makes a new search problem
            self.heuristicInfo = problem.heuristicInfo
        else:
            problem = self.searchType(state, True, self.heuristicInfo)
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        self.registerInitialState(state)
        return self.actions[0]

def ghostEvadeAndFoodSearchHeuristic(state, problem):
    ghost = ghostHeuristic(state, problem)
    food = foodHeuristic(state, problem)
    if ghost == 0:
        ghost = 1
    return food / (1 / ghost)

def foodHeuristic(state, problem):
    heuristic = manhattanDistance(state[0][0], problem.foodPosition[0])
    for foodPos in problem.foodPosition:
        heuristic = min(heuristic, manhattanDistance(state[0][0], foodPos))
    return heuristic

def ghostHeuristic(state, problem):
    heuristic = manhattanDistance(state[0][0], problem.ghostPositions[0])
    for ghostPos in problem.ghostPositions:
        heuristic = min(heuristic, manhattanDistance(state[0][0], ghostPos))

    detect_range = 1
    if heuristic == 0:
        return 9999999

    if heuristic > detect_range:
        return 0
    else:
        return 1 / heuristic

def gradientDescentHeuristic(state, problem):
    f = 999999999999999999
    g = 999999999999999999
    for foodHeuristic in foodHeuristicDijkstraDistance(state, problem):
        f = min(f, foodHeuristic)

    for ghostHeuristic in ghostHeuristicDijkstraDistance(state, problem):
        g = min(g, ghostHeuristic)

    if g == 0:
        g = 1
    return f / (1 / g)

def foodHeuristicDijkstraDistance(state, problem):
    foodHeuristics = problem.heuristicInfo.getDijkstraDistance(state[0][0], problem.foodPosition[0])
    for foodPos in problem.foodPosition:
        foodHeuristics = min(problem.foodPosition, problem.heuristicInfo.getDijkstraDistance(state[0][0], foodPos))
    return foodHeuristics

def ghostHeuristicDijkstraDistance(state, problem):
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