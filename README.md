# pacman
# User Guide
Python 2.7 is required to run this project.
To run type in the terminal of the current pacman folder:
```
python2 pacman.py -p [ AgentClass ] -l [ LayoutFile ] -n [ number of runs ] -k [ number of ghosts ]

AgentClass: in the searchAgents.py
LayoutFile: in the layouts folder
```

#A*
The heuristic function, which is used to evade ghost and eat food at the same time, is foodHeuristic / ghostHeuristic.
It has a limitation that is if the ghost is near the foods Pacman will ignore the ghosts.