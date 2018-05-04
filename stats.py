__author__ = "Nick Anderson"

### Stats
# This class tracks various stats for Searcher objects to print out in below format.
class Stats:

    # Takes in a search type, map file, hfn, and total nodes from Searcher object
    def __init__(self, newSearchType, infile, newHfn, newTotalNodes):
        # Initialize varibles with default values
        self.searchType = newSearchType
        self.mapFile = infile
        self.hfn = newHfn
        self.totalNodes = newTotalNodes
        self.startNode = ''
        self.goalNode = []
        self.endCost = 0
        self.endPath = []
        self.frontierTotal = 0
        self.frontierMax = 0
        self.depthTotal = 0
        self.depthMax = 0
        self.branchingTotal = 0
        self.branchingAve = 0
        self.expansion = []

    # Prints to terminal detailed stats of searcher object, additional stats for A* search and if in verbose mode
    def printStats(self, verbose):
        print('------------------------')
        print('SEARCH SUMMARY STATS:')
        print('Search Type: {}. Map file: {} Total Nodes in Graph: {}'.format(self.searchType, self.mapFile, self.totalNodes))
        if self.searchType == 'A*':
            print('Using h-function: {}'.format(self.hfn))
        print('Start node: {} ; Goal node(s): {}'.format(self.startNode, self.goalNode))
        print('Searched total of {} nodes out of total of {} in graph'.format(len(self.expansion), self.totalNodes))
        print('Ended at Node: {} with path cost: {}'.format(self.expansion[-1], self.endCost))
        print('Path ({}): {}'.format(len(self.endPath), self.endPath))
        print('Frontier size: Average= {:.2f} ; Max size= {}'.format(self.frontierTotal / len(self.expansion), self.frontierMax))
        print('Depth of search: Average= {:.2f} ; Max Depth= {}'.format(self.depthTotal / len(self.expansion), self.depthMax))
        print('Average branching factor= {:.2f}'.format(self.branchingTotal / len(self.expansion)))
        if verbose:
            print('Order of Node Expansion: {}'.format(self.expansion))
        print('------------------------')
        print('')

    # Sets start and end goals
    def setStartGoal(self, newStart, newGoal):
        self.startNode = newStart
        self.goalNode = newGoal

    # Updates order of node expansion
    def setExpansion(self, newExpansion):
        self.expansion = newExpansion

    # Sets end cost of search from last node explored
    def setEndCost(self, cost):
        self.endCost = cost

    # Sets end path of search from last node explored
    def setEndPath(self, path):
        self.endPath = path

    # Updates total branching factors to be used in calculating average branching factor
    def updateBranching(self, branching):
        self.branchingTotal = self.branchingTotal + branching

    # Updates total depth factors to be used in calculating average depth factor and max depth
    def updateDepth(self, depth):
        self.depthTotal = self.depthTotal + depth
        if depth > self.depthMax:
            self.depthMax = depth

    # Updates total frontier size to be used in calculating average frontier factor and max frontier
    def updateFrontier(self, frontier):
        self.frontierTotal = self.frontierTotal + frontier
        if frontier > self.frontierMax:
            self.frontierMax = frontier
