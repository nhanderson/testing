__author__ = "Nick Anderson"

import numpy as np
from graphviz import GraphViz
from searchnode import SearchNode
from stats import Stats

### Searcher
# This class implements various search algorithms for finding a path from start to goal on a graph. It provides methods for ways
# to run the search (optionally for X steps vs whole thing), way to set the start/goal nodes, and ways to print stats.
class Searcher:

    #Searcher takes in a search type DFS, BFS, Best-first, and A*, mapfile, a Heuristic Function, and boolen value for enabling verbose mode.
    def __init__(self, newSearchType, infile, newHfn, newVerbose=False):

        # Intializes variables for the class
        self.searchType = newSearchType
        self.hfn = newHfn
        self.verbose = newVerbose
        self.openList = []
        self.closedList = []
        self.edges = []
        self.nodeIndex = {}
        self.startNode = ''
        self.goalNode = []

        #Loads in the provided map file
        self.loadGraphFromFile(infile)

        #Create a new stats object to track stats
        self.stats = Stats(newSearchType, infile, newHfn, len(self.nodeIndex))

        # Uses GraphViz by Eck Doerry to create new instance and display graph
        self.graph = GraphViz()
        self.graph.loadGraphFromFile(infile)
        self.graph.plot()

    # Load in map file
    # Based on loadGraphFromFile function from GraphViz by Eck Doerry
    def loadGraphFromFile(self, infile):
        with open(infile) as f:
            lines = f.readlines()
        cleanlines = [x.strip() for x in lines] #strip off any weird leading or trailing spaces.
        for line in cleanlines:
            line=line.replace('\'','').replace('[','').replace(']','').replace(' ','').strip('()')
            rawEdge = line.split(',') # now just have nice clean comma-separated values. Make it a list
            [l1,l2,value,x1,y1,x2,y2] = rawEdge  # grab all the components of the edge
            if l1 not in self.nodeIndex:
                self.nodeIndex[l1]=(int(x1),int(y1))
            if l2 not in self.nodeIndex:
                self.nodeIndex[l2] = (int(x2), int(y2))
            self.edges.append([l1,l2,int(value)])
        f.close()


    # Takes in a new start node and a goal or list of goals. Resets graph for new search.
    def setStartGoal(self, newStart, newGoal):
        # Clears open list, sets new start node, and displays start node on graph
        self.goalNode = []
        for goal in newGoal:
            self.goalNode.append(goal.upper())
            self.graph.markGoal(goal.upper())   # mark goal node

        # Clears open and closed lists, sets new start node, adds start node to open list, and displays start node on graph
        self.openList = []
        self.closedList = []
        self.startNode = newStart.upper()
        self.graph.markStart(self.startNode)  # mark starting node
        self.openList.append(self.createNode(self.startNode,0))

        # Draws start and goal nodes
        self.stats.setStartGoal(self.startNode, self.goalNode)


    # Takes in a node and returns sorted list of child nodes
    def successor(self, node):
        # Searches through list of edges for lines invloving input the node
        children = []
        for edge in self.edges:
            # For each edge found call createNode to create the chilren nodes
            if edge[0] == node.label:
                children.append(self.createNode(edge[1], edge[2], node))
            elif edge[1] == node.label:
                children.append(self.createNode(edge[0], edge[2], node))

        # Update stats with branching factor of the parent node before illegal removals
        self.stats.updateBranching(len(children))

        # Sorts the list by label to preserve order
        return sorted(children, key=lambda node: node.label)

    # Takes in a method for inserting nodes into the open list and list child of child Nodes
    # Valid methods for insertion are 'front', 'end', and 'order'.
    # Order will return a list in accending order by node value.
    def insert(self, order, children):
        childList = []

        # Add nodes to front of open list, used in DEPTH search
        if order == 'front':
            # Checks if node already in open list, discards open list node if duplicate found
            for child in children:
                for node in self.openList:
                    if node.label == child.label:
                        self.openList.remove(node)
                childList.append(child)
            # Appends children node list to begining of open list
            self.openList = childList + self.openList

        # Add to back of open list, used in BREDTH search
        elif order == 'end':
            # Checks if node already in open list and discards child is duplicate found
            for child in children:
                found = False
                for node in self.openList:
                    if node.label == child.label:
                        found = True
                # If not adds to open list
                if not found:
                    childList.append(child)
            # Appends children node list to end of open list
            self.openList = self.openList + childList

        # Added children to open and sorts by value, used in BEST and A*
        elif order == 'order':
            # Checks if node already in open list
            for child in children:
                found = False
                for node in self.openList:
                    if node.label == child.label:
                        found = True
                        # If found in open list and new node is lower cost
                        # then remove old node, and add child node to childList
                        if(node.value > child.value):
                            self.openList.remove(node)
                            childList.append(child)
                # If not adds to open list
                if not found:
                    childList.append(child)
            self.openList = self.openList + childList
            # Sort open list in ascending order by node value
            self.openList = sorted(self.openList, key=lambda node: node.value)

        else:
            print("Unrecongized order")
        # If verbose is True then show the openList
        if self.verbose == True:
            self.showOpen()

    #Prints out the open list in values of (nodelabel, node depth from root, g(n), h(n), f(n))
    def showOpen(self):
        tempList = []
        for node in self.openList:
            tempList.append(node.get())
        print('Open list: {}'.format(tempList))

    # Takes in a list of nodes and returns a list of node labels
    def getNodeNames(self, nodeList):
        nodeNameList = []
        for node in nodeList:
            nodeNameList = nodeNameList + [node.label]
        return nodeNameList

    # Drives search, takes in option step value or runs to compeltion if none specified
    def go(self, steps=-1):
        #  If no step specified, print out search type and goals

        print('{} search: from {} to {}'.format(self.searchType, self.startNode, self.goalNode))
        # Grabs first open list item, checks for goal, and adds children to open list.
        while steps != 0:
            # Prints failure message if open list is empty
            if not self.openList:
                print('Search ended in failure')
                self.stats.printStats(self.verbose)
                break
            else:
                # Grabs first node in open list
                steps = steps - 1
                node = self.openList.pop(0)
                if self.verbose == True:
                    print('Exploring node: {}'.format(node.label))
                # Adds node exploring to closed list to prevent cycling
                self.closedList.append(node.label)
                # Updates stats object with depth, frontier, and exlpored path information
                self.stats.setExpansion(self.closedList)
                self.stats.updateDepth(node.depth)
                self.stats.updateFrontier(len(self.openList))
                # Updates graph to show exploring node
                self.graph.exploreNode(node.label, node.path)

                # Checks if explored node is a goal node.
                for goal in self.goalNode:
                    if goal == node.label:
                        if self.verbose == True:
                            print('Solved')
                        # Update ending stats and print to terminal
                        self.stats.setEndCost(node.distance)
                        self.stats.setEndPath(node.path)
                        self.stats.printStats(self.verbose)
                        return

                # Generates children of explored node and determines how to add to open list
                children = self.successor(node)
                # Update graph with new frontier nodes
                self.graph.exploreEdges(node.label, self.getNodeNames(children))
                # Removes any generates children already closed list
                for closed in self.closedList:
                    for child in children:
                        if child.label == closed:
                            children.remove(child)
                            break
                if self.verbose == True:
                    print('Inserting new children: {}'.format(self.getNodeNames(children)))
                # Determines what order children nodes should be added to open list
                if self.searchType == 'BREADTH':
                    self.insert('end', children)
                elif self.searchType == 'DEPTH':
                    self.insert('front', children)
                elif self.searchType in ['BEST', 'A*']:
                    self.insert('order', children)

    # Takes in a node label, distance from parent and parent node.
    # Creates and returns a new SearchNode object.
    def createNode(self, label, distance, parent='root'):
        # Handles special case of root node
        if parent == 'root':
            newNodeDistance = 0
            newNodePath = [label]
        else:
            # Calculates new values for path and distance based off parent node.
            parentDistance = parent.distance
            newNodePath = list(parent.path)
            newNodeDistance = parentDistance + distance
            newNodePath.append(label)

        # Creates the new node in the case of BREADTH, DEPTH, and BEST search types.
        if self.searchType in ['BREADTH', 'DEPTH','BEST']:
            newNode = SearchNode(label, newNodeDistance, 0, newNodeDistance, newNodePath)

        # Creates new node in case of A* search type. Calculates value for new node based on h function.
        if self.searchType == 'A*':
            # Creates new node based on hSLD function
            if self.hfn == hSLD:
                newHSLD =  hSLD(self, label)
                newValue = newNodeDistance + newHSLD
                newNode = SearchNode(label, newValue, newHSLD, newNodeDistance, newNodePath)

            # Creates new node based on hDir function
            if self.hfn == hDir:
                if parent == 'root':
                    newNode = SearchNode(label, newNodeDistance, 0, newNodeDistance, newNodePath)
                else:
                    newhDir =  hDir(self, parent.label, label)
                    newValue = newhDir + newNodeDistance
                    newNode = SearchNode(label, newValue, newhDir, newNodeDistance, newNodePath)

        return newNode

# A simple "Straight Line Distance (SLD)" heuristic function.
# Takes in node label and returns the distance to the closest goal.
def hSLD(searcher, label):
    x1,y1 = searcher.nodeIndex[label]
    for goal in searcher.goalNode:
        x2,y2 = searcher.nodeIndex[goal]
        dist = ( (x2 - x1)**2 + (y2 - y1)**2 )**(1/2)
        try:
            if(dist < min):
                min = dist
        except:
            min = dist
    return min

# Takes in a parent and child label and determines the angle between them and the goal in the straighest line.
# Equations and pseudo code taken from http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
# Normalization equation based on https://stackoverflow.com/questions/37459121/calculating-angle-between-three-points-but-only-anticlockwise-in-python
def hDir(searcher, parentLabel, childLabel):
    # Calculates the vector from parent to child
    x1, y1 = searcher.nodeIndex[parentLabel]
    parentPoint = np.array([x1, y1])
    x2, y2 = searcher.nodeIndex[childLabel]
    childPoint = np.array([x2, y2])
    parentChildVector = childPoint - parentPoint

    # For each goal calculates the vector from parent to goal
    for goal in searcher.goalNode:
        # Handles if child is goal node to prevent division by zero and sets equal to hDir to 0
        if childLabel == goal:
            min = 0
        else:
            x3, y3 = searcher.nodeIndex[goal]
            goalPoint = np.array([x3, y3])
            parentGoalVector = goalPoint - parentPoint

            # Uses numpy to calculate the dot product of the two vectors and normalize
            dotProduct = np.dot(parentChildVector,parentGoalVector) / (np.linalg.norm(parentChildVector) * np.linalg.norm(parentGoalVector))

            # Calculates the angle in radians with the formula angle = acos(v1•v2) and converts to degrees
            angle = np.arccos(dotProduct)
            angleDeg = np.degrees(angle)
            try:
                if(angleDeg < min):
                    min = angleDeg
            except:
                min = angleDeg

        return min

# Fake h function used to pass as varible when doing other searche types
def hNUll(self):
    return 0

z = Searcher('A*','50test.txt', hSLD, False)
z.setStartGoal('m',['c','af'])
z.go()
