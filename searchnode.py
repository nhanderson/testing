__author__ = "Nick Anderson"

# Class creates a new node to search.
class SearchNode:

    # Takes in nodelabel, node depth from root, g(n), h(n), f(n)
    def __init__(self, newLabel, newValue, newH, newDistance, newPath):
        self.label = newLabel
        # Value is distance to root for all but BEST and A* searches
        self.value = newValue
        self.h = newH
        self.distance = newDistance
        self.path = newPath
        # Removes 1 from length to start depth count at 0
        self.depth = len(self.path)-1

    # Get values for node, formats floats as strings to 2 decimals
    def get(self):
        return (self.label, self.depth, self.distance, '{:.2f}'.format(self.h), '{:.2f}'.format(self.value))

    # Accessor methods to values for node
    def label(self):
        return (self.label)

    def value(self):
        return (self.value)

    def h(self):
        return (self.h)

    def distance(self):
        return (self.distance)

    def path(self):
        return (self.path)

    def depth(self):
        return (self.depth)
