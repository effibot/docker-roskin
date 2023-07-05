import numpy as np


class Node():
    def __init__(self, position, room, parent,threshold):
        self.id = None
        self.position = position
        self.room = room
        self.parent = parent
        self.children = []
        self.visited = False
        self.aliases = []
    def __eq__(self, other):
        return self.position == other.position and self.room == other.room
    def __hash__(self):
        return hash((self.position, self.room))
    def addParent(self, parent):
        self.parent = parent
    def addChild(self, child):
        self.children.append(child)
    def getChildren(self):
        return self.children
    def getParent(self):
        return self.parent
    def isVisited(self):
        return self.visited
    def setVisited(self, visited):
        self.visited = visited
    def getPosition(self):
        return self.position
    def setAliases(self,aliases):
        self.aliases = aliases
    def getAliases(self):
        return self.aliases
    def setRoom(self,room):
        self.room = room
    def getRoom(self):
        return self.room


class Graph():
  def __init__(self):
    self.nodes = []
    # initialize the adjacency matrix
    self.adj_matrix = np.zeros((1, 1), dtype=np.int8)

    def addNode(self, parent, child):
        child.id = len(self.nodes)
        # check if parent exists
        if parent is not None:
            # expand the adj_matrix and fill with zeros
            self.adj_matrix = np.pad(self.adj_matrix, ((0, 1), (0, 1)), 'constant', constant_values=0)
            # TO DO: HOW TO ADD THE CONNECTIONS?
            self.nodes.append(child)
    def getNodes(self):
        return self.nodes
    def getNode(self, id):
        return self.nodes[id]
    def getAdjMatrix(self):
        return self.adj_matrix
    def isconnected(self, node1, node2):
        return self.adj_matrix[node1.id, node2.id] == 1
    def setconnected(self, node1, node2):
        self.adj_matrix[node1.id, node2.id] = 1
    # find the best node to go to a node to another
    def findBestNode(self, node1, node2):
        # TO DO: IMPLEMENT THIS
        return None