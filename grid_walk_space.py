'''
    Implementation of the search space for the grid walk search
    Written by: Bjornar Tessem
'''

import time
from copy import deepcopy
from queue import Queue, LifoQueue, PriorityQueue

from grid_walk_node import GridWalkNode
from grid_walk_path import GridWalkPath
from make_grid import is_goal
from zero_h import ZeroH
from difference_h import DifferenceH
from eucldian_h import EuclidianH
from difference_times_three import DifferenceTTH
from heuristic import Heuristic


class GridWalkSpace:
    '''
    This class is a representation of the search space for grid walk search.
    It allows for various approaches to search for a solution
    '''

    HEURISTIC = Heuristic()

    @staticmethod
    def h(node):
        '''
        The heuristic used in the A/A* search
        :param node: the GridWalkNode to compute h-value for
        :return: the estimated distance to the goal
        '''

        return GridWalkSpace.HEURISTIC.h(node)

    def __init__(self, frontier=Queue(), heuristic=Heuristic()):
        '''
        Initialisation of the space
        '''
        self.start = GridWalkNode(0, 0)
        # the search always starts in the upper left corner of the grid
        self.visited = set()
        # at start we have not visited any nodes (squares in the grid)

        # The search frontier is given in the argumentlist
        self.frontier = frontier
        GridWalkSpace.HEURISTIC = heuristic

        self.frontier.put(GridWalkPath([self.start], self.h(self.start)))
        # put the start node on the frontier
        # adding the start position's h-value as an estimate of length to goal state
        # the h-value is only useful in A search

    def solve(self):
        '''
        :return: path for grid walk solution
        '''
        start = time.time()
        # start time for timing

        while not self.frontier.empty():
            # if frontier is empty there is no solution
            next_path = self.frontier.get()
            # get the next path from the frontier
            last_node = next_path.node_list[-1]
            # get the last node of this path

            if is_goal(last_node.i, last_node.j):
                # if we have a solution

                end = time.time()
                # we stop timing
                print("Time: ", end - start)
                # print time spent

                print("Nodes visited ", len(self.visited) + 1)
                # print the number of nodes visited in the search space

                print("Length: ", len(next_path.node_list), " Cost: ", next_path.cost)
                # print length and cost of path

                return next_path
                # and return the found path

            children = self.get_children(next_path)
            # else find the children for this path
            for child in children:
                # put children on the frontier
                self.frontier.put(child)

        print("No solution found")
        # print than no solutions are found

        return None

    def get_children(self, path):
        '''
        makes a list of paths that are children of the input path
        :param path: the path to add children for
        :return: the list of children paths
        '''
        children = []
        last = path.node_list[-1]
        # find the last node of the path

        if not self.is_visited(last):
            # only do this if the last one is not already visited
            # add moves by making moves where you move in
            # in different directions
            self.add_move(children, path.node_list, last.up_left())
            self.add_move(children, path.node_list, last.up())
            self.add_move(children, path.node_list, last.up_right())
            self.add_move(children, path.node_list, last.left())
            self.add_move(children, path.node_list, last.right())
            self.add_move(children, path.node_list, last.down_left())
            self.add_move(children, path.node_list, last.down())
            self.add_move(children, path.node_list, last.down_right())

            self.visited.add(last)
            # add last to the set of visited nodes

        return children

    def is_visited(self, last):
        '''
        checks if a node in the search space is already visited
        :param last: the node to check
        :return: true if already visitetd
        '''
        return last in self.visited

    def add_move(self, children, path, node):

        '''
        Adds a single new path to a set of paths contained in children
        Copies path and adds node into the copied path
        :param children: the set of paths to add the new path to
        :param path: the path to which a next move is added
        :param node: the new node in the search space that is added to the path
        :return: nothing
        '''
        if node is not None:
            new_path = deepcopy(path)
            # make a deep copy of path

            new_path.append(node)
            # append node to this path

            grid_path = GridWalkPath(new_path, self.h(node))
            # makes a new GridWalkPath object with the new path and the h value (for A search)

            children.append(grid_path)


if __name__ == '__main__':
    """
    print("Breadth-first:")
    sp = GridWalkSpace(Queue())
    sp.solve()
    print('Task 3:')
    sp = GridWalkSpace(Queue(), DifferenceH())
    sp.solve()
    print('Task 4:')
    sp = GridWalkSpace(Queue(), EuclidianH())
    sp.solve()
    print('Task 5:')
    sp = GridWalkSpace(Queue(), DifferenceTTH())
    sp.solve()
    print('zero_h')
    sp = GridWalkSpace(Queue(), ZeroH())
    sp.solve()

    print("Depth-first:")
    sp = GridWalkSpace(LifoQueue())
    sp.solve()
    print('Task 3:')
    sp = GridWalkSpace(LifoQueue(), DifferenceH())
    sp.solve()
    print('Task 4:')
    sp = GridWalkSpace(LifoQueue(), EuclidianH())
    sp.solve()
    print('Task 5:')
    sp = GridWalkSpace(LifoQueue(), DifferenceTTH())
    sp.solve()
    print('zero_h')
    sp = GridWalkSpace(LifoQueue(), ZeroH())
    sp.solve()
    """
    print("Lowest-cost-first:")
    sp = GridWalkSpace(PriorityQueue())
    sp.solve()
    print('Task 3:')
    sp = GridWalkSpace(PriorityQueue(), DifferenceH())
    sp.solve()
    print('Task 4:')
    sp = GridWalkSpace(PriorityQueue(), EuclidianH())
    sp.solve()
    print('Task 5:')
    sp = GridWalkSpace(PriorityQueue(), DifferenceTTH())
    sp.solve()
    print('zero_h')
    sp = GridWalkSpace(PriorityQueue(), ZeroH())
    sp.solve()


"""
solution:
Breadth-first:
Time:  26.68190598487854
Nodes visited  6452
Length:  115  Cost:  351
Task 3:
Time:  31.583608627319336
Nodes visited  6452
Length:  115  Cost:  351
Task 4:
Time:  39.71876907348633
Nodes visited  6452
Length:  115  Cost:  351
Task 5:
Time:  26.218425989151
Nodes visited  6452
Length:  115  Cost:  351

Depth-first:
Time:  12.01282286643982
Nodes visited  787
Length:  737  Cost:  2129
Task 3:
Time:  11.58883810043335
Nodes visited  787
Length:  737  Cost:  2129
Task 4:
Time:  11.426193952560425
Nodes visited  787
Length:  737  Cost:  2129
Task 5:
Time:  11.43018126487732
Nodes visited  787
Length:  737  Cost:  2129

Lowest-cost-first:
Time:  20.47558307647705
Nodes visited  6450
Length:  130  Cost:  255
Task 3:
Time:  14.702891111373901
Nodes visited  5064
Length:  129  Cost:  255
Task 4:
Time:  15.02544903755188
Nodes visited  4993
Length:  130  Cost:  255
Task 5:
Time:  0.6183910369873047
Nodes visited  280
Length:  124  Cost:  282

"""