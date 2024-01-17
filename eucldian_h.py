# math.sqrt(a^2 + b^2)
from heuristic import Heuristic
import make_grid
import math


class EuclidianH(Heuristic):

    @staticmethod
    def h(node):
        grid = make_grid.SIZE
        diff = grid - node.i
        diff2 = grid - node.j

        return math.sqrt(diff**2 + diff2**2)
