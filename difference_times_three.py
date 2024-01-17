from heuristic import Heuristic
import make_grid


class DifferenceTTH(Heuristic):

    @staticmethod
    def h(node):
        grid = make_grid.SIZE
        diff = grid - node.i
        diff2 = grid - node.j

        return max(diff, diff2) * 3
