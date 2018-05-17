
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo



def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    goal = grid.getGoals()[0]
    visitQueue = PriorityQueue()
    visitQueue.put((0, 0, [grid.getStart()]))
    grid.addVisited(grid.getStart())

    while not visitQueue.empty():
        estimated_cost, src_cost, path = visitQueue.get()
        src_coord = path[-1]
        for coord, inc_cost in grid.getNeighbors(src_coord):
            if coord not in grid.getVisited():
                grid.addVisited(coord)

                dst_path = path + [coord]
                dst_cost = src_cost + inc_cost
                if coord == goal:
                    grid.setPath(dst_path)
                    return
                visitQueue.put((dst_cost + heuristic(coord, goal), dst_cost, dst_path))



def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    dists = [abs(a-b) for a,b in zip(current,goal)]
    linear_dist = max(dists) - min(dists)
    diagonal_dist = min(dists) * math.sqrt(2)
    return linear_dist + diagonal_dist


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    while not stopevent.is_set():
        pass # Your code here


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

