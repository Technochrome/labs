
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
    visitQueue = PriorityQueue()
    visitQueue.put((0, 0, grid.getStart(), []))

    while not visitQueue.empty():
        _, src_cost, src_coord, src_path = visitQueue.get()
        if src_coord not in grid.getVisited():
            path = src_path + [src_coord]
            if src_coord in grid.getGoals():
                grid.setPath(path)
                return
            grid.addVisited(src_coord)

            for dst_coord, inc_cost in grid.getNeighbors(src_coord):
                dst_cost = src_cost + inc_cost
                est_cost = min([heuristic(dst_coord, goal) for goal in grid.getGoals()])

                visitQueue.put((dst_cost + est_cost, dst_cost, dst_coord, path))



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
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    
    robot.world.wait_until_num_objects_visible(1)
    cube = [c for c in robot.world.connected_light_cubes if c.is_visible][0].pose.position
    print(cube, to_grid(cube.x, cube.y, grid))
    grid.addGoal(to_grid(cube.x, cube.y, grid))

    astar(grid, heuristic)
    path = [from_grid(pos[0], pos[1], grid) for pos in compress_path(grid.getPath())]
    for x,y in path[1:]:
        robot.go_to_pose(cozmo.util.pose_z_angle(x,y,0, cozmo.util.degrees(0))).wait_for_completed()
    while not stopevent.is_set():
        # has_in_progress_actions
        pass

def to_grid(x, y, grid):
    s = grid.getStart()
    return (x // 20 + s[0], y // 20 + s[1])

def from_grid(x, y, grid):
    s = grid.getStart()
    return ((x - s[0]) * 20, (y - s[1]) * 20)

def get_actions(path):
    curr_dir = cozmo.util.degress(0)
    actions = []
    for prev, curr in zip(path, path[1:]):
        new_dir = cozmo.util.radians(math.atan2(curr[0] - prev[0], curr[1] - prev[1]))
        actions.append((new_dir - curr_dir, ))

def compress_path(path):
    curr_dir = (0, 0)
    c_path = []
    for prev, curr in zip(path, path[1:]):
        dir = (curr[0] - prev[0], curr[1] - prev[1])
        if dir != curr_dir:
            curr_dir = dir
            c_path.append(prev)
    c_path.append(path[-1])
    return c_path



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

