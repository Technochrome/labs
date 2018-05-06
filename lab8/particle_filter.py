from grid import *
from particle import Particle
from utils import *
from setting import *
from math import sin,cos
import random


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    def move(p, odom):
        dx,dy,dh = odom
        dx,dy = rotate_point(dx, dy, p.h)
        return Particle(
            p.x + dx,
            p.y + dy,
            diff_heading_deg(p.h, -dh)
        )

    return [move(p, add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)) for p in particles]

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    markers = [parse_marker_info(m[0], m[1], m[2]) for m in grid.markers]

    def dist(*args):
        return math.sqrt(sum([x*float(x) for x in args]))

    def closeness(m1,m2):
        s = []
        for p1 in m1:   # Find closest marker
            s.append(min([(dist(p1[0] - p2[0], p1[1] - p2[1]), p1,p2) for p2 in m2]))
        return s


    def keep(p):
        if not (-.5 <= p.x <= grid.width + .5 and -.5 <= p.y <= grid.height + .5):
            return False
        return True

    def expand(p):
        rel_markers = [rotate_point(m[0] - p.x, m[1] - p.y, -p.h) for m in markers]
        vis_markers = [m for m in rel_markers if abs(m[1]) + 2 < m[0] ] # |y| < x

        # len(distances) = max( len(m_a), len(m_b))
        if len(measured_marker_list) < len(vis_markers):
            p.measures = closeness(vis_markers, measured_marker_list + [(0,0,0)])
        else:
            p.measures = closeness(measured_marker_list, rel_markers)

        c = sum([a[0] for a in p.measures])
        return [p] * int(10 - 2.0 * c)


    filtered = [p for p in particles if keep(p)]

    choices = [e for p in filtered for e in expand(p)] + \
              [Particle(random.uniform(0,grid.width), random.uniform(0,grid.height), random.uniform(-180, 180)) for _ in range(len(particles)//10)]

    ret = []
    for _ in particles:
        ret.append(random.choice(choices))
    return ret

