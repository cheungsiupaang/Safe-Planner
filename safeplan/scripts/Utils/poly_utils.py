import numpy as np
from decomp_ros_msgs.msg import PolyhedronArray
from decomp_ros_msgs.msg import Polyhedron
from geometry_msgs.msg import Point

def poly2hyper(poly):
    hyperplane_ = []
    ps = poly.points
    ns = poly.normals
    for p, n in zip(ps, ns):
        Ab = np.zeros(4)
        Ab[0], Ab[1], Ab[2] = n.x, n.y, n.z
        Ab[3] = (n.x * p.x + n.y * p.y + n.z * p.z)
        hyperplane_.append(Ab)
    return hyperplane_

def polys2hypers(polys):
    hypers_ = []
    for poly in polys.polyhedrons:
        hyperplane = poly2hyper(poly=poly)
        hypers_.append(hyperplane)
    return hypers_

def shrink_poly(poly, radius):
    polyhedron_shrinked = Polyhedron()
    ps, ns = poly.points, poly.normals
    for p, n in zip(ps, ns):
        A_np = np.zeros(3)
        A_np[0], A_np[1], A_np[2] = n.x, n.y, n.z
        p_np = np.zeros(3)
        p_np[0], p_np[1], p_np[2] = p.x, p.y, p.z
        p_shrinked = p_np - radius * A_np
        point, normal = Point(), Point()
        point.x, point.y, point.z = p_shrinked[0], p_shrinked[1], p_shrinked[2]
        normal.x, normal.y, normal.z = n.x, n.y, n.z
        polyhedron_shrinked.points.append(point)
        polyhedron_shrinked.normals.append(normal)
    return polyhedron_shrinked

def shrink_polys(polyhedrons, radius):
    polyhedrons_shrinked = PolyhedronArray()
    polyhedrons_shrinked.header = polyhedrons.header
    for poly in polyhedrons.polyhedrons:
        poly_shrinked = shrink_poly(poly, radius)
        polyhedrons_shrinked.polyhedrons.append(poly_shrinked)
    return polyhedrons_shrinked

def paths2waypoints(paths):
    waypoints_ = []
    for pose in paths.poses:
        px, py, pz = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        x = []
        x.append(px)
        x.append(py)
        x.append(pz)
        waypoints_.append(x)
    return waypoints_
