#!/usr/bin/env python3
import os, tf
import rospy
from Utils.dynconstraint import DynConstraint
from Utils.planner import Planner
from Utils.uamtraj import Uamtraj
from Utils.vis_utils import *
from Utils.poly_utils import *
base_dir = os.path.dirname(os.path.abspath(__file__))
cfg_dir = os.path.join(base_dir, '..', 'cfg')
coeff_dir = os.path.join('/home/zzp/Safeplan/pyuam_ws/src/pyuam/data')
from visualization_msgs.msg import MarkerArray, Marker
from decomp_ros_msgs.msg import PolyhedronArray
from std_msgs.msg import Header
from nav_msgs.msg import Path
import threading
import json

polys_shrinked = None  # PolyhedronArray
polys = None           # PolyhedronArray
paths = None           # Path

def trajOptimize(polys, paths, planner_param, radius=0.1):
    dim = {}
    dim['c'] = 3
    dim['a'] = 1
    dim['b'] = 2
    order = {}
    # the order of the polynomial 
    order['c'] = 6
    order['a'] = 4
    order['b'] = 4
    global polys_shrinked
    dynconstraint = DynConstraint(cfg_dir + '/dynconstraint.yaml')
    if len(polys.polyhedrons) is not len(paths.poses) - 1:
        rospy.ERROR("The number of polyhedrons is not equal to the number of paths")
        return 0

    hypers = polys2hypers(polys)
    polys_shrinked = shrink_polys(polys, radius=radius)
    hypers_shrinked = polys2hypers(polys_shrinked)

    waypoints = paths2waypoints(paths)

    planner = Planner(dynconstraint, planner_param=planner_param, dim=dim, order=order,
                      hypers=hypers, hypers_shrinked=hypers_shrinked, 
                      waypoints=waypoints)
    planner.setup()
    time_start = rospy.Time.now().to_sec()
    planner.solve()
    time_end = rospy.Time.now().to_sec()
    rospy.logwarn("plan time cost: %f" % (time_end - time_start))
    coeff_ = planner.rearrange_result()
    uamtraj_ = Uamtraj(coeff_, dynconstraint, dim, order)
    return coeff_, uamtraj_

def path_callback(msg):
    global paths
    paths = msg

def polyhedron_callback(msg):
    global polys
    polys = msg

def thread_start_end_publish():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        uav_traj_start_pub.publish(uav_start_marker)
        end_traj_end_pub.publish(end_end_marker)
        rate.sleep()

def thread_publish():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        uav_pub.publish(uav_markers)
        link0_pub.publish(link0_markers)
        link1_pub.publish(link1_markers)
        link2_pub.publish(link2_markers)
        uav_traj_pub.publish(uav_traj_marker)
        end_traj_pub.publish(end_traj_marker)
        refined_poly_pub.publish(polys_shrinked)
        end_effector_pub.publish(end_effector_marker)
        for t in tfs:
            t.header.stamp = rospy.Time.now()
            br.sendTransformMessage(t)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('safeplan_node')
    radius = rospy.get_param('/safeplan/safeplan/radius', 0.1)

    T_lambda = rospy.get_param('/safeplan/safeplan/T_lambda', 0.1)
    d4x_lambda = rospy.get_param('/safeplan/safeplan/d4x_lambda', 0.1)
    ddpsi_lambda = rospy.get_param('/safeplan/safeplan/ddpsi_lambda', 0.1)
    ddtheta_lambda = rospy.get_param('/safeplan/safeplan/ddtheta_lambda', 0.1)
    xef_lambda = rospy.get_param('/safeplan/safeplan/xef_lambda', 0.1)
    avoid_method = rospy.get_param('/safeplan/safeplan/avoid_method', 'sphere')

    planner_param = {
        'T_lambda': T_lambda, 
        'd4x_lambda': d4x_lambda,
        'ddpsi_lambda': ddpsi_lambda, 
        'ddtheta_lambda': ddtheta_lambda, 
        'xef_lambda': xef_lambda, 
        'avoid_method': avoid_method
    }

    poly_sub = rospy.Subscriber('/path_decomp_jps/polyhedron_array', PolyhedronArray, polyhedron_callback, queue_size=1)
    path_sub = rospy.Subscriber('/path_decomp_jps/path', Path, path_callback, queue_size=1)
    while polys is None:
        print("Waiting for polyhedrons")
        rospy.rostime.wallsleep(1)
    while paths is None:
        rospy.rostime.wallsleep(1)

    uav_pub = rospy.Publisher('uav_marker', MarkerArray, queue_size=10)
    link0_pub = rospy.Publisher('link0_marker', MarkerArray, queue_size=10)
    link1_pub = rospy.Publisher('link1_marker', MarkerArray, queue_size=10)
    link2_pub = rospy.Publisher('link2_marker', MarkerArray, queue_size=10)
    uav_traj_pub = rospy.Publisher('uav_traj', Marker, queue_size=10)
    end_traj_pub = rospy.Publisher('end_traj', Marker, queue_size=10)
    end_effector_pub = rospy.Publisher('end_effector', Marker, queue_size=10)
    
    uav_traj_start_pub = rospy.Publisher('uav_traj_start', Marker, queue_size=10)
    end_traj_end_pub = rospy.Publisher('end_traj_end', Marker, queue_size=10)
    
    br = tf.TransformBroadcaster()

    refined_poly_pub = rospy.Publisher('/path_decomp_jps/refined_polyhedron_array', PolyhedronArray, queue_size=10)

    wp_ = paths2waypoints(paths)
    uav_start_color_rgba = [0.0, 0.0, 1.0, 1.0]
    uav_start_scale = [0.12, 0.12, 0.12]
    uav_start_marker = getSphereMarker(uav_start_color_rgba, uav_start_scale, wp_[0])
    
    end_end_color_rgba = [99/255, 227/255, 152/255, 1.0]
    end_end_scale = [0.12, 0.12, 0.12]
    end_end_marker = getSphereMarker(end_end_color_rgba, end_end_scale, wp_[-1])
    
    start_publish_thread = threading.Thread(target=thread_start_end_publish)
    start_publish_thread.start()
    rospy.rostime.wallsleep(0.5)
    # === 1. solve the optimization problem ===
    coeff, uamtraj = trajOptimize(polys=polys, paths=paths, planner_param=planner_param, radius=radius)
    with open(coeff_dir + '/coeff.json', 'w') as f:
        json.dump(coeff, f)

    xb, x0, xm, xe, Rb, Re = uamtraj.xb, uamtraj.x0, uamtraj.xm, uamtraj.xe, uamtraj.Rb, uamtraj.Re

    # === 2. visualization ===
    header = Header()
    header.frame_id = "map"

    uav_traj_color_rgba = [0.0, 0.0, 1.0, 1.0]
    uav_traj_scale_x = 0.05
    uav_traj_marker = getLineMarker(uav_traj_color_rgba, uav_traj_scale_x, xb)

    joint2_traj_color_rgba = [0.5, 0.0, 0.5, 1.0]
    joint2_traj_scale_x = 0.05
    joint2_traj_marker = getLineMarker(joint2_traj_color_rgba, joint2_traj_scale_x, xm)

    end_traj_color_rgba = [99/255, 227/255, 152/255, 1.0]
    end_traj_scale_x = 0.05
    end_traj_marker = getLineMarker(end_traj_color_rgba, end_traj_scale_x, xe)

    time_margin = 200
    link0_markers = MarkerArray()
    link1_markers = MarkerArray()
    link2_markers = MarkerArray()
    uav_markers = MarkerArray()
    end_effector_marker = Marker()
    tfs = []
    for i in range(len(xb)):
        if i == 0 or i == len(xb) - 1:
            if i == 0:
                id = 1
            elif i == len(xb) - 1:
                id = 2
            uav_marker_color_rgba = [0.0, 0.0, 1.0, 1.0]
            uav_marker_scale = [1, 1, 1]
            uav_marker = getQuadMarker(uav_marker_color_rgba, uav_marker_scale, xb[i], id)
            uav_markers.markers.append(uav_marker)

            link0_marker_color_rgba = [125.0/255.0, 0.0/255.0, 125.0/255.0, 1.0]
            link0_marker_scale = [0.05, 0.05, 0.05]
            link0_marker = getLinkMarker(link0_marker_color_rgba, link0_marker_scale, xb[i], x0[i], id)
            link0_markers.markers.append(link0_marker)

            link1_marker_color_rgba = [125.0/255.0, 0.0/255.0, 125.0/255.0, 1.0]
            link1_marker_scale = [0.05, 0.05, 0.05]
            link1_marker = getLinkMarker(link1_marker_color_rgba, link1_marker_scale, x0[i], xm[i], id)
            link1_markers.markers.append(link1_marker)

            link2_marker_color_rgba = [125.0/255.0, 0.0/255.0, 125.0/255.0, 1.0]
            link2_marker_scale = [0.05, 0.05, 0.05]
            link2_marker = getLinkMarker(link2_marker_color_rgba, link2_marker_scale, xm[i], xe[i], id)
            link2_markers.markers.append(link2_marker)
            
            end_effector_marker_color_rgba = [99/255, 227/255, 152/255, 1.0]
            end_effector_marker_scale = [0.5, 0.05, 0.05]
            end_effector_marker = getArrowMarker(end_effector_marker_color_rgba, end_effector_marker_scale, xe[i], Re[i])
            te = getTF(xe[i], Re[i], id)
            tb = getTF(xb[i], Rb[i], id)
            tfs.append(te)

    publish_thread = threading.Thread(target=thread_publish)
    publish_thread.start()

    rospy.spin()
