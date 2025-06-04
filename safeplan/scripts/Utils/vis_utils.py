#!/usr/bin/env python3
import os
import tf
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
import numpy as np

def rotation2quaternion(R):
    T = np.eye(4)
    T[:3, :3] = R
    quaternion = quaternion_from_matrix(T)
    return quaternion

def getLineMarker(rgba, scale_x, traj):
    marker_ = Marker()
    marker_.header.frame_id = "map"
    marker_.type = Marker.LINE_STRIP
    marker_.action = Marker.ADD
    marker_.scale.x = scale_x
    marker_.color.r = rgba[0]
    marker_.color.g = rgba[1]
    marker_.color.b = rgba[2]
    marker_.color.a = rgba[3]
    marker_.pose.orientation.w = 1.0

    for traj_p in traj:
        p = Point()
        p.x, p.y, p.z = traj_p[0], traj_p[1], traj_p[2]
        marker_.points.append(p)
    return marker_

def getQuadMarker(rgba, scale, uav_traj, id):
    mesh_resource = "package://safeplan/meshes/hummingbird.mesh"
    marker_ = Marker()
    marker_.header.frame_id = "map"
    marker_.type = Marker.MESH_RESOURCE
    marker_.ns = "mesh"
    marker_.id = id
    marker_.pose.position.x = uav_traj[0]
    marker_.pose.position.y = uav_traj[1]
    marker_.pose.position.z = uav_traj[2]
    marker_.pose.orientation.w = 1.0
    marker_.scale.x = scale[0]
    marker_.scale.y = scale[1]
    marker_.scale.z = scale[2]
    marker_.color.r = rgba[0]
    marker_.color.g = rgba[1]
    marker_.color.b = rgba[2]
    marker_.color.a = rgba[3]
    marker_.mesh_resource = mesh_resource
    return marker_

def getLinkMarker(rgba, scale, uav_traj, end_traj, id):
    marker_ = Marker()
    marker_.header.frame_id = "map"
    marker_.type = Marker.LINE_LIST
    marker_.ns = "link"
    marker_.id = id
    marker_.pose.orientation.w = 1.0
    marker_.scale.x = scale[0]
    marker_.scale.y = scale[1]
    marker_.scale.z = scale[2]
    marker_.color.r = rgba[0]
    marker_.color.g = rgba[1]
    marker_.color.b = rgba[2]
    marker_.color.a = rgba[3]
    p1, p2 = Point(), Point()
    p1.x, p1.y, p1.z = uav_traj[0], uav_traj[1], uav_traj[2]
    p2.x, p2.y, p2.z = end_traj[0], end_traj[1], end_traj[2]
    marker_.points.append(p1)
    marker_.points.append(p2)
    return marker_

def getTF(position, rotation, id):
    t_ = TransformStamped()
    t_.header.frame_id = "map"
    t_.child_frame_id = "base_link_" + str(id)
    t_.transform.translation.x = position[0]
    t_.transform.translation.y = position[1]
    t_.transform.translation.z = position[2]
    q = rotation2quaternion(rotation)
    t_.transform.rotation.x = q[0]
    t_.transform.rotation.y = q[1]
    t_.transform.rotation.z = q[2]
    t_.transform.rotation.w = q[3]
    return t_

def getSphereMarker(rgba, scale_x, pos):
    marker_ = Marker()
    marker_.header.frame_id = "map"
    marker_.type = Marker.SPHERE
    marker_.action = Marker.ADD
    marker_.scale.x = scale_x[0]
    marker_.scale.y = scale_x[1]
    marker_.scale.z = scale_x[2]
    marker_.color.r = rgba[0]
    marker_.color.g = rgba[1]
    marker_.color.b = rgba[2]
    marker_.color.a = rgba[3]
    marker_.pose.orientation.w = 1.0
    marker_.pose.position.x = pos[0]
    marker_.pose.position.y = pos[1]
    marker_.pose.position.z = pos[2]
    marker_.lifetime = rospy.Duration(100)
    return marker_

def getArrowMarker(rgba, scale, pos, rot):
    marker_ = Marker()
    marker_.header.frame_id = "map"
    marker_.type = Marker.ARROW
    marker_.action = Marker.ADD
    marker_.scale.x = scale[0]
    marker_.scale.y = scale[1]
    marker_.scale.z = scale[2]
    marker_.color.r = rgba[0]
    marker_.color.g = rgba[1]
    marker_.color.b = rgba[2]
    marker_.color.a = rgba[3]
    marker_.pose.orientation.w = 1.0
    marker_.pose.position.x = pos[0]
    marker_.pose.position.y = pos[1]
    marker_.pose.position.z = pos[2]
    q = rotation2quaternion(rot)
    marker_.pose.orientation.x = q[0]
    marker_.pose.orientation.y = q[1]
    marker_.pose.orientation.z = q[2]
    marker_.pose.orientation.w = q[3]
    marker_.lifetime = rospy.Duration(100)
    return marker_