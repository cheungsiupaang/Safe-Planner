#!/usr/bin/env python3
import rospy
import numpy as np
import random, math, rosbag
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from planning_ros_msgs.msg import VoxelMap

RHO = 10000

def rotate_point(x, y, z, theta):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    x_ = cos_theta * x - sin_theta * y
    y_ = sin_theta * x + cos_theta * y
    return x_, y_, z

def add_cuboid_to_cloud(cloud_msg, inflate_cloud_msg, x_minmax, y_minmax, z_minmax, theta, dilation):
    x_min, x_max = x_minmax[0], x_minmax[1]
    y_min, y_max = y_minmax[0], y_minmax[1]
    z_min, z_max = z_minmax[0], z_minmax[1]
    cloud_msg_num = int(RHO * (x_max - x_min) * (y_max - y_min) * (z_max - z_min))
    inflate_cloud_msg_num = int(RHO * (x_max - x_min) * (y_max - y_min) * (z_max - z_min) / 4)

    for _ in range(cloud_msg_num):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = random.uniform(z_min, z_max)
        x -= (x_max + x_min) / 2
        y -= (y_max + y_min) / 2
        z -= (z_max + z_min) / 2
        x, y, z = rotate_point(x, y, z, theta)

        x += (x_max + x_min) / 2
        y += (y_max + y_min) / 2
        z += (z_max + z_min) / 2

        p = Point32(x, y, z)
        cloud_msg.points.append(p)

        for _ in range(2):
            x_ = random.uniform(x_min-dilation, x_max+dilation)
            y_ = random.uniform(y_min-dilation, y_max+dilation)
            z_ = random.uniform(z_min-dilation, z_max+dilation)

            x_, y_, z_ = rotate_point(x_, y_, z_, theta)
            p_ = Point32(x_, y_, z_)
            inflate_cloud_msg.points.append(p_)

def create_point_cloud(obs):
    cloud_msg = PointCloud()
    cloud_msg.header.stamp = rospy.Time.now()
    cloud_msg.header.frame_id = 'map'

    inflate_cloud_msg = PointCloud()
    inflate_cloud_msg.header.stamp = rospy.Time.now()
    inflate_cloud_msg.header.frame_id = 'map'

    for obs_i in obs:
        x_minmax = [obs_i['x_min'], obs_i['x_max']]
        y_minmax = [obs_i['y_min'], obs_i['y_max']]
        z_minmax = [obs_i['z_min'], obs_i['z_max']]
        theta = obs_i['theta']
        dilation = obs_i['dilation']
        add_cuboid_to_cloud(cloud_msg, inflate_cloud_msg, x_minmax, y_minmax, z_minmax, theta, dilation)
    return cloud_msg, inflate_cloud_msg

if __name__ == '__main__':
    rospy.init_node('map_creator')

    cloud_pub = rospy.Publisher('/path_decomp_jps/cloud', PointCloud, queue_size=1)
    inflate_cloud_pub = rospy.Publisher('/path_decomp_jps/inflate_cloud', PointCloud, queue_size=1)

    res = rospy.get_param('create_map/resolution', 0.5)
    dilation = rospy.get_param('dilation', 0.2)

    theta1 = rospy.get_param('theta1', 0.0)
    x1_min, x1_max = rospy.get_param('x1_min', 1.0), rospy.get_param('x1_max', 3.0)
    y1_min, y1_max = rospy.get_param('y1_min', 1.0), rospy.get_param('y1_max', 2.0)
    z1_min, z1_max = rospy.get_param('z1_min', 0.0), rospy.get_param('z1_max', 1.2)

    theta2 = rospy.get_param('theta2', 0.0)
    x2_min, x2_max = rospy.get_param('x2_min', 2.0), rospy.get_param('x2_max', 4.0)
    y2_min, y2_max = rospy.get_param('y2_min', 4.0), rospy.get_param('y2_max', 6.0)
    z2_min, z2_max = rospy.get_param('z2_min', 0.0), rospy.get_param('z2_max', 1.6)
    
    theta3 = rospy.get_param('theta3', 0.0)
    x3_min, x3_max = rospy.get_param('x3_min', 5.0), rospy.get_param('x3_max', 6.0)
    y3_min, y3_max = rospy.get_param('y3_min', 2.0), rospy.get_param('y3_max', 4.0)
    z3_min, z3_max = rospy.get_param('z3_min', 0.0), rospy.get_param('z3_max', 2.0)

    theta4 = rospy.get_param('theta3', 0.0)
    x4_min, x4_max = rospy.get_param('x4_min', 5.0), rospy.get_param('x4_max', 6.0)
    y4_min, y4_max = rospy.get_param('y4_min', 2.0), rospy.get_param('y4_max', 4.0)
    z4_min, z4_max = rospy.get_param('z4_min', 0.0), rospy.get_param('z4_max', 2.0)
    
    bag_name = rospy.get_param('bag_name', 'output.bag')

    obs = []
    obs.append({'x_min': x1_min, 'x_max': x1_max, 'y_min': y1_min, 'y_max': y1_max, 
                'z_min': z1_min, 'z_max': z1_max, 'theta': theta1, 'dilation': dilation})
    obs.append({'x_min': x2_min, 'x_max': x2_max, 'y_min': y2_min, 'y_max': y2_max,
                'z_min': z2_min, 'z_max': z2_max, 'theta': theta2, 'dilation': dilation})
    obs.append({'x_min': x3_min, 'x_max': x3_max, 'y_min': y3_min, 'y_max': y3_max,
                'z_min': z3_min, 'z_max': z3_max, 'theta': theta3, 'dilation': dilation})
    obs.append({'x_min': x4_min, 'x_max': x4_max, 'y_min': y4_min, 'y_max': y4_max,
                'z_min': z4_min, 'z_max': z4_max, 'theta': theta4, 'dilation': dilation})

    rate = rospy.Rate(1)
    cloud_msg, inflate_cloud_msg = create_point_cloud(obs=obs)
    
    with rosbag.Bag('/home/zzp/safeplan_ws/src/safeplan/data/'+bag_name, 'w') as bag:
        bag.write('/path_decomp_jps/cloud', cloud_msg)
        bag.write('/path_decomp_jps/inflate_cloud', inflate_cloud_msg)

    print("updated the ROS bag file.")

    while not rospy.is_shutdown():
        cloud_pub.publish(cloud_msg)
        inflate_cloud_pub.publish(inflate_cloud_msg)
        rate.sleep()
