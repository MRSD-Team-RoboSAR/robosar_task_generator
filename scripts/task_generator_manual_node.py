#!/usr/bin/env python3
# Created by Indraneel on 21/03/22

import rospy
from robosar_messages.srv import *

import numpy as np
from geometry_msgs.msg import PointStamped
import math
import threading

# Global variables
num_waypoints = 0
waypoints_array = []
got_all_waypoints = False
evt = threading.Event()

def handle_taskgen_getwaypts(req):
    global waypoints_array, got_all_waypoints, evt, num_waypoints
    # Extract map from req
    map_width = req.map.info.width
    map_height = req.map.info.height
    map_data = req.map.data
    map_origin_x = req.map.info.origin.position.x
    map_origin_y = req.map.info.origin.position.y
    map_resolution = req.map.info.resolution
    threshold = req.threshold
    iterations = req.iterations

    # Wait for all waypoints
    rospy.loginfo('Waiting for waypoints!')
    timeout = 100
    evt.wait(timeout)
    rospy.loginfo('Received %d waypoints',num_waypoints)

    # Convert waypoints to pixel coordinates
    waypoints= []
    for waypoint in waypoints_array:
        pixel_x = (int)((waypoint[0]-map_origin_x)/map_resolution)
        pixel_y = (int)((waypoint[1]-map_origin_y)/map_resolution)
        waypoints.append(pixel_x)
        waypoints.append(pixel_y)

    # Return
    num_pts = num_waypoints
    dims = 2
    waypts = waypoints #waypoints[:,0:2].flatten()
    return taskgen_getwayptsResponse(num_pts, dims, waypts)

def callback(data):
    global num_waypoints,waypoints_array,got_all_waypoints,evt

    num_waypoints = num_waypoints + 1
    rospy.loginfo("Received waypoint %d",num_waypoints)
    waypoints_array.append((data.point.x,data.point.y))

    # Check if received all waypoints
    threshold = 0.1
    if(num_waypoints>1):
        # Get distance betwen subsequent waypoints
        distance = math.hypot(waypoints_array[num_waypoints-1][0]-waypoints_array[num_waypoints-2][0],
                        waypoints_array[num_waypoints-1][1]-waypoints_array[num_waypoints-2][1])
        if(distance<threshold):
            evt.set()

    

def task_generator_server():
    rospy.init_node('task_generator_server')
    s = rospy.Service('taskgen_getwaypts', taskgen_getwaypts, handle_taskgen_getwaypts)
    rospy.Subscriber("clicked_point", PointStamped, callback)
    print("Ready to generate waypts")
    rospy.spin()

if __name__ == "__main__":
    # from matplotlib import pyplot as plt
    task_generator_server()