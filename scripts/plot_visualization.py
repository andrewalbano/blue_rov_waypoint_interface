#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
# from tf.transformations import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_matrix, euler_from_quaternion

global waypoints 
waypoints = []

def get_waypoints_callback(msg: PoseArray):
    # extract information from waypoints array
    waypoints = []  # erase the last waypoints received
    rospy.loginfo("received information")
    
    for waypoint in msg.poses:

        q = waypoint.orientation
        roll,pitch,yaw = euler_from_quaternion([q.x,q.y,q.z,q.w]) #,'sxyz')
        waypoints.append([waypoint.position.x,waypoint.position.y,waypoint.position.z, roll, pitch, yaw])
        print(waypoint)
    
    plot_3d(waypoints)
    

def plot_3d(waypoints):  
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1,projection='3d')


    elevation = -100
    azimuth = -153
    roll = 179
    # ax.view_init(elev=elevation, azim=azimuth, roll=roll)
    ax.view_init(elev=elevation, azim=azimuth)
    
    # Setting subplot details
    ax.set_title("target waypoints")
    ax.grid(True)
    ax.set_ylabel('y position')
    ax.set_xlabel('x position')
    ax.set_zlabel('depth')

    
    
    waypoints = np.array(waypoints)

    if waypoints.ndim > 1:
        # Unpack the x and y coordinates, and yaw values
        x, y, z, yaw = waypoints[:, 0], waypoints[:, 1],waypoints[:, 2], waypoints[:,5]
        
    else:
        x, y, z, yaw = waypoints[0], waypoints[1], waypoints[2], waypoints[5]

    # Plot the waypoints in a given subplot axis (ax)
    ax.plot(x, y, z, marker='o', linestyle='--', color = "b")
    axis_length = 0.1
    for wp in waypoints:
        R = euler_matrix(wp[3],wp[4],wp[5],'sxyz')
        # ax.quiver(wp[0], wp[1], wp[2], R[0, :], R[1, :], R[2, :],colors = ['red', 'green', 'blue'], length=axis_length, normalize=True)
        ax.quiver(wp[0], wp[1], wp[2], R[0, 0], R[1, 0], R[2, 0], colors = 'red', length=axis_length, normalize=True)

    
    # show the figure
    ax= plt.show()
    # note: need to close the plot after
    plt.savefig("waypoint_plot")

        
    
       
       




if __name__ =='__main__':
    # initialize node
    rospy.init_node("plot_visualizer")
    rospy.loginfo("plot_visualizer node has been started.")

    sub= rospy.Subscriber("waypoint_plot_visualization", PoseArray, callback= get_waypoints_callback)

    rospy.spin()