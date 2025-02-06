#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler,quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox

last_waypoint_transform = np.eye(4)
print(last_waypoint_transform)

# move 1 in x
x,y,z = 1, 0, 0 
roll, pitch, yaw = 0,0,0
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(rot_matrix,trans_matrix)
print(transform)

print("\nNext transform\n")
x,y,z = 0, 0, 0 
roll, pitch, yaw = 0,0,np.pi/2
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(transform, rot_matrix,trans_matrix)

# transform = concatenate_matrices(rot_matrix,trans_matrix)
print(transform)


print("\nNext transform\n")
x,y,z = 1,0, 0 
roll, pitch, yaw = 0,0,0
trans_matrix = translation_matrix([x,y,z])
rot_matrix = euler_matrix(roll,pitch,yaw)
print(rot_matrix)
transform = concatenate_matrices(transform, rot_matrix,trans_matrix)
print(transform)



x,y,z = 1, 0, 0 
roll, pitch, yaw = 0,0, np.pi/2
print(yaw)
q = quaternion_from_euler(roll,pitch,yaw)
roll,pitch,yaw = euler_from_quaternion(q) 

print(yaw)
