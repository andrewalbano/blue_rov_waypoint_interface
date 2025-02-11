#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_matrix, quaternion_from_matrix, translation_from_matrix, quaternion_from_euler, quaternion_inverse, quaternion_matrix, translation_matrix, concatenate_matrices

import numpy as np


if __name__ =='__main__':
    # initialize node
    rospy.init_node("fixed_tf_broadcaster")
    rospy.loginfo("fixed_tf_broadcaster node has been started.")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    # global to map
    transform_matrix = np.array([
    [0,  1,  0,  0],
    [1,  0,  0,  0],
    [0,  0,  -1,  0],
    [0,  0,  0,  1]
    ])

    quaternion = quaternion_from_matrix(transform_matrix)
    translation = translation_from_matrix(transform_matrix)

    # defining map and ned frame relationship
    # Define the translation
    translation2 = (0, 0, 0)

    # Define the rotation in roll, pitch, yaw and convert to quaternion
    roll = 0
    pitch = 0
    yaw = 3.141592653589793  # 180 degrees in radians
    quaternion2 = quaternion_from_euler(roll, pitch, yaw)

    # Get the inverse transformation (from NED to map)
    t_ned2map = [0,0,0]
    q_ned2map = quaternion_inverse(quaternion2)
    T = translation_matrix(t_ned2map)
    R = quaternion_matrix(q_ned2map)
    transform = concatenate_matrices(R,T)
    rospy.loginfo(q_ned2map)
    rospy.loginfo(transform)



    

    # Transform the rotation
    # q_map = quaternion_multiply(q_ned2map, q_ned)

    while not rospy.is_shutdown():
        # br.sendTransform(translation, quaternion,rospy.Time.now(),'map', 'global_frame')
        # br.sendTransform(translation, quaternion,rospy.Time.now(),'map', 'global_frame')
        # br.sendTransform(translation2, quaternion2,rospy.Time.now(),'NED', 'map')



        rate.sleep()
    
   

