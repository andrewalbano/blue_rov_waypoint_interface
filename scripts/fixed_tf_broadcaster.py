#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_matrix, quaternion_from_matrix, translation_from_matrix

import numpy as np


if __name__ =='__main__':
    # initialize node
    rospy.init_node("fixed_tf_broadcaster")
    rospy.loginfo("fixed_tf_broadcaster node has been started.")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    transform_matrix = np.array([
    [0,  1,  0,  0],
    [1,  0,  0,  0],
    [0,  0,  -1,  0],
    [0,  0,  0,  1]
    ])

    quaternion = quaternion_from_matrix(transform_matrix)
    translation = translation_from_matrix(transform_matrix)

    while not rospy.is_shutdown():
        br.sendTransform(translation, quaternion,rospy.Time.now(),"map_frame", "global_frame")
        rate.sleep()
    
   
