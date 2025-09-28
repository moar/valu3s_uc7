#!/usr/bin/env python  
import rospy
import tf
from std_msgs.msg import Float64

import numpy as np

if __name__ == '__main__':

    
    rospy.init_node('measure_distances_node')

    # get skeletons topics
    from_tf = rospy.get_param("~from_tf", "")
    to_tf = rospy.get_param("~to_tf", "")
    distance = rospy.get_param("~distance", "")

    print("")
    print("Joints names")
    print("  |- from_tf:", from_tf)
    print("  |- to_tf:", to_tf)
    print("")
    print("publications")
    print("  |- distance:", distance)  

    pub = rospy.Publisher(distance, Float64, queue_size=10)  

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(from_tf, to_tf, rospy.Time(0))
            distance = np.linalg.norm(trans)
            #str = "Distance between the hand and the world is = {0:f}".format(distance)
            
            #rospy.loginfo("Distance from " + from_tf + " to " + to_tf + " is " + str(distance))
            pub.publish(distance)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()