#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64

# import numpy as np

from skros_service_bounding_box.bounding_box import say_it_works

if __name__ == '__main__':
    rospy.init_node('test_node')
    say_it_works()