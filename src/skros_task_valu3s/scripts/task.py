#!/usr/bin/python

import time
import copy
import random
import numpy as np

import rospy
import sensor_msgs.msg

import group_interface

class Task:
    def __init__(self):

        self.robot = group_interface.GroupInterface(init_node=False)
        
        # conveyors
        self._pub_conveyor_00 = rospy.Publisher('/conveyor_00/joint_command', sensor_msgs.msg.JointState, queue_size=1)
        self._pub_conveyor_01 = rospy.Publisher('/conveyor_01/joint_command', sensor_msgs.msg.JointState, queue_size=1)
        self._pub_conveyor_02 = rospy.Publisher('/conveyor_02/joint_command', sensor_msgs.msg.JointState, queue_size=1)
        self._pub_conveyor_03 = rospy.Publisher('/conveyor_03/joint_command', sensor_msgs.msg.JointState, queue_size=1)

        self._pub_fridge_00 = rospy.Publisher('/fridge_00/joint_command', sensor_msgs.msg.JointState, queue_size=1)

        # lasers
        self._conveyor_01_laser = 2 ** 31
        self._conveyor_02_laser = 2 ** 31
        rospy.Subscriber("/conveyor_01/laser_scan", sensor_msgs.msg.LaserScan, self._sub_conveyor_01_laser_scan)
        rospy.Subscriber("/conveyor_02/laser_scan", sensor_msgs.msg.LaserScan, self._sub_conveyor_02_laser_scan)
        
        rospy.init_node('task', anonymous=False)

    def _sub_conveyor_01_laser_scan(self, msg):
        self._conveyor_01_laser = msg.ranges[0]

    def _sub_conveyor_02_laser_scan(self, msg):
        self._conveyor_02_laser = msg.ranges[0]

    def read_sensor(self, conveyor, sensor="laser"):
        if conveyor == "conveyor_01":
            tmp = self._conveyor_01_laser
            self._conveyor_01_laser = 2 ** 31
        elif conveyor == "conveyor_02":
            tmp = self._conveyor_02_laser
            self._conveyor_02_laser = 2 ** 31
        else:
            raise ValueError("Invalid conveyor {}".format(conveyor))
        return tmp

    def control_conveyor(self, conveyor, move=True, direction=1):
        msg = sensor_msgs.msg.JointState()
        rate = rospy.Rate(30)
        velocity = 10 if move else 0

        # prepare message
        if conveyor == "conveyor_00":
            msg.name = ["RevoluteJoint", "RevoluteJoint0", "RevoluteJoint1", "RevoluteJoint2", "RevoluteJoint3"]
            publisher = self._pub_conveyor_00
        elif conveyor == "conveyor_01":
            msg.name = ["RevoluteJoint", "RevoluteJoint0", "RevoluteJoint1", "RevoluteJoint2", "RevoluteJoint3", 
                        "RevoluteJoint4", "RevoluteJoint5", "RevoluteJoint6"]
            publisher = self._pub_conveyor_01
        elif conveyor == "conveyor_02":
            msg.name = ["RevoluteJoint", "RevoluteJoint0", "RevoluteJoint1", "RevoluteJoint2", "RevoluteJoint3", 
                        "RevoluteJoint4", "RevoluteJoint5", "RevoluteJoint6"]
            publisher = self._pub_conveyor_02
        elif conveyor == "conveyor_03":
            msg.name = ["RevoluteJoint", "RevoluteJoint0", "RevoluteJoint1", "RevoluteJoint2", "RevoluteJoint3", 
                        "RevoluteJoint4", "RevoluteJoint5", "RevoluteJoint6", "RevoluteJoint7", "RevoluteJoint8", 
                        "RevoluteJoint9", "RevoluteJoint10", "RevoluteJoint11", "RevoluteJoint12"]
            publisher = self._pub_conveyor_03
        else:
            raise ValueError("Invalid conveyor {}".format(conveyor))

        # publish message
        msg.velocity = [-direction * velocity] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()

    def control_piston(self, pistons, open=True):
        msg = sensor_msgs.msg.JointState()
        rate = rospy.Rate(30)
        positon = -0.65 if open else 0

        # prepare message
        if pistons == "pistons_01":
            msg.name = ["joint_piston1", "joint_piston2", "joint_piston3"]
            publisher = self._pub_conveyor_01
        elif pistons == "pistons_02":
            publisher = self._pub_conveyor_02
            msg.name = ["joint_piston1", "joint_piston2", "joint_piston3"]
        else:
            raise ValueError("Invalid pistons {}".format(pistons))

        # publish message
        msg.position = [positon] * len(msg.name)
        publisher.publish(msg)
        rate.sleep()

    def control_fridge_doors(self, fridge, doors, open=True):
        msg = sensor_msgs.msg.JointState()
        rate = rospy.Rate(30)
        positon = 200 * np.pi / 180.0 if open else 0

        # prepare message
        msg.name = [door for door in doors]
        msg.position = [positon for door in doors]

        if fridge == "fridge_00":
            publisher = self._pub_fridge_00
        else:
            raise ValueError("Invalid fridge {}".format(fridge))

        # publish message
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":

    print("starting...")
    task = Task()
    time.sleep(1)
    print("started")

    def control_robot(task):
        p = [0.2, 0.2, 0.8]
        q = [ -0.6408564, 0.6408564, -0.2988362, 0.2988362 ]
        pose = task.robot.get_cartesian_pose("arm")
        pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
        task.robot.reach_cartesian_pose("arm", pose)

        original_pose = task.robot.get_cartesian_pose("arm")
        
        for i in range(5):
            pose = copy.deepcopy(original_pose)
            pose.position.y += (random.random() * 2 - 1) * 0.1
            task.robot.reach_cartesian_pose("arm", pose)

        task.robot.reach_named_pose("arm", "up")

    # open pistons
    print(" - open pistons")
    task.control_piston("pistons_01", True)
    task.control_piston("pistons_02", True)
    time.sleep(1)

    # move conveyors
    print(" - move conveyors")
    task.control_conveyor("conveyor_00", True)
    task.control_conveyor("conveyor_01", True)
    task.control_conveyor("conveyor_02", True)
    task.control_conveyor("conveyor_03", True)

    print(" - laser scanning...")
    timeout = 15
    start_time = time.time()
    while True:
        if task.read_sensor("conveyor_01") < 1.5:
            break
        elif time.time() - start_time >= timeout:
            print("[WARNING] laser scanning timeout")
            break
        time.sleep(0.05)

    # stop conveyors
    print(" - stop conveyors")
    task.control_conveyor("conveyor_00", False)
    task.control_conveyor("conveyor_01", False)
    task.control_conveyor("conveyor_02", False)
    task.control_conveyor("conveyor_03", False)
    time.sleep(1)

    # close pistons
    print(" - close pistons")
    task.control_piston("pistons_01", False)
    task.control_piston("pistons_02", False)
    time.sleep(5)

    # open fridge doors
    print(" - open fridge doors")
    task.control_fridge_doors("fridge_00", ["up", "down"], True)
    time.sleep(3)

    # do some task with the robot
    print(" - robot")
    control_robot(task)

    # close fridge doors
    print(" - close fridge doors")
    task.control_fridge_doors("fridge_00", ["up", "down"], False)
    time.sleep(3)

    # open pistons
    print(" - open pistons")
    task.control_piston("pistons_01", True)
    task.control_piston("pistons_02", True)
    time.sleep(1)

    # move conveyors
    print(" - move conveyors")
    task.control_conveyor("conveyor_00", True)
    task.control_conveyor("conveyor_01", True)
    task.control_conveyor("conveyor_02", True)
    task.control_conveyor("conveyor_03", True)
    time.sleep(4.5)

    # # rate = rospy.Rate(10) # 10hz
    # # while not rospy.is_shutdown():
    # #     task.control_conveyor("conveyor_00", True)
    # print("spinning...")
    # # rospy.spin()
