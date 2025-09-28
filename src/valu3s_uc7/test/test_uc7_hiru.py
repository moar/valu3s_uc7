#!/usr/bin/env python

import unittest

import rospy

from skros_task_valu3s.robot_control import control_robot
from valu3s_uc7.bvh_broadcaster import BVHBroadcaster
from valu3s_uc7.ulises_test_utils.test_utils import TestLaunchInfo, UlisesTestHelper, DiagnosisTestResult, MQTTMessage

from mock import patch

from os import listdir
from os.path import isfile, join
import os

import time
#from valu3s_uc7.task.task import Task
from skros_task_valu3s.robot_control import Task

import copy
import random
import threading

PKG = 'valu3s_uc7'

def motion_thread(file_name, loops=1):
    for i in range(loops):
        motion(file_name)

def motion(file_name):
    bvh_test = BVHBroadcaster(file_name, 'world')
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, 'world'))
    bvh_test.broadcast(loop='')


class FridgeDisassemblyTest(unittest.TestCase):

    def test_fridge_dissasembly(self):
        print("starting...")
        task = Task()
        time.sleep(1)
        print("started")

        # rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)




        rospy.loginfo("KARPETA BASEEEEE:"+os.getcwd())

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            initial_waiting_thread = threading.Thread(target=motion_thread,
                                              args=("data/valu3s_generator/translate_waiting.bvh",1,))

            initial_waiting_thread.start()

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
            time.sleep(6)
            initial_waiting_thread.join()
            # open fridge doors
            print(" - open fridge doors")
            task.control_fridge_doors("fridge_00", ["up", "down"], True)
            time.sleep(3)

            # do some task with the robot
            print(" - robot")



            # bvh
            #open_thread = threading.Thread(target=motion_thread, args=("src/valu3s_uc7/data/valu3s_generator/translate_fallen_drawer.bvh",))
            # aqui secuencial
            # motion("src/valu3s_uc7/data/valu3s_generator/translate_fallen_drawer.bvh")
            working_thread = threading.Thread(target=motion_thread,
                                              args=("data/valu3s_generator/translate_fallen_drawer.bvh",1,))
            working_thread.start()
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
            working_thread.join()
            waiting_thread = threading.Thread(target=motion_thread,
                                              args=(
                                                  "data/valu3s_generator/translate_waiting.bvh",1,))
            waiting_thread.start()
            waiting_thread.join()
            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()



if __name__ == '__main__':
    # unittest.main()
    import rostest
    print("ESPERANDO.........")
    time.sleep(20)
    print("COMINEZA.........")
    rostest.rosrun(PKG, 'test_Fridge_dissassembly_suite', FridgeDisassemblyTest)
