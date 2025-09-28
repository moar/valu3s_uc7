import unittest

import rospy

from valu3s_uc7.bvh_broadcaster import BVHBroadcaster
from valu3s_uc7.ulises_test_utils.test_utils import TestLaunchInfo, UlisesTestHelper, DiagnosisTestResult, MQTTMessage

from mock import patch

from os import listdir
from os.path import isfile, join
import time
from valu3s_uc7.task.task import Task
import copy
import random
import threading


def motion_thread(file_name, loops=1):
    for i in range(loops):
        motion(file_name)

def motion(file_name):
    bvh_test = BVHBroadcaster(file_name, 'world')
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, 'world'))
    bvh_test.broadcast(loop='')


class FridgeDisassemblyTest(unittest.TestCase):
    def test_frogak(self):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            # motion("src/valu3s_uc7/data/valu3s_generator/translate_cxk.bvh")
            # este lo podnria un poco mas adelante
            motion("src/valu3s_uc7/data/valu3s_generator/translate_door_opening.bvh")

            # este esta perfecto, un pelinadelante
            motion("src/valu3s_uc7/data/valu3s_generator/translate_door_closing.bvh")

            # este esta perfecto, un pelin adelante
            motion("src/valu3s_uc7/data/valu3s_generator/translate_fallen_drawer.bvh")

            # Este esta un pelin mas adelante
            motion("src/valu3s_uc7/data/valu3s_generator/translate_one_door.bvh")

            # Este esta un pelin mas adelantea
            motion("src/valu3s_uc7/data/valu3s_generator/translate_two_doors.bvh")

            # este moveria un pelinadelante y un pelin a la izquierda
            motion("src/valu3s_uc7/data/valu3s_generator/translate_waiting.bvh")

            #
            #motion("src/valu3s_uc7/data/valu3s_generator/translate_drawer_extraction_right.bvh")
            #motion("src/valu3s_uc7/data/valu3s_generator/translate_drawer_extraction_right.bvh")




            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()


    def test_uc7(self):

        print("starting...")
        time.sleep(1)
        print("started")

        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)



        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            working_thread = threading.Thread(target=motion_thread,
                                              args=("src/valu3s_uc7/data/valu3s_generator/translate_fallen_drawer.bvh",1,))
            working_thread.start()
            working_thread.join()
            helper.wait_assert_passed()



if __name__ == '__main__':
    unittest.main()
