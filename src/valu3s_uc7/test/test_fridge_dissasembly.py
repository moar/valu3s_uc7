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

    def test_froga(self):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            file_name = "src/valu3s_uc7/data/door_opening.bvh"
            frame_name = "world"
            loop = ''
            bvh_test = BVHBroadcaster(file_name, frame_name)
            rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, frame_name))
            for i in range(1):
                rospy.loginfo("Motion started ::" + str(i))
                bvh_test = BVHBroadcaster(file_name, frame_name)
                bvh_test.broadcast(loop='')

            rospy.loginfo("Broadcasting done")
            helper.wait_assert_passed()

    @patch('ulises_test_utils.test_utils.Client', autospec=True)
    def test_froga_mqtt_mock(self, mqtt_client):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            file_name = "src/valu3s_uc7/data/door_opening.bvh"
            frame_name = "world"
            loop = ''
            bvh_test = BVHBroadcaster(file_name, frame_name)
            rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, frame_name))
            for i in range(3):
                rospy.loginfo("Motion started ::" + str(i))
                bvh_test = BVHBroadcaster(file_name, frame_name)
                bvh_test.broadcast(loop='')

            rospy.loginfo("Broadcasting done")

            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()

    def test_door_opening(self):
        self.assertEqual(True, False)  # add assertion here

    def test_long_door_opening(self):
        self.assertEqual(True, False)  # add assertion here

    def test_door_closing(self):
        self.assertEqual(True, False)  # add assertion here

    def test_door_closing(self):
        self.assertEqual(True, False)  # add assertion here

    def test_walking_around(self):
        self.assertEqual(True, False)  # add assertion here

    def test_door_waiting(self):
        self.assertEqual(True, False)  # add assertion here

    def test_frogak_bi(self):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            # file_name = "src/valu3s_uc7/data/walk_90y.bvh"
            # file_name = "src/valu3s_uc7/data/walk_combined_3_motions_Y.bvh"
            # file_name = "src/valu3s_uc7/data/walk_rotate_points_nire.bvh"
            # file_name = "src/valu3s_uc7/data/generator/demo_group_1_120.bvh"
            # file_name = "src/valu3s_uc7/data/generator/demo_test_2_120.bvh"
            file_name = "src/valu3s_uc7/data/door_opening.bvh"
            bvh_test = BVHBroadcaster(file_name, 'world')
            rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, 'world'))
            bvh_test.broadcast(loop='')

            file_name = "src/valu3s_uc7/data/door_opening.bvh"
            bvh_test = BVHBroadcaster(file_name, 'world')
            rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, 'world'))
            bvh_test.broadcast(loop='')
            file_name = "src/valu3s_uc7/data/valu3s_generator/translate_door_opening.bvh"
            bvh_test = BVHBroadcaster(file_name, 'world')
            rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, 'world'))
            bvh_test.broadcast(loop='')
            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()

    def test_frogak(self):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123,  situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            # motion("src/valu3s_uc7/data/valu3s_generator/translate_cxk.bvh")
            # este lo podnria un poco mas adelante
            motion("src/valu3s_uc7/data/valu3s_generator/translate_door_opening.bvh")
            motion("src/valu3s_uc7/data/valu3s_generator_uc7_2000/translate_door_opening.bvh")
            motion("src/valu3s_uc7/data/valu3s_generator_uc7_1000/translate_door_opening.bvh")
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
        task = Task()
        time.sleep(1)
        print("started")

        # rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)




        def control_robot(task):
            p = [0.2, 0.2, 0.8]
            q = [-0.6408564, 0.6408564, -0.2988362, 0.2988362]
            pose = task.robot.get_cartesian_pose("arm")
            pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
            task.robot.reach_cartesian_pose("arm", pose)
            original_pose = task.robot.get_cartesian_pose("arm")

            for j in range(2):
                for i in range(5):
                    pose = copy.deepcopy(original_pose)
                    pose.position.y += (random.random() * 2 - 1) * 0.1
                    task.robot.reach_cartesian_pose("arm", pose)

                task.robot.reach_named_pose("arm", "up")

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            initial_waiting_thread = threading.Thread(target=motion_thread,
                                              args=("src/valu3s_uc7/data/valu3s_generator/translate_waiting.bvh",1,))
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
                                              args=("src/valu3s_uc7/data/valu3s_generator/translate_fallen_drawer.bvh",1,))
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
                                                  "src/valu3s_uc7/data/valu3s_generator/translate_waiting.bvh",1,))
            waiting_thread.start()
            waiting_thread.join()
            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()

    def test_karpeta_uc7(self):
        rospy.init_node('Test_Froga')
        test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
        fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                               step_name='test', constraints='test', eval_result=True)

        with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
            # motion("src/valu3s_uc7/data/valu3s_generator/translate_cxk.bvh")
            motions_path = 'src/valu3s_uc7/data/valu3s_generator_uc7'
            motion_list = [f for f in listdir(motions_path) if isfile(join(motions_path, f))]
            length = len(motion_list)
            for i in range(length - 1):
                motion(motions_path + '/' + motion_list[i])

            helper.__test_result_message_callback__(client=None, userdata=None,
                                                    message=MQTTMessage(topic=helper.result_topic,
                                                                        payload=bytes(fake_test_result.as_json())))
            helper.wait_assert_passed()

    def test_danak(self):
            rospy.init_node('Test_Froga')
            test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
            fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                                   step_name='test', constraints='test', eval_result=True)

            with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
                # motion("src/valu3s_uc7/data/valu3s_generator/translate_cxk.bvh")
                # este lo podnria un poco mas adelante
                motions_path = 'src/valu3s_uc7/data/valu3s_generator'
                motion_list = [f for f in listdir(motions_path) if isfile(join(motions_path, f))]
                length = len(motion_list)
                for i in range(length - 1):
                    motion(motions_path + '/' + motion_list[i])


                helper.__test_result_message_callback__(client=None, userdata=None,
                                                        message=MQTTMessage(topic=helper.result_topic,
                                                                            payload=bytes(fake_test_result.as_json())))
                helper.wait_assert_passed()


if __name__ == '__main__':
    unittest.main()
