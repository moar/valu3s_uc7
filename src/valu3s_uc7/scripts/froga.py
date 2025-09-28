#!/usr/bin/env python
import tf
import math
import numpy
import rospy
import string
import argparse
from valu3s_uc7.bvh_broadcaster import BVHBroadcaster

from valu3s_uc7.ulises_test_utils.test_utils import TestLaunchInfo, UlisesTestHelper, DiagnosisTestResult
from valu3s_uc7.ulises_test_utils.test_utils import TestLaunchAck,MQTTMessage

from mock import patch




def test_usage_mock_gabe():
    test_info = TestLaunchInfo(observation_model_name='Model Aitor 1', task_id='Fridge Dissasembly with Human Door Opening', test_id='2')
    
    with UlisesTestHelper(test_launch_info=test_info, ignore_ack=True) as helper:
        froga()
        helper.wait_assert_passed()

@patch('ulises_test_utils.test_utils.Client', autospec=True)
def test_usage(mqtt_client):
    test_info = TestLaunchInfo(observation_model_name='Model Aitor 2', task_id='Fridge Dissasembly', test_id='1')
    fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='Door opening', situation_name='Door Opening',
                                            step_name='Door Opening', constraints='test', eval_result=True)

    with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
        froga()
        helper.__test_result_message_callback__(client=None, userdata=None,
                                                message=MQTTMessage(topic=helper.result_topic,
                                                                    payload=bytes(fake_test_result.as_json())))


        helper.wait_assert_passed()


@patch('ulises_test_utils.test_utils.Client', autospec=True)
def test_failed( mqtt_client):
    test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
    test_info_ack = TestLaunchAck(test_id='id', message='ACK')
    fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                            step_name='test', constraints='test', eval_result=False)


    with UlisesTestHelper(test_launch_info=test_info, ignore_ack=True) as helper:
        helper.__test_result_message_callback__(client=None, userdata=None,
                                                message=MQTTMessage(topic=helper.result_topic,
                                                                    payload=bytes(fake_test_result.as_json())))
        froga()
        helper.wait_assert_passed()

@patch('ulises_test_utils.test_utils.Client', autospec=True)
def test_failed_assert_on_exit(mqtt_client):
    test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
    fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                            step_name='test', constraints='test', eval_result=False)

    with UlisesTestHelper(test_launch_info=test_info, assert_on_exit=True, ignore_ack=True) as helper:
        helper.__test_result_message_callback__(client=None, userdata=None,
                                                message=MQTTMessage(topic=helper.result_topic,
                                                                    payload=bytes(fake_test_result.as_json())))
        froga()


def argsparser():
    parser = argparse.ArgumentParser("python BVHBroadcaster.py")
    parser.add_argument('bvh_file', help="A path to bvh file that you want to broadcast")
    parser.add_argument('base_frame', help="An existing frame in rviz on which the skeleton will be loaded")
    parser.add_argument('-n', '--name', help="Node name, default: BVHBroadcaster", default="BVHBroadcaster")
    parser.add_argument('-l', '--loop', help="Loop broadcasting", action="store_true")
    return parser.parse_args()



def froga():
    rospy.init_node(args.name)
    file_name = "src/valu3s_uc7/data/door_opening.bvh"
    frame_name = "world"
    loop=''
    bvh_test = BVHBroadcaster(file_name, frame_name)
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, frame_name))
    rospy.loginfo("Only once")
    bvh_test.broadcast(loop=loop)
    rospy.loginfo("Broadcasting done")


def main(args):
    rospy.init_node(args.name)
    test_usage()

    #test_failed_assert_on_exit()
    #rospy.loginfo('ejecutado lo de Gorka con failed')

    test_usage_mock_gabe()
    rospy.loginfo('ejecutado lo de Gorka sin respuesta')

    # file_name = "/home/mingfei/Documents/RobotManipulationProject/mocap/62/62_07.bvh"
    bvh_test = BVHBroadcaster(args.bvh_file, args.base_frame)
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s"%(args.bvh_file, args.base_frame))
    if args.loop:
        rospy.loginfo("Loop")
    else: 
        rospy.loginfo("Only once")
    bvh_test.broadcast(loop=args.loop)
    rospy.loginfo("Broadcasting done")


if __name__ == "__main__":
    args = argsparser()
    main(args)





