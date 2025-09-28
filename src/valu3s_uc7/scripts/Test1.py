import rospy


from valu3s_uc7.bvh_broadcaster import BVHBroadcaster
from valu3s_uc7.ulises_test_utils.test_utils import TestLaunchInfo, UlisesTestHelper, DiagnosisTestResult,MQTTMessage


from mock import patch



def test_usage_mock_gabe():
    test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')

    with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
        froga_test_behaviour()
        helper.wait_assert_passed()


@patch('ulises_test_utils.test_utils.Client', autospec=True)
def test_usage(mqtt_client):
    test_info = TestLaunchInfo(observation_model_name='', task_id='test', test_id='id')
    fake_test_result = DiagnosisTestResult(timestamp=123, phase_name='test', situation_name='test',
                                           step_name='test', constraints='test', eval_result=True)

    with UlisesTestHelper(mqtt_host="172.17.6.163", test_launch_info=test_info, ignore_ack=True) as helper:
        froga_test_behaviour()
        helper.__test_result_message_callback__(client=None, userdata=None,
                                                message=MQTTMessage(topic=helper.result_topic,
                                                                    payload=bytes(fake_test_result.as_json())))

        helper.wait_assert_passed()


def froga_test_behaviour():
    rospy.init_node('test-froga')
    file_name = "src/valu3s_uc7/data/door_opening.bvh"
    frame_name = "world"
    loop = ''
    bvh_test = BVHBroadcaster(file_name, frame_name)
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s" % (file_name, frame_name))
    rospy.loginfo("Only once")
    for i in range(10):
        # rospy.loginfo("Only once::"+str(i))
        bvh_test = BVHBroadcaster(file_name, frame_name)
        bvh_test.broadcast(loop='')

    rospy.loginfo("Broadcasting done")


def main():
    test_usage()
    test_usage_mock_gabe()


if __name__ == "__main__":
    main()
