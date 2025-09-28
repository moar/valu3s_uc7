import rospy
import tf
import numpy as np
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker


def get_bounding_box(points):
    # Transform the points to the global frame
    listener = tf.TransformListener()
    listener.waitForTransform('/Hip', '/Neck', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/Hip', '/Neck', rospy.Time(0))
    trans = np.array(trans)
    rot = np.array(rot)
    rotation_matrix = tf.transformations.quaternion_matrix(rot)
    transformation_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
                                                                    rotation_matrix)
    transformed_points = np.dot(transformation_matrix, np.vstack([points.T, np.ones(len(points))]))
    transformed_points = transformed_points[:3, :].T

    # Calculate the bounding box
    min_point = np.min(transformed_points, axis=0)
    max_point = np.max(transformed_points, axis=0)
    center = (min_point + max_point) / 2
    size = max_point - min_point

    # Publish the bounding box marker
    marker = Marker()
    marker.header.frame_id = "/Hip"
    marker.type = marker.CUBE
    marker.pose.position = Point(center[0], center[1], center[2])
    marker.scale = Vector3(size[0], size[1], size[2])
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker_pub.publish(marker)

'''
def point_cloud_callback(msg):
    # Convert the point cloud message to a numpy array
    points = np.array([(msg.data[i], msg.data[i + 1], msg.data[i + 2]) for i in range(0, len(msg.data), 12)])
    get_bounding_box(points)
'''
def tf_callback(msg):
    for transform in msg.transforms:
        if transform.child_frame_id == "/camera_frame":
            trans = transform.transform.translation
            rot = transform.transform.rotation
            size = np.array([0.2, 0.1, 0.05]) # Example size, replace with your own calculation
            get_bounding_box(trans, rot, size)

if __name__ == '__main__':
    rospy.init_node('bounding_box_calculator')
    marker_pub = rospy.Publisher('/bounding_box', Marker, queue_size=10)

    # Subscribe to the point cloud topic
    #rospy.Subscriber('/camera/point_cloud', PointCloud2, point_cloud_callback)
    # Subscribe to the TF topic
    rospy.Subscriber('/tf', tf.msg.tfMessage, tf_callback)

    rospy.spin()

