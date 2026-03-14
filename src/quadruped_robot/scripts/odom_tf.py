import rospy
import tf
import tf.transformations
import numpy as np
from nav_msgs.msg import Odometry


class OdomTf:

    def __init__(self):
        rospy.init_node('odom_tf')

        trans_array = rospy.get_param("~sensor_in_base",
                                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

        self.__sensor_in_base = self.__parts_to_transform(
            trans_array[:3], trans_array[3:])
        self.__base_in_sensor = np.linalg.inv(self.__sensor_in_base)

        self.__br = tf.TransformBroadcaster()
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)

    def work(self):
        rospy.spin()

    def __parts_to_transform(self, pos, quat):
        px, py, pz = pos
        qw, qx, qy, qz = quat
        trans = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
        trans[:3, 3] = np.array([px, py, pz])
        return trans

    def __transform_to_parts(self, trans):
        pos = tf.transformations.translation_from_matrix(trans)
        ori = tf.transformations.quaternion_from_matrix(trans)
        quat = [ori[3], ori[0], ori[1], ori[2]]
        return pos, quat

    def __odom_callback(self, msg):
        self.__br = tf.TransformBroadcaster()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        sensor_in_odom = self.__sensor_in_base @ self.__parts_to_transform(
            [pos.x, pos.y, pos.z], [ori.w, ori.x, ori.y, ori.z])
        base_in_odom = sensor_in_odom @ self.__base_in_sensor

        pos, ori = self.__transform_to_parts(base_in_odom)
        self.__br.sendTransform(
            (pos[0], pos[1], pos[2]),
            (ori[1], ori[2], ori[3], ori[0]),
            msg.header.stamp,
            "base_link",
            "odom",
        )


if __name__ == '__main__':
    odom_tf = OdomTf()
    odom_tf.work()
