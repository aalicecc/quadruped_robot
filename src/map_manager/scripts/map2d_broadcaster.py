#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import yaml
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class Map2dBroadcaster:

    def __init__(self):
        rospy.init_node('map2d_broadcaster', anonymous=None)

        # Parameter
        map_frame = rospy.get_param("~map_frame", "map")
        yaml_path = rospy.get_param("~yaml_path", "")
        trinary_flag = rospy.get_param("~trinary_flag", True)
        self.__map_origin = None
        self.__map_resolution = None
        self.__map_image = None
        self.__negate = None
        self.__free_thresh = None
        self.__occupied_thresh = None
        with open(yaml_path, 'r', encoding='utf-8') as f:
            yaml_parameters = yaml.safe_load(f)
            self.__map_origin = np.array(yaml_parameters['origin'])
            self.__map_resolution = yaml_parameters['resolution']
            self.__map_image = cv2.imread(yaml_parameters['image'],
                                          cv2.IMREAD_GRAYSCALE)
            self.__negate = yaml_parameters['negate']
            self.__free_thresh = yaml_parameters['resolution']
            self.__occupied_thresh = yaml_parameters['occupied_thresh']

        # Sensor Variable
        self.__new_flag = False
        self.__map_msg = OccupancyGrid()
        self.__map_msg.header.frame_id = map_frame

        self.__map_msg.info.resolution = self.__map_resolution
        self.__map_msg.info.width = self.__map_image.shape[1]
        self.__map_msg.info.height = self.__map_image.shape[0]

        self.__map_msg.info.origin.position.x = self.__map_origin[0]
        self.__map_msg.info.origin.position.y = self.__map_origin[1]
        self.__map_msg.info.origin.position.z = self.__map_origin[2]
        self.__map_msg.info.origin.orientation.w = 1.0
        self.__map_msg.info.origin.orientation.x = 0.0
        self.__map_msg.info.origin.orientation.y = 0.0
        self.__map_msg.info.origin.orientation.z = 0.0

        self.__map_msg.data = np.zeros(self.__map_msg.info.width *
                                       self.__map_msg.info.height,
                                       dtype=np.int8)
        raw_data = None
        if self.__negate:
            raw_data = 255 - self.__map_image
        else:
            raw_data = self.__map_image
        raw_data = (255.0 - raw_data) / 255.0
        for i in range(self.__map_msg.info.width):
            for j in range(self.__map_msg.info.height):
                curr_index = i + (self.__map_msg.info.height - j -
                                  1) * self.__map_msg.info.width
                if raw_data[j, i] > self.__occupied_thresh:
                    self.__map_msg.data[curr_index] = 100
                elif raw_data[j, i] < self.__free_thresh:
                    self.__map_msg.data[curr_index] = 0
                elif trinary_flag:
                    self.__map_msg.data[curr_index] = -1
                else:
                    ratio = (raw_data[j, i] - self.__free_thresh) / (
                        self.__occupied_thresh - self.__free_thresh)
                    self.__map_msg.data[curr_index] = 1 + ratio * 98

        # Subscriber
        self.__pose_sub = rospy.Subscriber("/ndt_pose", PoseStamped,
                                           self.__pose_callback)
        self.__pose_sub

        # Publisher
        self.__map_pub = rospy.Publisher("/map",
                                         OccupancyGrid,
                                         queue_size=1,
                                         latch=True)

    def __quat_to_rot_mat(self, quat):
        w = quat[0]
        x = quat[1]
        y = quat[2]
        z = quat[3]
        rot_mat = np.array([
            [
                1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w,
                2 * x * z + 2 * y * w
            ],
            [
                2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z,
                2 * y * z - 2 * x * w
            ],
            [
                2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w,
                1 - 2 * x * x - 2 * y * y
            ],
        ])
        return rot_mat

    def __pose_callback(self, msg):
        self.__map_msg.header.stamp = msg.header.stamp
        self.__map_msg.info.map_load_time = msg.header.stamp

        if self.__map_msg.header.frame_id == "odom":
            pose_trans = np.array(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            trans_in_map = self.__map_origin - pose_trans
            odom_to_map = self.__quat_to_rot_mat([
                msg.pose.orientation.w, -msg.pose.orientation.x,
                -msg.pose.orientation.y, -msg.pose.orientation.z
            ])
            trans_in_odom = np.dot(odom_to_map, trans_in_map)

            self.__map_msg.info.origin.position.x = trans_in_odom[0]
            self.__map_msg.info.origin.position.y = trans_in_odom[1]
            self.__map_msg.info.origin.position.z = trans_in_odom[2]
            self.__map_msg.info.origin.orientation.w = msg.pose.orientation.w
            self.__map_msg.info.origin.orientation.x = -msg.pose.orientation.x
            self.__map_msg.info.origin.orientation.y = -msg.pose.orientation.y
            self.__map_msg.info.origin.orientation.z = -msg.pose.orientation.z

        self.__new_flag = True

    def work(self):
        if self.__map_msg.header.frame_id == "odom":
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                if self.__new_flag:
                    self.__map_pub.publish(self.__map_msg)
                    self.__new_flag = False
                rate.sleep()
        else:
            self.__map_pub.publish(self.__map_msg)
            rospy.spin()


def main():
    map2d_broadcaster = Map2dBroadcaster()
    map2d_broadcaster.work()


if __name__ == '__main__':
    main()