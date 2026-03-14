#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import json
import math
import numpy as np
import open3d as o3d

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utils import MAP_DIR, RAW_TRANS_FILE, BASE_TRANS_FILE, JSON_FILE
from utils import SENSOR_IN_BASE, BASE_IN_SENSOR
from utils import TRANS_MIN_GOAL_DIST
from utils import pcd_to_trans, trans_to_pcd


class TransModifier:

    def __init__(
        self,
        raw_trans_path,
        base_trans_path,
        json_path,
        sensor_in_base,
        base_in_sensor,
        min_goal_dist,
    ):
        self.__raw_trans_path = raw_trans_path
        self.__base_trans_path = base_trans_path
        self.__json_path = json_path
        self.__sensor_in_base = sensor_in_base
        self.__base_in_sensor = base_in_sensor

        print(f"read trans: {self.__raw_trans_path}")
        self.__raw_pcd = o3d.t.io.read_point_cloud(self.__raw_trans_path)
        self.__out_pcd = o3d.t.io.read_point_cloud(self.__raw_trans_path)
        trans_shape = self.__out_pcd.point["positions"].numpy().shape
        print(f"trans shape: {trans_shape[0]}")

        self.__out_json = dict()
        self.__out_json["min_goal_dist"] = min_goal_dist
        self.__out_json["frame_id"] = "map"
        self.__out_json["goal_count"] = trans_shape[0]
        self.__out_json["goal"] = []

    def __pcd_transform(self):
        pos = self.__out_pcd.point["positions"].numpy()
        roll = self.__raw_pcd.point["roll"].numpy()
        pitch = self.__raw_pcd.point["pitch"].numpy()
        yaw = self.__raw_pcd.point["yaw"].numpy()

        for i in range(pos.shape[0]):
            raw_tarns = pcd_to_trans(pos[i],
                                     [roll[i][0], pitch[i][0], yaw[i][0]])
            base_trans = self.__sensor_in_base @ raw_tarns @ self.__base_in_sensor
            base_pos, base_euler = trans_to_pcd(base_trans)
            pos[i] = np.asarray(base_pos)
            roll[i] = base_euler[0]
            pitch[i] = base_euler[1]
            yaw[i] = base_euler[2]

        self.__out_pcd.point["positions"] = pos
        self.__out_pcd.point["roll"] = roll
        self.__out_pcd.point["pitch"] = pitch
        self.__out_pcd.point["yaw"] = yaw

    def __pcd_to_json(self):
        position = self.__out_pcd.point["positions"].numpy()
        yaw = self.__out_pcd.point["yaw"].numpy()

        last_goal = dict()
        last_goal["num"] = 1
        last_goal["pos_x"] = float(position[0][0])
        last_goal["pos_y"] = float(position[0][1])
        last_goal["pos_r"] = float(yaw[0][0])
        self.__out_json["goal"].append(last_goal)

        for index in range(self.__out_json["goal_count"]):
            goal = dict()
            goal["num"] = last_goal["num"] + 1
            goal["pos_x"] = float(position[index][0])
            goal["pos_y"] = float(position[index][1])
            goal["pos_r"] = float(yaw[index][0])

            delta_x = goal["pos_x"] - last_goal["pos_x"]
            delta_y = goal["pos_y"] - last_goal["pos_y"]
            delta_yaw = np.fabs(goal["pos_r"] - last_goal["pos_r"])
            distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
            if distance > 3.0 or delta_yaw > 0.5:
                self.__out_json["goal"].append(goal)
                last_goal = goal.copy()

        final_goal = dict()
        final_index = self.__out_json["goal_count"] - 1
        final_goal["num"] = last_goal["num"] + 1
        final_goal["pos_x"] = float(position[final_index][0])
        final_goal["pos_y"] = float(position[final_index][1])
        final_goal["pos_r"] = float(yaw[final_index][0])
        delta_x = final_goal["pos_x"] - last_goal["pos_x"]
        delta_y = final_goal["pos_y"] - last_goal["pos_y"]
        delta_yaw = np.fabs(final_goal["pos_r"] - last_goal["pos_r"])
        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        if distance > 0.2 or delta_yaw > 0.1:
            self.__out_json["goal"].append(final_goal)
            last_goal = final_goal.copy()

        self.__out_json["goal_count"] = last_goal["num"]

    def work(self):
        self.__pcd_transform()
        self.__pcd_to_json()

        with open(self.__json_path, 'w') as file:
            json.dump(self.__out_json, file)

        o3d.t.io.write_point_cloud(self.__base_trans_path, self.__out_pcd)


def main():
    print(222)
    map_name = "default"
    if len(sys.argv) > 1:
        map_name = sys.argv[1]

    raw_trans_path = f"{MAP_DIR}/{map_name}/{RAW_TRANS_FILE}"
    base_trans_path = f"{MAP_DIR}/{map_name}/{BASE_TRANS_FILE}"
    json_path = f"{MAP_DIR}/{map_name}/{JSON_FILE}"
    print(111)
    trans_modifier = TransModifier(
        raw_trans_path=raw_trans_path,
        base_trans_path=base_trans_path,
        json_path=json_path,
        sensor_in_base=SENSOR_IN_BASE,
        base_in_sensor=BASE_IN_SENSOR,
        min_goal_dist=TRANS_MIN_GOAL_DIST,
    )
    trans_modifier.work()


if __name__ == '__main__':
    main()
