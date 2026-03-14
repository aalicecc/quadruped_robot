#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import yaml
import numpy as np
import open3d as o3d

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utils import MAP_DIR, FINAL_MAP_FILE, BASE_TRANS_FILE, BASE_FRAME_FOLD, GRID_FOLD
from utils import BASE_IN_SENSOR
from utils import MAP2D_HEIGHT_LIMIT, MAP2D_GRID_RESOLUTION, MAP2D_GRID_FREE, MAP2D_GRID_OCCUPY
from utils import pcd_to_trans


class Map2dTransform:

    def __init__(
        self,
        final_map_path,
        base_trans_path,
        base_frame_dir,
        grid_dir,
        base_in_sensor,
        height_limit,
        grid_resolution,
        grid_free,
        grid_occupy,
    ):
        # transform
        self.__base_in_sensor = base_in_sensor
        trans_pcd = o3d.t.io.read_point_cloud(base_trans_path)
        self.__trans_position = trans_pcd.point["positions"].numpy()
        self.__trans_roll = trans_pcd.point["roll"].numpy()
        self.__trans_pitch = trans_pcd.point["pitch"].numpy()
        self.__trans_yaw = trans_pcd.point["yaw"].numpy()

        # keyframe
        self.__base_frame_dir = base_frame_dir
        self.__high_limit = height_limit[0]
        self.__low_factor_1 = height_limit[1]
        self.__low_factor_2 = height_limit[2]
        self.__curr_frame = None

        # bound
        map3d_bound = o3d.io.read_point_cloud(
            final_map_path).get_axis_aligned_bounding_box()
        min_bound = map3d_bound.get_min_bound()
        max_bound = map3d_bound.get_max_bound()
        grid_bound = [[min_bound[0], min_bound[1]],
                      [max_bound[0], max_bound[1]]]

        # output
        self.__grid_image_path = f"{grid_dir}/map.png"
        self.__grid_yaml_path = f"{grid_dir}/map.yaml"
        self.__grid_resolution = grid_resolution
        self.__grid_shape = [
            int((grid_bound[1][1] - grid_bound[0][1]) / grid_resolution + 0.5)
            + 2,
            int((grid_bound[1][0] - grid_bound[0][0]) / grid_resolution) + 2
        ]
        self.__grid_start_index = [1, 1]
        self.__grid_start_position = grid_bound[0]
        self.__grid_image = np.ones(self.__grid_shape, dtype=np.uint8) * 127
        self.__grid_free = grid_free
        self.__grid_occupy = grid_occupy
        self.__ray_start_index = [0, 0]
        self.__ray_end_index = [0, 0]

    def __height_filter(self):
        positions = []
        curr_frame_positions = self.__curr_frame.point["positions"].numpy()
        for i in range(curr_frame_positions.shape[0]):
            low_limit = self.__low_factor_2 * curr_frame_positions[i][
                2] + self.__low_factor_1
            if curr_frame_positions[i][
                    2] < self.__high_limit and curr_frame_positions[i][
                        2] > low_limit:
                positions.append(curr_frame_positions[i])
        self.__curr_frame.point["positions"] = np.array(positions,
                                                        dtype=np.float32)

    def __update_gridmap(self, i):
        base_in_odom = pcd_to_trans(self.__trans_position[i], [
            self.__trans_roll[i][0], self.__trans_pitch[i][0],
            self.__trans_yaw[i][0]
        ])
        sensor_in_odom = base_in_odom @ self.__base_in_sensor

        self.__ray_start_index = [
            int((sensor_in_odom[0, 3] - self.__grid_start_position[0]) /
                self.__grid_resolution) + self.__grid_start_index[0],
            int((sensor_in_odom[1, 3] - self.__grid_start_position[1]) /
                self.__grid_resolution) + self.__grid_start_index[1]
        ]
        self.__ray_start_index[
            1] = self.__grid_shape[0] - self.__ray_start_index[1]
        # print(f"ray start index: {self.__ray_start_index}")

        # P_out = T * P_in
        self.__curr_frame.transform(base_in_odom)
        pcd_points = self.__curr_frame.point["positions"].numpy()
        for i in range(pcd_points.shape[0]):
            self.__ray_end_index = [
                int((pcd_points[i, 0] - self.__grid_start_position[0]) /
                    self.__grid_resolution) + self.__grid_start_index[0],
                int((pcd_points[i, 1] - self.__grid_start_position[1]) /
                    self.__grid_resolution) + self.__grid_start_index[1]
            ]
            self.__ray_end_index[
                1] = self.__grid_shape[0] - self.__ray_end_index[1]
            self.__ray_cast()

    def __bresenham(self, start, end):
        x1, y1 = start[0], start[1]
        x2, y2 = end[0], end[1]

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        steep = abs(dy) > abs(dx)

        if steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        error = int(dx / 2)
        ystep = 1 if y1 < y2 else -1

        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if steep else (x, y)
            points.append(coord)
            error -= dy
            if error < 0:
                y += ystep
                error += dx

        if swapped:
            points.reverse()

        return points

    def __ray_cast(self):
        # bresenham
        points = self.__bresenham(self.__ray_start_index, self.__ray_end_index)

        if len(points) == 0:
            return

        for point in points:
            temp = self.__grid_image[point[1]][point[0]] + self.__grid_free
            self.__grid_image[point[1]][point[0]] = temp if temp < 255 else 255

        temp = self.__grid_image[points[-1][1]][points[-1]
                                                [0]] + self.__grid_occupy
        self.__grid_image[points[-1][1]][points[-1]
                                         [0]] = temp if temp > 0 else 0

    def __save_gridmap(self):
        cv2.imshow("grid map", self.__grid_image)
        cv2.waitKey(0)
        cv2.imwrite(self.__grid_image_path, self.__grid_image)

        yaml_data = {
            "image":
            self.__grid_image_path,
            "resolution":
            self.__grid_resolution,
            "origin": [
                float(self.__grid_start_position[0]),
                float(self.__grid_start_position[1]), 0.0
            ],
            "negate":
            0,
            "occupied_thresh":
            0.65,
            "free_thresh":
            0.196
        }
        with open(self.__grid_yaml_path, "w") as f:
            yaml.dump(yaml_data, f)

    def work(self):
        for i in range(self.__trans_position.shape[0]):
            if (i + 1) % 50 == 0:
                print(
                    f"MAP2D Transforming keyframe {i+1}/{self.__trans_position.shape[0]}"
                )
            raw_curr_frame = o3d.t.io.read_point_cloud(
                f"{self.__base_frame_dir}/{i}.pcd")
            self.__curr_frame = o3d.t.geometry.PointCloud()
            self.__curr_frame.point["positions"] = raw_curr_frame.point[
                "positions"]

            self.__height_filter()
            if self.__curr_frame.point["positions"].shape[0] > 0:
                self.__update_gridmap(i)

        self.__save_gridmap()


def main():
    map_name = "default"
    if len(sys.argv) > 1:
        map_name = sys.argv[1]

    final_map_path = f"{MAP_DIR}/{map_name}/{FINAL_MAP_FILE}"
    base_trans_path = f"{MAP_DIR}/{map_name}/{BASE_TRANS_FILE}"
    base_frame_dir = f"{MAP_DIR}/{map_name}/{BASE_FRAME_FOLD}"
    grid_dir = f"{MAP_DIR}/{map_name}/{GRID_FOLD}"
    if not os.path.exists(grid_dir):
        os.mkdir(grid_dir)

    map2d_transform = Map2dTransform(
        final_map_path=final_map_path,
        base_trans_path=base_trans_path,
        base_frame_dir=base_frame_dir,
        grid_dir=grid_dir,
        base_in_sensor=BASE_IN_SENSOR,
        height_limit=MAP2D_HEIGHT_LIMIT,
        grid_resolution=MAP2D_GRID_RESOLUTION,
        grid_free=MAP2D_GRID_FREE,
        grid_occupy=MAP2D_GRID_OCCUPY,
    )
    map2d_transform.work()


if __name__ == '__main__':
    main()
