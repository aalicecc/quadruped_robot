#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import numpy as np
import open3d as o3d

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utils import MAP_DIR, FINAL_MAP_FILE
from utils import MAPVIS_POINT_SIZE, MAPVIS_BACKGROUND_COLOR, MAPVIS_COLOR_FACTOR_1, MAPVIS_COLOR_FACTOR_2, MAPVIS_COLOR_FACTOR_3, MAPVIS_COLOR_FACTOR_4

class MapViewer:

    def __init__(self, map_path, point_size, background_color):
        self.__color_map = cv2.applyColorMap(np.arange(256, dtype=np.uint8),
                                             cv2.COLORMAP_JET).reshape(256, 3)
        self.__color_map = self.__color_map[::-1, :]
        self.__color_map = self.__color_map / 255.0

        self.__point_size = point_size
        self.__background_color = background_color

        self.__pcd_cloud = self.__read_pcd(map_path)

    def __apply_color_map(self, normed_intensity):
        point_num = normed_intensity.shape[0]
        color = np.empty(shape=(point_num, 3))

        for i in range(point_num):
            index = (MAPVIS_COLOR_FACTOR_1 + MAPVIS_COLOR_FACTOR_2 * normed_intensity[i])

            if index > MAPVIS_COLOR_FACTOR_3:
                index = MAPVIS_COLOR_FACTOR_3
            elif index < MAPVIS_COLOR_FACTOR_4:
                index = MAPVIS_COLOR_FACTOR_4

            color[i, :] = self.__color_map[int(index * 255.0)]
            # color[i, ::-1] = self.__color_map[int(index * 255.0)]

        return color

    def __read_pcd(self, pcd_path):
        pcd_file = o3d.t.io.read_point_cloud(pcd_path)

        pcd_points = pcd_file.point["positions"][:, :].numpy()
        pcd_intensity = pcd_points[:, 2]

        i_max = 0.0
        i_min = 25.0
        process_count = 0
        for i in range(pcd_intensity.shape[0]):
            if pcd_intensity[i] > i_max:
                i_max = pcd_intensity[i]
            elif pcd_intensity[i] < i_min:
                i_min = pcd_intensity[i]
            process_count += 1
            if process_count % 20000 == 0:
                print(f"{process_count} points Finish")
        points_intensity = (pcd_intensity - i_min) / (i_max - i_min)

        pcd_cloud = o3d.geometry.PointCloud()
        pcd_color = self.__apply_color_map(points_intensity)
        pcd_cloud.points = o3d.utility.Vector3dVector(pcd_points)
        pcd_cloud.colors = o3d.utility.Vector3dVector(pcd_color)
        print(f"{pcd_path} read Finish, total num : {process_count}")

        return pcd_cloud

    def work(self):
        pcd_vis = o3d.visualization.Visualizer()
        pcd_vis.create_window()

        pcd_opt = pcd_vis.get_render_option()
        pcd_opt.point_size = self.__point_size
        pcd_opt.background_color = self.__background_color

        pcd_vis.add_geometry(self.__pcd_cloud)
        pcd_vis.run()
        pcd_vis.destroy_window()


def main():
    map_name = "default"
    if len(sys.argv) > 1:
        map_name = sys.argv[1]

    map_path = f"{MAP_DIR}/{map_name}/{FINAL_MAP_FILE}"

    map_viewer = MapViewer(map_path=map_path,
                           point_size=MAPVIS_POINT_SIZE,
                           background_color=MAPVIS_BACKGROUND_COLOR)
    map_viewer.work()


if __name__ == '__main__':
    main()
