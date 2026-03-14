#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import numpy as np
import open3d as o3d

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utils import MAP_DIR, BASE_TRANS_FILE, RAW_FRAME_FOLD, BASE_FRAME_FOLD, FINAL_MAP_FILE
from utils import SENSOR_IN_BASE
from utils import MAP3D_FINAL_VOXEL_SIZE, MAP3D_FILTER_HEIGHT_BOUND, MAP3D_FILTER_FOV_BOUND, MAP3D_FILTER_DISTANCE_BOUND, MAP3D_FILTER_INTENSITY_BOUND
from utils import pcd_to_trans


class Map3dTransform:

    def __init__(
        self,
        base_trans_path,
        raw_frame_dir,
        base_frame_dir,
        final_map_path,
        sensor_in_base,
        voxel_size,
        height_bound,
        fov_bound,
        distance_bound,
        intensity_bound,
    ):
        # transform
        self.__sensor_in_base = sensor_in_base
        trans_pcd = o3d.t.io.read_point_cloud(base_trans_path)
        self.__trans_position = trans_pcd.point["positions"].numpy()
        self.__trans_roll = trans_pcd.point["roll"].numpy()
        self.__trans_pitch = trans_pcd.point["pitch"].numpy()
        self.__trans_yaw = trans_pcd.point["yaw"].numpy()

        # frame
        self.__raw_frame_dir = raw_frame_dir
        self.__base_frame_dir = base_frame_dir

        # bound
        self.__height_bound = height_bound
        self.__fov_bound = fov_bound
        self.__distance_bound = distance_bound
        self.__intensity_bound = intensity_bound

        # output
        self.__voxel_size = voxel_size
        self.__out_pcd = o3d.geometry.PointCloud()
        self.__out_path = final_map_path

    def __read_intensities(self, i):
        file_name = f"{self.__raw_frame_dir}/{i}.pcd"
        with open(file_name, 'rb') as f:
            header_bytes = b""
            while True:
                line = f.readline()
                header_bytes += line
                if line.strip().startswith(b"DATA"):
                    break

            # parse the header
            header_str = header_bytes.decode('utf-8', errors='ignore')
            num_points = None
            for line in header_str.splitlines():
                if line.startswith("POINTS"):
                    num_points = int(line.split()[1])
                    break

            # read the data
            point_dtype = np.dtype([('x', np.float32), ('y', np.float32),
                                    ('z', np.float32),
                                    ('intensity', np.float32),
                                    ('normal_x', np.float32),
                                    ('normal_y', np.float32),
                                    ('normal_z', np.float32),
                                    ('curvature', np.float32)])
            pcd_data = np.fromfile(f, dtype=point_dtype, count=num_points)

            # extract the intensities
            intensities = pcd_data['intensity']

        return intensities

    def __frame_process(self, i):
        raw_keyframe = o3d.io.read_point_cloud(
            f"{self.__raw_frame_dir}/{i}.pcd")

        # point to base
        # P_out = T * P_in
        raw_keyframe.transform(self.__sensor_in_base)

        # filter
        points = np.asarray(raw_keyframe.points)
        intensities = self.__read_intensities(i)
        if self.__intensity_bound is not None:
            mask = (intensities > self.__intensity_bound[0]) & (
                intensities < self.__intensity_bound[1])
            points = points[mask]
        if self.__height_bound is not None:
            mask = (points[:, 2] > self.__height_bound[0]) & (
                points[:, 2] < self.__height_bound[1])
            points = points[mask]
        if self.__fov_bound is not None:
            fov = np.arctan2(points[:, 1], points[:, 0])
            mask = np.zeros_like(fov, dtype=bool)
            for bound in self.__fov_bound:
                curr_mask = (fov > bound[0]) & (fov < bound[1])
                mask |= curr_mask
            points = points[mask]
        if self.__distance_bound is not None:
            distance = np.linalg.norm(points[:, :2], axis=-1)
            mask = (distance > self.__distance_bound[0]) & (
                distance < self.__distance_bound[1])
            points = points[mask]

        # check whether the keyframe is empty
        if points.shape[0] == 0:
            return

        filtered_keyframe = o3d.geometry.PointCloud()
        filtered_keyframe.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(f"{self.__base_frame_dir}/{i}.pcd",
                                 filtered_keyframe)

        # point to map
        base_in_map = pcd_to_trans(self.__trans_position[i], [
            self.__trans_roll[i][0], self.__trans_pitch[i][0],
            self.__trans_yaw[i][0]
        ])
        # P_out = T * P_in
        filtered_keyframe.transform(base_in_map)

        # merge
        self.__out_pcd += filtered_keyframe

    def work(self):
        # frame process
        for i in range(self.__trans_position.shape[0]):
            if (i + 1) % 50 == 0:
                print(
                    f"MAP3D Transforming keyframe {i+1}/{self.__trans_position.shape[0]}"
                )
            self.__frame_process(i)

        # downsample
        if self.__voxel_size is not None:
            self.__out_pcd = self.__out_pcd.voxel_down_sample(
                self.__voxel_size)

        # save pcd
        print(f"save out: {self.__out_path}")
        out_shape = np.asarray(self.__out_pcd.points).shape
        print(f"out shape: {out_shape[0]}")
        o3d.io.write_point_cloud(self.__out_path, self.__out_pcd)


def main():
    map_name = "default"
    if len(sys.argv) > 1:
        map_name = sys.argv[1] 

    base_trans_path = f"{MAP_DIR}/{map_name}/{BASE_TRANS_FILE}"
    raw_frame_dir = f"{MAP_DIR}/{map_name}/{RAW_FRAME_FOLD}"
    base_frame_dir = f"{MAP_DIR}/{map_name}/{BASE_FRAME_FOLD}"
    final_map_path = f"{MAP_DIR}/{map_name}/{FINAL_MAP_FILE}"
    if not os.path.exists(base_frame_dir):
        os.makedirs(base_frame_dir)

    map3d_transform = Map3dTransform(
        base_trans_path=base_trans_path,
        raw_frame_dir=raw_frame_dir,
        base_frame_dir=base_frame_dir,
        final_map_path=final_map_path,
        sensor_in_base=SENSOR_IN_BASE,
        voxel_size=MAP3D_FINAL_VOXEL_SIZE,
        height_bound=MAP3D_FILTER_HEIGHT_BOUND,
        fov_bound=MAP3D_FILTER_FOV_BOUND,
        distance_bound=MAP3D_FILTER_DISTANCE_BOUND,
        intensity_bound=MAP3D_FILTER_INTENSITY_BOUND,
    )
    map3d_transform.work()


if __name__ == '__main__':
    main()
