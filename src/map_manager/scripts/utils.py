import rospkg
import numpy as np


def trans_inv(trans):
    trans_inv = np.eye(4)
    trans_inv[0:3, 0:3] = trans[0:3, 0:3].T
    trans_inv[0:3, 3] = -trans_inv[0:3, 0:3] @ trans[0:3, 3]
    return trans_inv


def euler_to_rot(euler):
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    roll_rot = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)],
                         [0, np.sin(roll), np.cos(roll)]])
    pitch_rot = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0],
                          [-np.sin(pitch), 0, np.cos(pitch)]])
    yaw_rot = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    return yaw_rot @ pitch_rot @ roll_rot


def rot_to_euler(matrix):
    pitch = np.arcsin(-matrix[2, 0])
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    roll = np.arctan2(matrix[2, 1], matrix[2, 2])

    return [roll, pitch, yaw]


def pcd_to_trans(position, euler):
    trans = np.eye(4)
    trans[0, 3] = position[0]
    trans[1, 3] = position[1]
    trans[2, 3] = position[2]
    trans[0:3, 0:3] = euler_to_rot(euler)
    return trans


def trans_to_pcd(trans):
    position = trans[0:3, 3]
    euler = rot_to_euler(trans[0:3, 0:3])
    return position, euler


### PATH ###
MAP_NAME = "default"
RAW_TRANS_FILE = "transformations.pcd"
BASE_TRANS_FILE = "base_transformations.pcd"
JSON_FILE = "memory.json"
FINAL_MAP_FILE = "FinalMap.pcd"
RAW_FRAME_FOLD = "keyFrames"
BASE_FRAME_FOLD = "baseFrames"
GRID_FOLD = "map2d"

### PARAM ###
# common
SENSOR_IN_BASE = np.array([[0.0000000, 0.9205049, 0.3907311, 0.16013],
                           [-1.0000000, 0.0000000, 0.0000000, 0.00000],
                           [0.0000000, -0.3907311, 0.9205049, 0.22777],
                           [0.0000000, 0.0000000, 0.0000000, 1.00000]])
# trans_modifier
TRANS_MIN_GOAL_DIST = 3.0
# map3d_transform
MAP3D_FINAL_VOXEL_SIZE = 0.1
MAP3D_FILTER_HEIGHT_BOUND = [-0.5, 50.0]
MAP3D_FILTER_FOV_BOUND = [[-2.35619449, 2.35619449]]
MAP3D_FILTER_DISTANCE_BOUND = [0.2, 50.0]
MAP3D_FILTER_INTENSITY_BOUND = [0.0, 10000.0]
# map2d_transform
MAP2D_HEIGHT_LIMIT = [0.3, 0.05, 0.000]
MAP2D_GRID_RESOLUTION = 0.1
MAP2D_GRID_FREE = 10
MAP2D_GRID_OCCUPY = -50
# map_viewer
MAPVIS_POINT_SIZE = 2.5
MAPVIS_BACKGROUND_COLOR = np.asarray([0, 0, 0])
MAPVIS_COLOR_FACTOR_1 = 0.1
MAPVIS_COLOR_FACTOR_2 = 1.0
MAPVIS_COLOR_FACTOR_3 = 0.9
MAPVIS_COLOR_FACTOR_4 = 0.1

### GENERATE ###
# path
MAP_DIR = f"{rospkg.RosPack().get_path('map_manager')}/output"

# param
BASE_IN_SENSOR = trans_inv(SENSOR_IN_BASE)
