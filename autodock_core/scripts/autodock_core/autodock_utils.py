#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import math
from typing import Tuple, List
import numpy as np

import tf
from tf import transformations as ts

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from autodock_core.msg import AutoDockingFeedback


##############################################################################

Pose2D = Tuple[float, float, float]


class DockState:
    INVALID = AutoDockingFeedback.STATE_INVALID
    IDLE = AutoDockingFeedback.STATE_IDLE
    PREDOCK = AutoDockingFeedback.STATE_PREDOCK
    PARALLEL_CORRECTION = AutoDockingFeedback.STATE_PARALLEL_CORRECTION
    STEER_DOCK = AutoDockingFeedback.STATE_STEER_DOCK
    LAST_MILE = AutoDockingFeedback.STATE_LAST_MILE
    ACTIVATE_CHARGER = AutoDockingFeedback.STATE_ACTIVATE_CHARGER
    RETRY = AutoDockingFeedback.STATE_RETRY
    PAUSE = AutoDockingFeedback.STATE_PAUSE

    def to_string(input):
        _map = {
            DockState.INVALID: 'INVALID',
            DockState.IDLE: 'IDLE',
            DockState.PREDOCK: 'PREDOCK',
            DockState.PARALLEL_CORRECTION: 'PARALLEL_CORRECTION',
            DockState.STEER_DOCK: 'STEER_DOCK',
            DockState.LAST_MILE: 'LAST_MILE',
            DockState.ACTIVATE_CHARGER: 'ACTIVATE_CHARGER',
            DockState.RETRY: 'RETRY',
            DockState.PAUSE: 'PAUSE'
        }
        return _map[input]

    def to_percent(input):
        """
        Simple util to convert DockState to percent representation,
        use in publishing feedback in dock server
        """
        _map = {
            DockState.IDLE: 0.0,
            DockState.PREDOCK: 0.15,
            DockState.PARALLEL_CORRECTION: 0.35,
            DockState.STEER_DOCK: 0.50,
            DockState.LAST_MILE: 0.8,
            DockState.ACTIVATE_CHARGER: 0.9,
            DockState.RETRY: 0.1,
            DockState.PAUSE: 0.1  # TODO
        }
        return _map[input]

##############################################################################


def get_2d_inverse(pose_2d: Pose2D) -> Pose2D:
    """
    Inverse a 2d transformation, mainly to switch the frame of reference
    :param tf1, tf2:    2d transformation
    :return:            2d tf of [x, y, yaw]
    """
    _tr = (pose_2d[0], pose_2d[1], 0)
    _q = ts.quaternion_from_euler(0, 0, pose_2d[2])
    _tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
    _i_tf_mat = ts.inverse_matrix(_tf_mat)
    trans = ts.translation_from_matrix(_i_tf_mat)
    euler = ts.euler_from_matrix(_i_tf_mat)
    return trans[0], trans[1], euler[2]


def get_centre_tf(tf1: np.ndarray, tf2: np.ndarray, offset=0.0) -> Pose2D:
    """
    Get centre point of both tf1 and tf2, Note that this output new frame
    has a different orientation as the marker, which it's frame is x axis
    (pointing out of the marker) is directed towards the robot's camera.
    :param tf1, tf2:    4x4 homogenous matrix from tf1 and tf2
    :param offset:      (optional) offset target point to the normal
    :return:            2d tf of the centre [x, y, yaw]
    """
    x1, y1, _ = get_2d_pose(tf1)
    x2, y2, _ = get_2d_pose(tf2)
    _x = (x1 + x2)/2
    _y = (y1 + y2)/2
    _yaw = -math.atan2((x2 - x1), (y2 - y1))
    _x += math.cos(_yaw)*offset
    _y += math.sin(_yaw)*offset
    # print("Inverse! ", get_2d_inverse((_x, _y, _yaw))) # for sanity check
    return _x, _y, _yaw


def get_mat_from_transfrom_msg(msg: TransformStamped) -> np.ndarray:
    """
    This will return a homogenous transformation of transform msg
    :param :    input transform msg
    :return :   homogenous transformation matrix
    """
    _rot = msg.transform.rotation
    _q = (_rot.x, _rot.y, _rot.z, _rot.w)

    _trans = msg.transform.translation
    _tr = (_trans.x, _trans.y, _trans.z)
    _tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
    return _tf_mat


def get_mat_from_odom_msg(msg: Odometry) -> np.ndarray:
    """
    This will return a homogenous transformation of odom pose msg
    :param :    input odom msg
    :return :   homogenous transformation matrix
    """
    _rot = msg.pose.pose.orientation
    _q = (_rot.x, _rot.y, _rot.z, _rot.w)
    _trans = msg.pose.pose.position
    _tr = (_trans.x, _trans.y, _trans.z)
    _tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
    return _tf_mat


def get_2d_pose(_tf: np.ndarray) -> Pose2D:
    """
    :param:   input homogenous matrix
    :return : 2dPose in x, y, yaw format
    """
    trans = ts.translation_from_matrix(_tf)
    euler = ts.euler_from_matrix(_tf)
    return trans[0], trans[1], euler[2]


def apply_2d_transform(mat: np.ndarray, transform: Pose2D) -> np.ndarray:
    """
    Apply a 2d transform to a homogenous matrix
    :param mat:         the input 4x4 homogenous matrix
    :param transform :  2d transform which to apply to the mat
    :return :           transformed homogenous transformation matrix
    """
    # req target transformation from base
    q = tf.transformations.quaternion_from_euler(0, 0, transform[2])
    tf_mat = ts.concatenate_matrices(
        ts.translation_matrix(
            (transform[0], transform[1], 0)), ts.quaternion_matrix(q))
    return np.matmul(mat, tf_mat)


def compute_tf_diff(current_tf: np.ndarray, ref_tf: np.ndarray) -> Pose2D:
    """
    Find the diff of two transformation matrix
    :param :  homogenous transformation of 2 matrices
    :return :  the 2d planer trans fo the 2 inputs; [x, y, yaw]
    """
    tf_diff = np.matmul(ts.inverse_matrix(current_tf), ref_tf)
    trans = ts.translation_from_matrix(tf_diff)
    euler = ts.euler_from_matrix(tf_diff)
    return trans[0], trans[1], euler[2]


def avg_2d_poses(poses: List[Pose2D]) -> Pose2D:
    """
    Provide the average of a list of 2D poses
    :param poses    : a list of 2d poses
    :return         : output avg Pose2D
    """
    _l = len(poses)
    if (_l == 0):
        return None
    _x = 0
    _y = 0
    _yaw = 0
    for pose in poses:
        _x += pose[0]
        _y += pose[1]
        _yaw += pose[2]
    return _x/_l, _y/_l, _yaw/_l


def sat_proportional_filter(
        input: float, abs_min=0.0, abs_max=10.0, factor=1) -> float:
    """
    Simple saturated proportional filter
    :param input                : input value
    :param abs_min and abs_max  : upper and lower bound, abs value
    :param factor               : multiplier factor for the input value
    :return                     : output filtered value, within boundary
    """
    output = 0.0
    input *= factor
    if abs(input) < abs_min:
        if (input < 0):
            output = -abs_min
        else:
            output = abs_min
    elif abs(input) > abs_max:
        if (input > 0):
            output = abs_max
        else:
            output = -abs_max
    else:
        output = input
    return output


def bin_filter(input: float, abs_boundary: float) -> float:
    """
    Simple binary filter, will provide abs_ceiling as a binary output,
    according to the 'negativity' of the input value
    :param input        : input value
    :param abs_boundary : abs boundary value
    :return             : output binary value
    """
    output = abs(abs_boundary)
    if input < 0:
        output = -abs(abs_boundary)
    return output


def flip_yaw(yaw: float) -> float:
    """
    Flip yaw angle by 180 degree, input yaw range should be within
    [-pi, pi] radian. Else use set_angle() fn to fix the convention.
    Output will also be within the same range of [-pi, pi] radian.
    """
    if yaw >= 0:
        return yaw - math.pi
    else:
        return yaw + math.pi


def set_angle(angle: float) -> float:
    """
    Ensure the angle is within the range of [-pi, pi] radian convention
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def flip_base_frame(input: Pose2D):
    """
    Flip the current reference frame by 180 degree. As such, the negativity 
    of translation is flipped, and the yaw angle is located at the opposite 
    quadrant. Currently is used to flip from 'back dock' to 'front dock'
    """
    return -input[0], -input[1], flip_yaw(input[2])
