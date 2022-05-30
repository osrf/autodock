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


import rospy
import tf2_ros
import actionlib

from autodock_core.autodock_utils import DockState, Pose2D
import autodock_core.autodock_utils as utils

import numpy as np
from typing import Tuple

# rosmsgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from autodock_core.msg import AutoDockingAction, AutoDockingFeedback
from autodock_core.msg import AutoDockingGoal, AutoDockingResult
from std_srvs.srv import SetBool


##############################################################################


class AutoDockConfig:
    """
    Default AutoDockConfig
    """
    # General configs
    tf_expiry: float
    dock_timeout: float
    controller_rate: float
    # frames and topics
    base_link: str
    left_marker: str
    right_marker: str
    # vel profile
    linear_vel_range: Tuple[float, float]
    angular_vel_range = Tuple[float, float]
    max_linear_vel: float         # m/s, for parallel.c and steer
    min_linear_vel: float        # m/s, for lastmile
    max_angular_vel: float       # rad/s
    min_angular_vel: float       # rad/s
    # stop thresh
    stop_yaw_diff: float         # radian
    stop_trans_diff: float      # meters
    # debug state
    debug_mode: bool

##############################################################################
##############################################################################

class AutoDockServer:
    """
    This AutoDock server is the base class for the AutoDockServer. Here, we
    have abstracted all ROS Interfaces. The user is only required to derived
    from this class and implement his/her own custom implementation.
    """

    def __init__(self, config: AutoDockConfig, run_server: bool):
        rospy.loginfo("Starting AutoDockServer Node")
        self.cfg = config
        self.run_server = run_server

        # param check
        assert (len(self.cfg.linear_vel_range) == 2 and
                len(self.cfg.linear_vel_range) == 2
                ), "linear and angular vel range should have size of 2"
        assert (self.cfg.linear_vel_range[0] < self.cfg.linear_vel_range[1]
                ), "linear vel min should be larger than max"
        assert (self.cfg.angular_vel_range[0] < self.cfg.angular_vel_range[1]
                ), "linear vel min should be larger than max"

        # create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # create_publisher to cmd_vel
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # create_subscriber to pause dock
        rospy.Subscriber("/pause_dock", Bool, self.__pause_dock_cb)

        # debug timer for state machine marker
        if self.cfg.debug_mode:
            self.__marker_pub = rospy.Publisher('/sm_maker', Marker, queue_size=1)
            self.__timer = rospy.Timer(rospy.Duration(0.5), self.__timer_cb)
        self.is_pause = False # TODO

        self.dock_state = DockState.INVALID
        self.start_time = rospy.Time.now()
        self.sleep_period = rospy.Duration(1/self.cfg.controller_rate)

        # create service client to 
        # 1. enable aruco detections only when action is started
        # 2. disable aruco detections when action is idle
        # disable on initialize
        self.set_aruco_detections(detection_state=False)
        if run_server:
            self.__as = actionlib.SimpleActionServer(
                "autodock_action",
                AutoDockingAction,
                execute_cb=self.__execute_cb,
                auto_start=False)
            self.__as.start()
            self.feedback_msg = AutoDockingFeedback()
            rospy.loginfo("~ Start AutodockServer with ROS Action Server")

    def start(self) -> bool:
        """
        Virtual function. This function will be triggered when autodock request
        is requested
        :return : if action succeeded
        """
        rospy.logwarn("Server implementation has not been specified. "
                      "Do overload the start() function")
        return False

    def set_state(self, state: DockState, printout=""):
        """
        set state of the auto dock server
        :param state:       Current DockState
        :param printout:    Verbose description of the state
        """
        state_str = DockState.to_string(state)
        rospy.logwarn(f" State: [{state_str}] | {printout}")
        self.dock_state = state
        if self.run_server:
            self.feedback_msg.state = state
            self.feedback_msg.progress = DockState.to_percent(state)
            self.feedback_msg.status = f"{state_str} | {printout}"
            self.__as.publish_feedback(self.feedback_msg)

    def check_cancel(self) -> bool:
        """
        Check if to cancel this docking action. This will happen if a
        preempt is requested during server mode. or if a timeout is reached.
        :return : true if cancel is requested. false as default
        """
        if self.run_server and self.__as.is_preempt_requested():
            rospy.logwarn('Preempted Requested!')
            return True

        # check if dock_timeout reaches
        if (rospy.Time.now() - self.start_time).secs > self.cfg.dock_timeout:
            rospy.logwarn('Timeout reaches!')
            self.set_state(self.dock_state, "Reach Timeout")
            return True
        return False

    def publish_cmd(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        if (msg.linear.x > self.cfg.linear_vel_range[1]):
            msg.linear.x = self.cfg.linear_vel_range[1]
        elif(msg.linear.x < self.cfg.linear_vel_range[0]):
            msg.linear.x = self.cfg.linear_vel_range[0]

        if (msg.linear.x > self.cfg.angular_vel_range[1]):
            msg.linear.x = self.cfg.angular_vel_range[1]
        elif(msg.linear.x < self.cfg.angular_vel_range[0]):
            msg.linear.x = self.cfg.angular_vel_range[0]

        # print(f"   cmd_vel: [{msg.linear.x:.3f}, {msg.angular.z :.3f}]")
        self.__cmd_vel_pub.publish(msg)

    def get_odom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return utils.get_mat_from_odom_msg(
                rospy.wait_for_message(
                    "/odom", Odometry, timeout=self.cfg.tf_expiry)
            )
        except rospy.exceptions.ROSException:
            rospy.logerr(f"Failed to get odom")
            return None

    def get_tf(self,
               target_link: str,
               ref_link=None,
               target_time=None) -> np.ndarray:
        """
        This will provide the transformation of the marker,
        if ref_link is not provided, we will use robot's base_link as ref
        :param now : this is a hack fix
        :return : 4x4 homogenous matrix, None if not avail
        """
        if ref_link is None:
            ref_link = self.cfg.base_link
        if target_time is None:
            target_time = rospy.Time.now()

        try:
            return utils.get_mat_from_transfrom_msg(
                self.__tfBuffer.lookup_transform(
                    ref_link, target_link, target_time,
                    rospy.Duration(self.cfg.tf_expiry))
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed lookup: {target_link}, from {ref_link}")
            return None

    def get_centre_of_side_markers(self, offset=0.0) -> Pose2D:
        """
        Get centre tf of both side markers, reference to base_link
        :return: tf of the centre [x, y, yaw]
        """
        now = rospy.Time.now()
        left_tf = self.get_tf(self.cfg.left_marker, target_time=now)
        right_tf = self.get_tf(self.cfg.right_marker, target_time=now)

        if (left_tf is None or right_tf is None):
            return None

        # Offset is normal to the line, direction is outward to the
        # the camera. this is to create a point in front of the goal
        return utils.get_centre_tf(left_tf, right_tf, offset)

    def do_pause(self) -> bool:
        """
        Blocking function which will pause the action server.
        """
        prev_state = self.dock_state
        self.set_state(DockState.PAUSE, f"pause action from {prev_state}")
        self.publish_cmd()
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if not self.is_pause:
                self.set_state(prev_state, f"resume action from pause")
                return True

            rospy.sleep(self.sleep_period)
        exit(0)

    def move_with_odom(self, forward: float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        self.set_state(self.dock_state, f"move robot: {forward:.2f} m")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (forward, 0, 0))

        # second mat
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            _curr_tf = self.get_odom()
            if _curr_tf is None:
                return False

            dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)
            print(f" current x, y, yaw diff: {dx:.3f} | {dy:.2f} | {dyaw:.2f}")

            if abs(dx) < self.cfg.stop_trans_diff:
                rospy.logwarn("Done with move robot")
                return True

            # This makes sure that the robot is actually moving linearly
            ang_vel = utils.sat_proportional_filter(
                dyaw, abs_max=self.cfg.min_angular_vel, factor=0.2)
            l_vel = utils.bin_filter(dx, self.cfg.min_linear_vel)

            self.publish_cmd(linear_vel=l_vel, angular_vel=ang_vel)
            rospy.sleep(self.sleep_period)
        exit(0)

    def rotate_with_odom(self, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        """
        self.set_state(self.dock_state, f"Turn robot: {rotate:.2f} rad")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (0, 0, rotate))

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            if self.is_pause:
                if not self.do_pause():
                    return False

            _curr_tf = self.get_odom()
            if _curr_tf is None:
                return False

            dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)
            print(f"current x, y, yaw diff: {dx:.2f} | {dy:.2f} | {dyaw:.2f}")

            if abs(dyaw) < self.cfg.stop_yaw_diff:
                rospy.logwarn("Done with rotate robot")
                return True

            # publish rotate
            ang_vel = utils.sat_proportional_filter(
                dyaw,
                abs_min=self.cfg.min_angular_vel,
                abs_max=self.cfg.max_angular_vel)
            self.publish_cmd(angular_vel=ang_vel)
            rospy.sleep(self.sleep_period)
        exit(0)

    def set_aruco_detections(self, detection_state) -> bool:
        """
        Set aruco detections to True or False
        :return : success
        """
        if self.cfg.debug_mode:
            return True
        else:
            try:
                detection_srv_name = "/enable_detections"
                rospy.wait_for_service(detection_srv_name, timeout=3.0)
                enable_detections_srv = rospy.ServiceProxy(detection_srv_name, SetBool)
                resp = enable_detections_srv(detection_state)
                rospy.loginfo("Enable detections response: " + resp.message)
                return resp.success
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: " + str(e))

    def __pause_dock_cb(self, msg):
        self.is_pause = msg.data

    def __execute_cb(self, goal: AutoDockingGoal):
        self.start_time = rospy.Time.now()
        _result = AutoDockingResult()
        _result.is_success = self.start()
        _prev_state = DockState.to_string(self.feedback_msg.state)
        if _result.is_success:
            _duration = rospy.Time.now() - self.start_time
            _result.status = f"Succeeded! Took {_duration.secs}s"
            self.__as.set_succeeded(_result)
        elif self.__as.is_preempt_requested():
            _result.is_success = False
            _result.status = f"Cancel during [{_prev_state}], " \
                             f"with status: {self.feedback_msg.status}"
            self.__as.set_preempted(_result)
            self.set_state(DockState.IDLE, "Dock Action is canceled")
        else:
            _result.is_success = False
            _result.status = f"Failed during [{_prev_state}], " \
                             f"with status: {self.feedback_msg.status}"
            self.__as.set_aborted(_result)
            self.set_state(DockState.IDLE, "Failed execute Dock Action")

    def __timer_cb(self, timer):
        # This is mainly for debuging
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.cfg.base_link
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 1.1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.text = DockState.to_string(self.dock_state)
        self.__marker_pub.publish(marker)
        # print(" now: ", rospy.Time.now().secs)
