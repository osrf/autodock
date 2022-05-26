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
import argparse

import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState

from autodock_core.autodock_utils import DockState
import autodock_core.autodock_utils as utils
from autodock_core.autodock_server import AutoDockServer, AutoDockConfig

##############################################################################


class DefaultAutoDockConfig(AutoDockConfig):
    """
    Default config is used in simulation
    # Note: all these tf computations are reference to robot base_link. Hence,
    """
    # General Configs
    cam_offset = 0.35           # camera to base link, for edge2edge distance
    front_dock = False
    tf_expiry: float = 1.0      # sec
    dock_timeout = 220          # sec
    controller_rate = 4.0       # hz

    # goal thresh
    stop_yaw_diff = 0.03        # radian
    stop_trans_diff = 0.02      # meters

    # frames and topics
    base_link = "base_link"
    left_marker = "fiducial_10"
    right_marker = "fiducial_11"
    centre_marker = "fiducial_20"

    # velocity profiles
    linear_vel_range = (-0.6, 0.6)
    angular_vel_range = (-0.4, 0.4)

    max_linear_vel = 0.5         # m/s, for parallel.c and steer
    min_linear_vel = 0.25        # m/s, for lastmile
    max_angular_vel = 0.35       # rad/s
    min_angular_vel = 0.20       # rad/s

    # predock state
    max_parallel_offset = 0.16     # m, will move to parallel.c if exceeded
    predock_tf_samples = 5         # tf samples to avg, parallel.c validation

    # steer dock state
    offset_to_angular_vel = 2   # factor to convert y-offset to ang vel
    to_last_mile_dis = 0.50     # edge2edge distance where transition to LM
    to_last_mile_tol = 0.25     # transition tolerance from SD to LM

    # last mile
    stop_distance = 0.10        # edge2edge distance to stop from charger
    max_last_mile_odom = 0.20   # max last mile odom move without using marker

    # activate charger state
    enable_charger_srv = True
    check_battery_status = False
    check_battery_timeout = 1.0
    charger_srv_name = "trigger_charger_srv"
    battery_status_topic = "battery_state"

    # retry state
    retry_count = 1             # how many times to retry
    retry_retreat_dis = 0.4     # meters, distance retreat during retry

    # debug state
    debug_mode = True           # when False selectively turns on aruco detections only
                                # a valid action is in progress.


class AutoDockStateMachine(AutoDockServer):
    """
    Implementation of the AutoDock Server with a Simple State Machine. Also
    this describes the logic of which the control loop of each state.
    """

    def __init__(self,
                 config: DefaultAutoDockConfig,
                 run_server=False,
                 load_rosparam=False,
                 fake_clock=False):
        # /Note: This ugly fix is to solve time sync issue when nodes are
        # running on a different PC, which time in not sync. As we will only
        # wish to enable sim time on 'AutoDockServer Node', please ensures
        # this node is not starting along with other nodes.
        if fake_clock:
            rospy.logwarn("WARNING!!!! fake clock is in used. "
                          "Temporary set use_sim_time to true")
            rospy.set_param("/use_sim_time", True)

        rospy.init_node('auto_dock_node')

        if fake_clock:
            rospy.logwarn(
                "WARNING!!!! fake clock enabled! now disable use_sim_time")
            rospy.set_param("/use_sim_time", False)

        self.cfg = config
        if load_rosparam:
            self.init_params()

        super().__init__(self.cfg, run_server)
        self.dock_state = DockState.IDLE

    def init_params(self):
        rospy.loginfo("Getting AutoDockServer params from rosparams server")
        param_names = [attr for attr in dir(self.cfg) if not callable(
            getattr(self.cfg, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param(
                "~" + param_name, getattr(self.cfg, param_name))
            print(f" set param [{param_name}] to [{param_val}]")
            setattr(self.cfg, param_name, param_val)

            # brute check to ensure numerical config is positive
            if isinstance(param_val, (int, float)):
                assert param_val >= 0, f"[{param_name}] param should be +ve"

    def start(self) -> bool:
        """
        Start Docking Sequence
        """
        rospy.loginfo("Start Docking Action Now")
        self.init_params()
        self.set_aruco_detections(detection_state=True)
        rospy.sleep(self.sleep_period)

        current_retry_count = 0
        rospy.loginfo(f"Will attempt with {self.cfg.retry_count} retry")

        while(True):

            if(
                self.do_predock()
                and self.do_steer_dock()
                and self.do_last_mile()
                and self.do_activate_charger()
            ):
                self.publish_cmd()
                self.set_state(DockState.IDLE, "All Success!")
                self.set_aruco_detections(detection_state=False)
                return True

            # If Dock failed
            self.publish_cmd()
            rospy.logwarn("Failed to Dock attempt")

            # Break from a retry
            if(current_retry_count >= self.cfg.retry_count):
                self.set_aruco_detections(detection_state=False)
                break

            # check again if it failed because of canceled
            if self.check_cancel():
                self.set_aruco_detections(detection_state=False)
                break

            # Attempt a Retry
            current_retry_count += 1
            rospy.logwarn("Attempting retry: "
                          f"{current_retry_count}/{self.cfg.retry_count}")

            if not self.do_retry():
                rospy.logwarn("Not able to retry")
                self.set_aruco_detections(detection_state=False)
                break

        # Note: will not set_state IDLE here since we will wish to capture
        # the state which failed in the action result status
        self.publish_cmd()
        return False

    def do_retry(self) -> bool:
        """
        Attempt to retry the docking again, only retry by retreating if it
        failed in activate charger, last mile. if Failed with steerDock,
        will straightaway attempt predock without retreating.
        """
        if self.check_cancel():
            return False

        if self.dock_state in [DockState.ACTIVATE_CHARGER,
                               DockState.LAST_MILE]:
            self.set_state(DockState.RETRY, "Retry by retreating")
            _dis = self.cfg.retry_retreat_dis
            if self.cfg.front_dock:
                _dis *= -1
            return self.move_with_odom(_dis)
        elif self.dock_state == DockState.STEER_DOCK:
            self.set_state(DockState.RETRY, "skip retreat and try predock")
            return True
        else:
            rospy.loginfo("State is not retry-able")
            return False

    def do_parallel_correction(self, offset: float) -> bool:
        """
        A parallel "parking" correction will be executed if the y offset
        exceeds the allowable threshold. Note this is an open loop operation.
        :return: success
        """
        self.set_state(DockState.PARALLEL_CORRECTION,
                       f"activate parallel correction with {offset:.2f}m")
        return (
            # Step 1: turn -90 degrees respective to odom
            self.rotate_with_odom(-math.pi/2)
            # Step 2: parallel correction respective to odom
            and self.move_with_odom(offset)
            # Step 3: turn 90 degrees respective to odom
            and self.rotate_with_odom(math.pi/2)
        )

    def do_single_side_marker_rotate(self) -> bool:
        """
        This predock's substate which handles single side marker detection,
        adjust the yaw according to the pose estimation of one single marker
        :return : success
        """
        self.set_state(DockState.PREDOCK, "Try single marker yaw adjustment")

        left_tf = self.get_tf(self.cfg.left_marker)
        if left_tf is not None:
            rospy.logwarn(f"Rotate with left marker: {self.cfg.left_marker}")
            yaw = utils.get_2d_pose(left_tf)[2]
            if self.cfg.front_dock:
                yaw = utils.flip_yaw(yaw)
            return self.rotate_with_odom(yaw - math.pi/2)

        right_tf = self.get_tf(self.cfg.right_marker)
        if right_tf is not None:
            rospy.logwarn(f"Rotate with right marker: {self.cfg.right_marker}")
            yaw = utils.get_2d_pose(right_tf)[2]
            if self.cfg.front_dock:
                yaw = utils.flip_yaw(yaw)
            return self.rotate_with_odom(yaw - math.pi/2)

        rospy.logerr("Not detecting two side markers, exit state")
        return False

    def do_predock(self) -> bool:
        """
        This is the predock phase, which will simply check if the the robot
        is located in the allowable zone, and rotate the robot base as such as
        the robot orientation is normal to the side markers.
        @return: success
        """
        self.set_state(DockState.PREDOCK, "start docking")

        # initial check if both markers are detected,
        # if only one single side marker detected, will purely depends to that
        # marker to rotate till facing the charger. then will check if both
        # markers exist again.
        if self.get_centre_of_side_markers() is None:
            if not self.do_single_side_marker_rotate():
                return False

        # start predock loop
        _pose_list = []
        rospy.loginfo("Both Markers are detected, running predock loop")
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            centre_tf = self.get_centre_of_side_markers()

            if centre_tf is None:
                rospy.logerr("Not detecting two side markers, exit state")
                return False

            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)
            _, _, yaw = centre_tf
            print(f"current yaw diff: {yaw:.3f}")

            # Check if the robot is now normal to the charging station
            if abs(yaw) < self.cfg.stop_yaw_diff:
                rospy.logwarn(f"Done with yaw correction, tf: {centre_tf}")
                self.publish_cmd()

                # switch the reference frame to charging station.
                marker_to_base_link_tf = utils.get_2d_inverse(centre_tf)
                if len(_pose_list) < self.cfg.predock_tf_samples:
                    remainings = self.cfg.predock_tf_samples - len(_pose_list)
                    print(f"Getting {remainings} more samples for averaging")
                    _pose_list.append(marker_to_base_link_tf)
                    continue

                y_offset = utils.avg_2d_poses(_pose_list)[1]
                if self.cfg.front_dock:  # TODO
                    y_offset *= -1
                _pose_list = []

                # if robot y axis is way off, we will do parallel correction
                # after this, will repeat predock
                if abs(y_offset) > self.cfg.max_parallel_offset:
                    if not self.do_parallel_correction(y_offset):
                        return False

                    self.publish_cmd()
                    rospy.sleep(rospy.Duration(self.cfg.tf_expiry))
                    self.set_state(DockState.PREDOCK, "try predock again")
                    continue

                return True

            # publish rotate
            ang_vel = utils.bin_filter(yaw, self.cfg.min_angular_vel)
            self.publish_cmd(angular_vel=ang_vel)

            rospy.sleep(self.sleep_period)
        exit(0)

    def do_steer_dock(self) -> bool:
        """
        This will utilize the side markers to steer dock the robot
        closer to the target charging station
        @return: success
        """
        self.set_state(DockState.STEER_DOCK)
        offset_from_charger = \
            self.cfg.cam_offset + self.cfg.to_last_mile_dis
        transition_dis_with_tol = \
            offset_from_charger + self.cfg.to_last_mile_tol
        _dir = 1 if self.cfg.front_dock else -1

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            # Note that centre_tf is consists of a front_offset
            centre_tf = self.get_centre_of_side_markers(offset_from_charger)

            # check if both markers are not detected
            if centre_tf is None:
                rospy.logwarn("Not detecting two sides marker")
                centre_tf_mat = self.get_tf(self.cfg.centre_marker)

                if centre_tf_mat is None:
                    rospy.logerr("Not able to locate centre marker too")
                    return False

                dis, _, _ = utils.get_2d_pose(centre_tf_mat)
                # check centre marker is near, with safety tol
                if (abs(dis) > transition_dis_with_tol):
                    rospy.logerr(
                        f"Centre marker is {dis:.3f} m away, too far, exit")
                    return False

                rospy.logwarn(
                    f"Centre marker {dis}m away, transition to last_mile")
                return True

            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)
            dis, offset, _ = centre_tf

            # transition distance to last mile
            # since we offset with offset_from_charger, This should be 0,
            if (dis > 0):
                rospy.logwarn("Transition to lastmile state")
                self.publish_cmd()
                return True

            print(f"  DETECTED both side markers!! "
                  f"[d: {dis:.2f}, offset: {offset:.2f}]")

            ang_vel = utils.sat_proportional_filter(
                -offset, factor=self.cfg.offset_to_angular_vel)
            self.publish_cmd(linear_vel=_dir*self.cfg.max_linear_vel,
                             angular_vel=ang_vel)
            rospy.sleep(self.sleep_period)
        exit(0)

    def do_last_mile(self) -> bool:
        """
        This will utilize the single centre marker to conduct a final last
        mile docking to the target charging station
        @return: success
        """
        self.set_state(DockState.LAST_MILE)

        remaining_dis = self.cfg.to_last_mile_dis
        _dir = 1 if self.cfg.front_dock else -1

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            centre_tf_mat = self.get_tf(self.cfg.centre_marker)

            # Final Dock based on odom if centre marker is getting to close
            if centre_tf_mat is None:
                rospy.logwarn("Not detecting centre marker")
                if remaining_dis < self.cfg.max_last_mile_odom:
                    rospy.logwarn(f"move {remaining_dis}m with odom")
                    return self.move_with_odom(_dir*remaining_dis)
                else:
                    rospy.logerr("exceeded max_last_mile_odom with "
                                 "remaining dis of {remaining_dis}, exit!")
                    return False

            centre_tf = utils.get_2d_pose(centre_tf_mat)
            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)
            dis, _, yaw = centre_tf

            yaw -= math.pi/2
            remaining_dis = - dis - self.cfg.stop_distance - self.cfg.cam_offset
            print(f" Approaching Charger -> d: {dis:.3f}, yaw: {yaw:.2f}"
                  f", remaining dis: {remaining_dis:.3f}")

            if (remaining_dis <= 0):
                rospy.loginfo(" ~ STOP!! Reach destination! ~")
                self.publish_cmd()
                return True

            ang_vel = utils.sat_proportional_filter(
                yaw, abs_min=0.0, abs_max=self.cfg.min_angular_vel, factor=1.2)
            self.publish_cmd(linear_vel=_dir*self.cfg.min_linear_vel,
                             angular_vel=ang_vel)
            rospy.sleep(self.sleep_period)
        exit(0)

    def do_activate_charger(self) -> bool:
        self.set_state(DockState.ACTIVATE_CHARGER, "Docked, activate charger!")
        rospy.sleep(rospy.Duration(2.0))
        # Charger trigger service
        if self.cfg.enable_charger_srv:
            try:
                _trigger = rospy.ServiceProxy(
                    self.cfg.charger_srv_name, Trigger)
                res = _trigger()
                rospy.loginfo(f"Trigger charger srv, "
                              f"success [{res.success}] | msg: {res.message}")
                if not res.success:
                    rospy.logerr("trigger charger failed")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"Charger srv call failed: {e}")
                return False

        # BatteryCharge status Validation
        if self.cfg.check_battery_status:
            try:
                # check status every 2s
                start = rospy.Time.now()
                while (True):
                    battery_status_msg = rospy.wait_for_message(
                        self.cfg.battery_status_topic, BatteryState, timeout=2.0)
                    charge_status = battery_status_msg.power_supply_status

                    if (charge_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING):
                        break

                    print(f"battery state is {charge_status}, Not Charging!")
                    rospy.sleep(rospy.Duration(1.0))
                    if ((rospy.Time.now() - start).secs >
                            self.cfg.check_battery_timeout):
                        rospy.logerr("time out for check battery!")
                        return False
                rospy.loginfo("Battery is now Charging!")
            except rospy.exceptions.ROSException:
                rospy.logerr(f"Failed to get battery state")
                return False
        self.set_state(DockState.ACTIVATE_CHARGER, "Celebration!!")
        return True


##############################################################################
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simple AutoDockServer')
    parser.add_argument('--server', dest='run_server',
                        action='store_true', help="run action server")
    parser.add_argument('--fake_clock', dest='fake_clock',
                        action='store_true',
                        help="Danger! use this to solve time sync issue")
    parser.add_argument('--rosparam', dest='load_rosparam',
                        action='store_true', help="load rosparam for configs")
    args, unknown = parser.parse_known_args()

    # Default Default Configuration
    config = DefaultAutoDockConfig()

    if args.run_server:
        node = AutoDockStateMachine(
            config,
            run_server=True,
            load_rosparam=args.load_rosparam,
            fake_clock=args.fake_clock)
        rospy.spin()
    else:
        node = AutoDockStateMachine(
            config,
            run_server=False,
            load_rosparam=args.load_rosparam,
            fake_clock=args.fake_clock)
        node.start()
