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


import os
import subprocess
import random
import argparse
from typing import List

# ros
import rospy
import actionlib
from autodock_core.msg import AutoDockingAction, AutoDockingGoal


# Params
model_name = "mock_docking_robot"
cam_offset = 0.35           # offset from base_link to cam_link
stop_distance = 0.10        # prevent collission with the charging station
operating_area = {
    'x': (1.5, 2.1),
    'y': (-0.3, 0.3),
    'z': (-0.4, 0.4)
}


##############################################################################


def autodock_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient(
        'autodock_action', AutoDockingAction)
    client.wait_for_server()

    # Sends the goal to the action server.
    goal_msg = AutoDockingGoal()
    client.send_goal(goal_msg)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    return client.get_result()


def move_robot_randomly():
    # change robot pose
    _x = random.uniform(operating_area['x'][0], operating_area['x'][1])
    _y = random.uniform(operating_area['y'][0], operating_area['y'][1])
    _yaw = random.uniform(operating_area['z'][0], operating_area['z'][1])
    pose_cmd = (f"gz model -m {model_name} "
                f"-x {_x:.2f} -y {_y:.2f} -Y {_yaw:.2f}")
    print(f"Move robot with command: [{pose_cmd}]")
    os.system(pose_cmd)


def get_robot_pose():
    get_pose_cmd = (f"gz model -p -m {model_name}")

    print(f"Get Pose with command: [{get_pose_cmd}]")
    ret = subprocess.check_output(get_pose_cmd, shell=True).decode("utf-8")
    pose = ret.strip().split(" ")
    return float(pose[0]), float(pose[1]), float(pose[5])


def check_result(pose, result, allowance=(0.06, 0.06, 0.06)):
    x_allowance = allowance[0]
    y_allowance = allowance[1]
    yaw_allowance = allowance[2]
    if result is None or not result.is_success:
        print("invalid action result")
        return False

    # compare robot edge to charging station edge distance, with safety tol
    _x = abs(pose[0]) - cam_offset - stop_distance
    if _x > x_allowance:
        print(f"Err: exceed x {_x} allowance of {x_allowance}")
        return False

    if abs(pose[1]) > y_allowance:
        print(f"Err: exceed y {pose[1]} allowance of {y_allowance}")
        return False
    if abs(pose[2]) > yaw_allowance:
        print(f"Err: exceed yaw  {pose[2]} allowance of {yaw_allowance}")
        return False
    return True


##############################################################################


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--count", type=int, default=1,
                        help="loop count, default 1")
    parser.add_argument("-x", "--x_allowance", type=float, default=0.03,
                        help="x allowance, default 0.03")
    parser.add_argument("-y", "--y_allowance", type=float, default=0.03,
                        help="y allowance, default 0.03")
    parser.add_argument("-Y", "--yaw_allowance", type=float, default=0.03,
                        help="yaw allowance, default 0.03")
    args = parser.parse_args()

    _allowance = (args.x_allowance, args.y_allowance, args.yaw_allowance)
    print(f"Start Docking Test Script, with {args.count} random requests")
    print(f"Operating Area is {operating_area}")
    print(f"Allowance, (x, y, yaw) is {_allowance}")

    rospy.init_node('test_dock_client')

    for i in range(args.count):
        try:
            print("\n" + "-----"*5 + f"Start {i}" + "-----"*5)
            move_robot_randomly()
            result = autodock_client()
            print("-----"*5 + "Done" + "-----"*5)
            print("Action response:   ", result)
            robot_pose = get_robot_pose()
            print("Robot is located at: ", robot_pose)
            print("-----"*5 + "Check" + "-----"*5)
            check = check_result(robot_pose, result, allowance=_allowance)
            print("Check result: ", check)
            assert check, "Failed to Dock"
        except rospy.ROSInterruptException:
            print("program interrupted before completion")
