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


import random
import argparse

# ros
import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from autodock_core.msg import AutoDockingAction, AutoDockingGoal


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


def stop_charging(charger_srv_name):
    if not charger_srv_name:
        return False

    print(f"calling stop charging srv: ", charger_srv_name)
    try:
        _trigger = rospy.ServiceProxy(charger_srv_name, Trigger)
        res = _trigger()
        print(f"stop charger srv, "
              f"success [{res.success}] | msg: {res.message}")
        if not res.success:
            print("stop charger failed")
            return False
    except rospy.ServiceException as e:
        print(f"Stop Charger srv call failed: {e}")
        return False


def move_robot_randomly(move_duration, linear_range, angular_range):
    """
    Move the robot to a predocking position from the docking station, by
    using `/cmd_vel` topic.
    :arg move_duration:     How long to move the robot in secs (Float)
    :arg linear_range:      Set with upper and lower bound of the linear vel
    :arg angular_range:     Set wiht upper and lower bound of the angular vel
    """
    print(f"Move Robot with CMD VEL!")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sleep_duration = 0.2
    linear_vel = random.uniform(linear_range[0], linear_range[1])
    angular_vel = random.uniform(angular_range[0], angular_range[1])

    total_rotation = move_duration*angular_vel * 180/3.14
    print(f" =>   cmd_vel: [{linear_vel:.3f}, {angular_vel :.3f}]")
    print(f" => to travel: {move_duration*linear_vel}m, "
          f"turn {total_rotation} degree")
    msg = Twist()
    msg.linear.x = linear_vel
    msg.angular.z = angular_vel

    _loop_count = int(move_duration/sleep_duration)
    for _ in range(_loop_count):
        pub.publish(msg)
        rospy.sleep(rospy.Duration(sleep_duration))
    # stop
    print(f"Reached random position, STOP!!")

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)


##############################################################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--count", type=int, default=1,
                        help="loop count, default 1")
    parser.add_argument("-r", "--retreat_duration", type=float, default=6.0,
                        help="Duration to retreat after one docking cycle")
    parser.add_argument("-ll", "--lower_linear", type=float, default=0.12,
                        help="Lower bound of the linear vel during retreat")
    parser.add_argument("-ul", "--upper_linear", type=float, default=0.20,
                        help="Lower bound of the linear vel during retreat")
    parser.add_argument("-la", "--lower_angular", type=float, default=-0.06,
                        help="Lower bound of the angular vel during retreat")
    parser.add_argument("-ua", "--upper_angular", type=float, default=0.06,
                        help="Upper bound of the angular vel during retreat")
    parser.add_argument("-cs", "--charger_srv_name", type=str, default=None,
                        help="Charging station srv name, default None")
    args = parser.parse_args()

    # This is to indicate the range of the robot vel during retreat after
    # one docking cycle.
    linear_range = (args.lower_linear, args.upper_linear)
    angular_range = (args.lower_angular, args.upper_angular)

    print(f"Start Docking Test Script, with {args.count} random requests")
    rospy.init_node('test_dock_client')

    for i in range(args.count):
        try:
            print("\n" + "-----"*5 + f"Start {i}" + "-----"*5)
            stop_charging(args.charger_srv_name)
            move_robot_randomly(
                args.retreat_duration, linear_range, angular_range)

            print("-----"*5 + "Start Dock" + "-----"*5)
            result = autodock_client()

            print("-----"*5 + "Done" + "-----"*5)
            print("Action response:   ", result)
            assert result.is_success, "Failed to Dock"

        except rospy.ROSInterruptException:
            print("program interrupted before completion")
