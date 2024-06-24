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
from sensor_msgs.msg import CameraInfo
from rosgraph_msgs.msg import Clock

##############################################################################


class CameraInfoFilter:
    """
    This simple filtering node exists due to the messy population of the
    '/camera_info' topic. In the gaussian robot, the camera info topic is
    target topic of multiple rgb and depth img output. This is not helpful
    in anyway. Thus, this node will extract the camera (frame_id)_ which we
    are interested, and publish its camera info to a seperate topic.
    """

    def __init__(self):
        rospy.init_node('camera_info_filter')

        self.extract_frame_id = rospy.get_param(
            "~extract_frame_id", "intrinsic_color2")

        self.new_frame_id = rospy.get_param(
            "~new_frame_id", "back_camera_frame")

        # create_subscriber to messy camera_info
        rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_cb)

        # create_publisher to camera info
        self.camera_info_pub = rospy.Publisher(
            '/filtered_camera_info', CameraInfo, queue_size=5)

        # Note: This is extremely Dangerous. Use this with caution
        # create_publisher to Clock
        rospy.logwarn("Danger, fake_clock mode! Publish host time to /clock")
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=5)

        rospy.loginfo(
            "Staring Camera Info Filtering for [%s] -> [%s]",
            self.extract_frame_id, self.new_frame_id)
        rospy.spin()

    def camera_info_cb(self, msg: CameraInfo):
        if (msg.header.frame_id == self.extract_frame_id):
            msg.header.frame_id = self.new_frame_id
            self.camera_info_pub.publish(msg)

            # to publish a fake clock
            self.clock_pub.publish(msg.header.stamp)

##############################################################################
if __name__ == "__main__":
    node = CameraInfoFilter()
