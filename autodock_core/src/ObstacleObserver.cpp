/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <cmath>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autodock_core/AutoDockingAction.h>

using AutoDockFeedbackMsg = autodock_core::AutoDockingActionFeedback;
using FeedbackStateMsg = autodock_core::AutoDockingFeedback;

class ObstacleObserver
{
// This simple obstacle observer node checks if any obstacle is near to the 
// vicinity of the robot. This mainly serves as a safety node for the current
// autodock action. Essentially, it subscribes to the robot's local_costmap
// and publish a pause to the /pause_dock topic. also, only certain state
// we will activate the pausing capability, so prevent robot stoping when it
// is near the charging station.
public:
  ObstacleObserver(ros::NodeHandle nh) : nh_(nh)
{
  pause_dock_pub_ = nh_.advertise<std_msgs::Bool>("/pause_dock", 3000);

  pause_vicinity_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(
      "/pause_dock_vicinity", 100);

  costmap_sub_ = nh_.subscribe(
      "/move_base/global_costmap/local_costmap", 100,
      &ObstacleObserver::costmap_cb, this);

  autodock_feedback_sub_ = nh_.subscribe(
      "/autodock_action/feedback", 100,
      &ObstacleObserver::autodock_feedback_cb, this);

  /// Default rate: 3.33 hz
  ros_timer_ = nh_.createTimer(
      ros::Duration(0.3), &ObstacleObserver::periodic_pub_cb, this);

  /// Get all rosparams
  /// Pause when occupied pixels are within the vicinity radius (meters)
  nh.param<float>("vicinity_radius", vicinity_radius_, 0.3);
  ROS_INFO("param: [vicinity_radius]: %f", vicinity_radius_);

  /// Pause when occupied pixel is above coverage_percent, ( 0.0 - 1.0 )
  nh.param<float>("coverage_percent", coverage_percent_thresh_, 0.10);
  ROS_INFO("param: [coverage_percent]: %f", coverage_percent_thresh_);

  /// Consider a pixel as occupied if above occupancy_prob ( 0 - 100 )
  nh.param<int>("occupancy_prob", occupancy_prob_thresh_, 60);
  ROS_INFO("param: [occupancy_prob]: %d", occupancy_prob_thresh_);

  // frame_id  of the the vicinity_msg (footprint), mainly for debug
  nh.param<std::string>(
      "base_link_name", vicinity_msg_.header.frame_id, "base_link");

  geometry_msgs::Point32 point;
  for (float rad = 0; rad <= 2 * M_PI; rad += 0.2)
  {
    point.x = vicinity_radius_ * cos(rad);
    point.y = vicinity_radius_ * sin(rad);
    vicinity_msg_.polygon.points.push_back(point);
  }
  ROS_INFO("Done Initialize Obstacle Observer Node");
}

void costmap_cb(
    const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  const auto info = msg->info;
  int checked_pixels = 0;
  int occluded_pixels = 0;

  const float pixel_radius = vicinity_radius_ / info.resolution;
  // TODO: add debug flag
  // ROS_INFO("heard robot costmap %dx%d, pixel_radius: %f",
  //           info.height, info.width, pixel_radius );

  const int a = info.width / 2;
  const int b = info.height / 2;
  const int r_sq = pow(pixel_radius, 2);

  const int x_max = a + pixel_radius;
  const int x_min = a - pixel_radius;
  const int y_max = b + pixel_radius;
  const int y_min = b - pixel_radius;

  for (int y = y_min; y <= y_max; y++)
  {
    for (int x = x_min; x <= x_max; x++)
    {
      // use foot print radius
      if (pow(x - a, 2) + pow(y - b, 2) > r_sq)
        continue;

      checked_pixels++;
      u_int i = x + (info.height - y - 1) * info.width;

      // std::cout << (int)msg->data[i] << '|';
      if (msg->data[i] > occupancy_prob_thresh_)
        occluded_pixels++;
    }
  }

  // std::cout << std::endl;
  // ROS_INFO(" Occlusion pixels count is %d / %d, from %d",
  //          occluded_pixels, checked_pixels, info.height * info.width);

  occupancy_percent_ = occluded_pixels / (float)checked_pixels;
}

void autodock_feedback_cb(
    const AutoDockFeedbackMsg::ConstPtr &msg)
{
  ROS_INFO("ObstacleObserver received autodock state: %d",
           msg->feedback.state);
  dock_state_ = msg->feedback.state;
  publish_msg_fn();
}

void periodic_pub_cb(const ros::TimerEvent &)
{
  publish_msg_fn();
}

void publish_msg_fn()
{
  std_msgs::Bool msg;
  msg.data = false;

  // These states are pause - able
  if (dock_state_ == FeedbackStateMsg::STATE_PREDOCK ||
      dock_state_ == FeedbackStateMsg::STATE_STEER_DOCK ||
      dock_state_ == FeedbackStateMsg::STATE_PAUSE ||
      dock_state_ == FeedbackStateMsg::STATE_PARALLEL_CORRECTION)
  {
    // ROS_INFO("Checking State occupancy percent: %0.3f of State: %d",
    //          occupancy_percent_, dock_state_);

    if (occupancy_percent_ > coverage_percent_thresh_)
    {
      msg.data = true; // is occupied
      ROS_INFO("PAUSE!, ocupancy percent %0.3f > %0.3f thresh",
               occupancy_percent_, coverage_percent_thresh_);
    }
  }
  pause_dock_pub_.publish(msg);
  vicinity_msg_.header.stamp = ros::Time::now();
  pause_vicinity_pub_.publish(vicinity_msg_);
}

private:
ros::NodeHandle nh_;
uint8_t dock_state_ = FeedbackStateMsg::STATE_INVALID;

ros::Subscriber costmap_sub_;
ros::Subscriber autodock_feedback_sub_;
ros::Publisher pause_dock_pub_;
ros::Publisher pause_vicinity_pub_; //for debug
ros::Timer ros_timer_;
geometry_msgs::PolygonStamped vicinity_msg_;

float occupancy_percent_ = 0.0;

// rosparams
float coverage_percent_thresh_;
float vicinity_radius_;
int occupancy_prob_thresh_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_observer");
  std::cout << "Running Obstatcle Observer Node" << std::endl;
  ros::NodeHandle nh("~");
  ObstacleObserver dock_server(nh);
  ros::spin();
}
