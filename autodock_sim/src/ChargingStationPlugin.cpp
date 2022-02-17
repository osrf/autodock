/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include <functional>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/plugins/FlashLightPlugin.hh>

namespace gazebo
{
  class ChargingStationPlugin : public FlashLightPlugin
  {
  public:
    using FlashLightPlugin::Load;
    using FlashLightPlugin::OnUpdate;

    using TriggerReq = std_srvs::Trigger::Request;
    using TriggerRes = std_srvs::Trigger::Response;

    ChargingStationPlugin() :
      charging_led_on_(false)
    {}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      ROS_WARN("\n\t\t START ChargingStationPlugin!");
      FlashLightPlugin::Load(_parent, _sdf);
      load_param(_sdf);

      this->rosnode_ = new ros::NodeHandle();
      this->charging_srv_ = this->rosnode_->advertiseService(
          "trigger_charger_srv",
          &ChargingStationPlugin::TriggerChargerCallback,
          this);

      // TODO sdf reader topic
      _gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
      _gazebo_node->Init();
      _charge_state_sub = _gazebo_node->Subscribe(
          param_.contact_topic,
          &ChargingStationPlugin::contact_state_cb, this);
    }

    // Called by the world update start event
    void OnUpdate()
    {
      FlashLightPlugin::OnUpdate();
      if (charging_led_on_ &&
          ((ros::Time::now() - trigger_charger_time_).sec > param_.reset_timeout))
      {
        ROS_INFO("Switch back to original charging led color");
        this->ChangeColor(
            param_.light_name, param_.light_link_name, param_.original_color);
        charging_led_on_ = false;
      }
    }

  private:
    /// sdf param loader
    void load_param(sdf::ElementPtr _sdf)
    {
      if (_sdf->HasElement("target_color"))
        param_.target_color = _sdf->Get<ignition::math::Color>("target_color");
      else
        ROS_ERROR("<target_color> is not defined");

      if (_sdf->HasElement("contact_topic"))
        param_.contact_topic = _sdf->Get<std::string>("contact_topic");
      else
        ROS_WARN("<contact_topic> is not defined, use default %s",
                 param_.contact_topic.c_str());

      if (_sdf->HasElement("target_collision"))
        param_.target_collision = _sdf->Get<std::string>("target_collision");
      else
        ROS_WARN("<target_collision> is not defined, use default %s",
                 param_.target_collision.c_str());

      if (_sdf->HasElement("reset_timeout"))
        param_.reset_timeout = _sdf->Get<double>("reset_timeout");
      else
        ROS_WARN("<reset_timeout> is not defined, use default %f",
                 param_.reset_timeout);

      // Light Block
      if (_sdf->HasElement("light"))
      {
        sdf::ElementPtr sdfLight = _sdf->GetElement("light");
        if (sdfLight->HasElement("color"))
          param_.original_color = sdfLight->Get<ignition::math::Color>("color");
        else
          ROS_ERROR("original_color: `<light><color>` is not defined");

        if (sdfLight->HasElement("id"))
        {
          std::string lightId = sdfLight->Get<std::string>("id");
          int posDelim = lightId.rfind("/");
          param_.light_name = lightId.substr(posDelim + 1, lightId.length());
          param_.light_link_name = lightId.substr(0, posDelim);
          ROS_WARN("Light name: %s, LinkName %s",
                   param_.light_name.c_str(), param_.light_link_name.c_str());
        }
        else
          gzerr << "Parameter <id> is missing." << std::endl;
      }
      else
        ROS_ERROR("Parameter `<light>` block is not defined");
    }

    void contact_state_cb(ConstContactsPtr &_msg)
    {
      // check if there's any contact,
      // and check if collision is as defined target
      if (_msg->contact().size() != 0 &&
          (param_.target_collision == _msg->contact().Get(0).collision2()))
      {
        // update collision time as this is a valid one
        last_collision_time_ = ros::Time::now();
        // TODO check if body_1_wrench force (x) exceeds a threshhold
        last_collision_verbose_msg_ = _msg->contact().Get(0).DebugString();

        last_collision_x_wrench = 0.0;
        for (const auto wrench : _msg->contact().Get(0).wrench())
        {
          last_collision_x_wrench += wrench.body_1_wrench().force().x();
        }
      }
    }

    // gz transport
    gazebo::transport::NodePtr _gazebo_node;
    gazebo::transport::SubscriberPtr _charge_state_sub;

    // ros node
    ros::NodeHandle *rosnode_;
    ros::ServiceServer charging_srv_;

    // runtime dynamic variables
    ros::Time last_collision_time_;
    std::string last_collision_verbose_msg_;
    double last_collision_x_wrench;
    ros::Time trigger_charger_time_;
    std::atomic<bool> charging_led_on_;

    struct PluginParam
    {
      std::string contact_topic = "gazebo/default/physics/contacts";
      std::string target_collision = "mock_docking_robot::body::collision";
      double reset_timeout = 8.0;
      double max_allowable_force; /// TODO
      ignition::math::Color target_color;
      ignition::math::Color original_color;
      std::string light_name;
      std::string light_link_name;
    };

    PluginParam param_;

    bool TriggerChargerCallback(
        TriggerReq &req, TriggerRes &res)
    {
      ROS_INFO("Received Charging Trigger Request");

      trigger_charger_time_ = ros::Time::now();
      ROS_INFO("Time Debug: last_collision_time: %d.%d",
               last_collision_time_.sec, last_collision_time_.nsec);

      // Check if there's any collision within time duration
      const auto time_diff = trigger_charger_time_ - last_collision_time_;
      if (time_diff.sec > param_.reset_timeout)
      {
        res.message = "No contact betwwen the robot and the charger";
        res.success = false;
      }
      else
      {
        // ROS_INFO("Contact Point: %s", last_collision_verbose_msg_.c_str());
        this->ChangeColor(
            param_.light_name, param_.light_link_name, param_.target_color);
        res.message = "valid contact with robot, total x wrench force: " +
                      std::to_string(last_collision_x_wrench) + " N";
        charging_led_on_ = true;
        res.success = true;
      }
      return true;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ChargingStationPlugin)
}
