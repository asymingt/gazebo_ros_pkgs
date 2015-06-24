/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_ROS_GPS_HH
#define GAZEBO_ROS_GPS_HH

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
 
#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosGps : public SensorPlugin
  {
    /// \brief Constructor
    public: GazeboRosGps();

    /// \brief Destructor
    public: virtual ~GazeboRosGps();

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    // Called on initialization
    public: void Load(sensors::SensorPtr sensor, sdf::ElementPtr root);
    
    // Reset the sensor
    public: void Reset();

    // Gazebo internals
    private: physics::WorldPtr world;
    private: physics::LinkPtr link;
    private: sensors::GpsSensorPtr parent;
    private: event::ConnectionPtr conUpdate;

    // ROS internals
    private: ros::NodeHandle* rosnode;
    private: ros::Publisher pub;
    private: bool should_pub;
    private: sensor_msgs::NavSatFix msg;
    private: PubQueue<sensor_msgs::NavSatFix>::Ptr pub_queue;
    private: PubMultiQueue pmq;
  };
}
#endif
