/*
 * Copyright 2013 Open Source Robotics Foundation
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
/*
 * Desc: 3D position interface for ground truth.
 * Author: Andrew Symington
 * Date: 24 June 2015
 */

#include <gazebo_plugins/gazebo_ros_gps.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGps);

////////////////////////////////////////////////////////////////////////////////
GazeboRosGps::GazeboRosGps() : should_pub(false)
{
  // Do nothing
}

////////////////////////////////////////////////////////////////////////////////
GazeboRosGps::~GazeboRosGps()
{
  this->parent->DisconnectUpdated(this->conUpdate);
  this->rosnode->shutdown();
  delete this->rosnode;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGps::Load(sensors::SensorPtr sensor, sdf::ElementPtr root)
{
  // Get the parent world and link related to this gps sensor
  this->parent = boost::dynamic_pointer_cast<sensors::GpsSensor>(sensor);
  this->world = physics::get_world(this->parent->GetWorldName());
  this->link = boost::dynamic_pointer_cast<physics::Link>(
    this->world->GetEntity(this->parent->GetParentName()));

  // Load the namespace
  std::string robot_namespace = "/gps/";
  if (root->HasElement("namespace"))
    robot_namespace = root->Get<std::string>("namespace") + "/";

  // Load the topic
  std::string topic_name = "solution";
  if (root->HasElement("topic"))
    topic_name = root->Get<std::string>("topic");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Create a new ROS handle
  this->rosnode = new ros::NodeHandle(robot_namespace);
  if (topic_name != "")
  {
    should_pub = true;
    this->pub_gps = this->rosnode->advertise<sensor_msgs::NavSatFix>(topic_name+(std::string)"_lla",1000);
    this->pub_point = this->rosnode->advertise<geometry_msgs::Point>(topic_name+(std::string)"_xyz",1000);
  }

  // ROS callback queue for processing subscription
  this->conUpdate = this->parent->ConnectUpdated(
    boost::bind(&GazeboRosGps::OnUpdate, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGps::OnUpdate()
{
  // Populate the message
  common::Time stamp = parent->GetLastMeasurementTime();
  this->msg_gps.header.stamp = ros::Time(stamp.sec,stamp.nsec);
  this->msg_gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  this->msg_gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  this->msg_gps.latitude = parent->GetLatitude().Degree();
  this->msg_gps.longitude = parent->GetLongitude().Degree();
  this->msg_gps.altitude = parent->GetAltitude();
  if (this->should_pub)
    this->pub_gps.publish(this->msg_gps);

  // Publish 
  math::Vector3 lla(msg_gps.latitude, msg_gps.longitude, msg_gps.altitude);
  math::Vector3 xyz = world->GetSphericalCoordinates()->LocalFromSpherical(lla);
  this->msg_point.x = xyz.x;
  this->msg_point.y = xyz.y;
  this->msg_point.z = xyz.z;
  if (this->should_pub)
    this->pub_point.publish(this->msg_point);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGps::Reset()
{
  // Do nothing
}