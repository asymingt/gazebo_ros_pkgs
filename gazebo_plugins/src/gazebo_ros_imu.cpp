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
 * Desc: IMU sensor plugin for ROS
 * Author: Andrew Symington
 * Date: 24 June 2015
 */

#include <gazebo_plugins/gazebo_ros_imu.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImu);

////////////////////////////////////////////////////////////////////////////////
GazeboRosImu::GazeboRosImu() : should_pub(false)
{
  // Do nothing
}

////////////////////////////////////////////////////////////////////////////////
GazeboRosImu::~GazeboRosImu()
{
  this->parent->DisconnectUpdated(this->conUpdate);
  this->rosnode->shutdown();
  delete this->rosnode;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosImu::Load(sensors::SensorPtr sensor, sdf::ElementPtr root)
{
  // Get the parent world and link related to this gps sensor
  this->parent = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensor);
  this->world = physics::get_world(this->parent->GetWorldName());
  this->link = boost::dynamic_pointer_cast<physics::Link>(
    this->world->GetEntity(this->parent->GetParentName()));

  // Load the namespace
  std::string robot_namespace = "/imu";
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
    this->pub_imu = this->rosnode->advertise<sensor_msgs::Imu>(topic_name,1000);
  }

  // ROS callback queue for processing subscription
  this->conUpdate = this->parent->ConnectUpdated(
    boost::bind(&GazeboRosImu::OnUpdate, this));
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosImu::OnUpdate()
{
  // Get the sensor data and the measurement time
  msgs::IMU data = parent->GetImuMessage();
  common::Time stamp = parent->GetLastMeasurementTime();
  math::Quaternion orientation = msgs::Convert(data.orientation());
  math::Vector3 angular_velocity = msgs::Convert(data.angular_velocity());
  math::Vector3 linear_acceleration = msgs::Convert(data.linear_acceleration());

  // Assemble the message
  this->msg_imu.header.stamp = ros::Time(stamp.sec, stamp.nsec);
  this->msg_imu.angular_velocity.x = angular_velocity.x;
  this->msg_imu.angular_velocity.y = angular_velocity.y;
  this->msg_imu.angular_velocity.z = angular_velocity.z;
  this->msg_imu.linear_acceleration.x = linear_acceleration.x;
  this->msg_imu.linear_acceleration.y = linear_acceleration.y;
  this->msg_imu.linear_acceleration.z = linear_acceleration.z;
  this->msg_imu.orientation.w = orientation.w;
  this->msg_imu.orientation.x = orientation.x;
  this->msg_imu.orientation.y = orientation.y;
  this->msg_imu.orientation.z = orientation.z;
  if (this->should_pub)
    this->pub_imu.publish(this->msg_imu);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosImu::Reset()
{
  // Do nothing
}