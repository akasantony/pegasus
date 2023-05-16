/*
 * Copyright (c) 2016, Vanderbilt University
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
 * Author: Addisu Z. Taddese
 */

#ifndef INCLUDE_MAC_GAZEBO_MAGNET_H_
#define INCLUDE_MAC_GAZEBO_MAGNET_H_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <memory>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "fullbot_magnetic_plugin/magnet_container.h"
#include "fullbot_magnetic_plugin/Magnet.h"
#include <ignition/math.hh>

namespace gazebo {

class DipoleMagnet : public ModelPlugin {

 public:
  DipoleMagnet():
  _nh("fullbot")
  {
    ROS_INFO("******** Initiated Constructer ********");
    this->connect_count = 0;
  }

  /// \brief Loads the plugin
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for when subscribers connect
void Connect() {
  this->connect_count++;
}

void Disconnect() {
  this->connect_count--;
}



  /// \brief Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/);

  /// \brief Calculate force and torque of a magnet on another
  /// \parama[in] p_self Pose of the first magnet
  /// \parama[in] m_self Dipole moment of the first magnet
  /// \parama[in] p_other Pose of the second magnet
  /// \parama[in] m_other Dipole moment of the second magnet on which the force is calculated
  /// \param[out] force Calculated force vector
  /// \param[out] torque Calculated torque vector
  void GetForceTorque(const ignition::math::Pose3d& p_self,  const ignition::math::Vector3d& m_self,
      const ignition::math::Pose3d& p_other, const ignition::math::Vector3d& m_other,
      ignition::math::Vector3d& force, ignition::math::Vector3d& torque);

  void QueueThread();

  bool MagnetAttachCallback(fullbot_magnetic_plugin::Magnet::Request& req, fullbot_magnetic_plugin::Magnet::Response& res);
  bool MagnetDetachCallback(fullbot_magnetic_plugin::Magnet::Request& req, fullbot_magnetic_plugin::Magnet::Response& res);

  // Pointer to the model
 private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::LinkPtr magneticLink;

  std::shared_ptr<MagnetContainer::Magnet> mag;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool should_publish;
  ros::NodeHandle* rosnode;
  ros::NodeHandle _nh;
  std::string magneticLink_name;

  geometry_msgs::WrenchStamped wrench_msg;
  sensor_msgs::MagneticField mfs_msg;

  private: boost::mutex lock;
  int connect_count;

  // Custom Callback Queue
  ros::CallbackQueue queue;
  boost::thread callback_queue_thread;

  common::Time last_time;
  double update_rate;
  // Pointer to the update event connection
  event::ConnectionPtr update_connection;

  sdf::ElementPtr sdf;

  ros::ServiceServer magnetAttachService, magnetDetachService;
};

}
#endif  // INCLUDE_MAC_GAZEBO_DIPOLE_MAGNET_DIPOLE_MAGNET_H_
