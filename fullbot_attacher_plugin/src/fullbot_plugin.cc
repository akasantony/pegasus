#ifndef _FULLBOT_PLUGIN_HH_
#define _FULLBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <ros/ros.h>
#include "fullbot_plugin/Attach.h"
#include "fullbot_plugin/Detach.h"
#include "fullbot_plugin/Reverse.h"
#include "fullbot_plugin/Magnet.h"
#include "fullbot_plugin/magnet_container.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/MagneticField.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>


namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class FullbotPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: FullbotPlugin():
    _nh("fullbot_plugin")
    {
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

      // Store the pointer to the model
      this->sdf = _sdf;
      this->model = _parent;
      this->world = _parent->GetWorld();
      gzdbg << "Loading FullbotPlugin plugin" << std::endl;

      this->attachService = this->_nh.advertiseService("link/attach", &FullbotPlugin::attachCallback, this);

      this->detachService = this->_nh.advertiseService("link/detach", &FullbotPlugin::detachCallback, this);

      this->reverseService = this->_nh.advertiseService("link/reverse", &FullbotPlugin::reverseCallback, this);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&FullbotPlugin::OnUpdate, this));

    }

    private: void QueueThread() {
      static const double timeout = 0.01;

      while (this->rosnode->ok())
      {
        this->queue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private: bool attachCallback(fullbot_plugin::Attach::Request &req, fullbot_plugin::Attach::Response &res) {
    ROS_INFO("******************Calling Attach Func*******************");
    this->parentModel = this->world->ModelByName(req.parent_model);
    this->parentLink = this->parentModel->GetLink(req.parent_link);

    this->childModel = this->world->ModelByName(req.child_model);
    this->childLink = this->childModel->GetLink(req.child_link);

    physics::JointPtr joint = this->parentModel->CreateJoint(req.joint_name, "fixed", this->parentLink,
      this->childLink);

    joint->Attach(parentLink, childLink);

    res.status = true;

    return true;

  }

  private: bool detachCallback(fullbot_plugin::Detach::Request &req, fullbot_plugin::Detach::Response &res) {
    ROS_INFO("******************Calling Detach Func*******************");
    gzdbg << "Loading FullbotPlugin plugin :"<<req.detach_model << std::endl;
    this->detachModel = this->world->ModelByName(req.detach_model);
    std::vector<gazebo::physics::JointPtr> joints = this->detachModel->GetJoints();

    // Loop through all joints and print their names
    ROS_INFO("Model: %s", this->detachModel->GetName().c_str());
    ROS_INFO("Detach Joint Name: %s", &req.detach_joint);
    for (auto joint : joints)
    {
      ROS_INFO("Joint Names: %s", joint->GetName().c_str());
    }
    this->detachJoint = this->detachModel->GetJoint(req.detach_joint);
    this->detachJoint->Detach();
    res.status = true;
    return true;
  }


  bool reverseJointLinks(const physics::JointPtr& joint){
    // Get the current parent and child links
    physics::LinkPtr parentLink = joint->GetParent();
    physics::LinkPtr childLink = joint->GetChild();

    joint->Attach(childLink, parentLink);

    // Update the joint properties
    joint->Update();

    return true;
  }

  private: bool reverseCallback(fullbot_plugin::Reverse::Request &req, fullbot_plugin::Reverse::Response &res) {
    this->reverseModel = this->world->ModelByName(req.reverse_model);
    this->reverseJoint = this->reverseModel->GetJoint(req.reverse_joint);

    bool reverseStatus = this->reverseJointLinks(reverseJoint);

    res.status = reverseStatus;

    return true;
  }




  public: void OnUpdate(){

  }

    private: 
      ros::NodeHandle _nh;
      ros::ServiceServer attachService, detachService, reverseService, magnetAttachService, magnetDetachService;
      int count;  
      physics::WorldPtr world;
      physics::ModelPtr parentModel, childModel, detachModel, reverseModel;
      physics::LinkPtr parentLink, childLink, detachLink, reverseLink, magneticLink;
      physics::JointPtr detachJoint, reverseJoint;
      double k;
      event::ConnectionPtr updateConnection;

      physics::ModelPtr model;
      physics::LinkPtr link;
      sdf::ElementPtr sdf;

      static std::shared_ptr<MagnetContainer::Magnet> mag;

      std::string link_name;
      std::string robot_namespace;
      std::string topic_ns;

      bool should_publish;
      ros::NodeHandle* rosnode;
      ros::Publisher wrench_pub;
      ros::Publisher mfs_pub;

      geometry_msgs::WrenchStamped wrench_msg;
      sensor_msgs::MagneticField mfs_msg;

      private: boost::mutex lock;
      int connect_count;

      common::Time last_time;
      double update_rate;
      ros::CallbackQueue queue;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(FullbotPlugin)
};
#endif
