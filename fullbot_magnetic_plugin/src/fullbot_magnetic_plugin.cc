#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <vector>
#include <cstdint>

#include "fullbot_magnetic_plugin/magnet.h"
#include "fullbot_magnetic_plugin/Magnet.h"
#include <ignition/math.hh>

namespace gazebo {
  // DipoleMagnet::DipoleMagnet():
  // _nh("fullbot_magnet_plugin")
  // {
  //   ROS_INFO("******** Initiated Constructer ********");
  //   this->connect_count = 0;
  // }

// DipoleMagnet::~DipoleMagnet() {
//   gazebo::event::Events::DisconnectWorldUpdateBegin(this->update_connection);
//   if (this->should_publish) {
//     this->queue.clear();
//     this->queue.disable();
//     this->rosnode->shutdown();
//     this->callback_queue_thread.join();
//     delete this->rosnode;
//   }
//   if (this->mag){
//     MagnetContainer::Get().Remove(this->mag);
//   }
// }


void DipoleMagnet::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_INFO("******** Load Func ********");
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;
  gzdbg << "  poleMagnet plugin" << std::endl;

  this->mag = std::make_shared<MagnetContainer::Magnet>();

  // load parameters
  // this->robot_namespace = "";
  // if (_sdf->HasElement("robotNamespace"))
  //   this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  // ROS_INFO("Debug Point 1");

  if(!_sdf->HasElement("bodyName")) {
    gzerr << "DipoleMagnet plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  }else {
    this->magneticLink_name = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  this->magneticLink = this->model->GetLink(this->magneticLink_name);
  if(!this->magneticLink){
    gzerr << "Error: link named " << this->magneticLink_name << " does not exist" << std::endl;
    return;
  }

  this->should_publish = false;
  if (_sdf->HasElement("shouldPublish"))
  {
    this->should_publish = _sdf->GetElement("shouldPublish")->Get<bool>();
  }
  ROS_INFO("Debug Point 2");

  if (!_sdf->HasElement("updateRate"))
  {
    gzmsg << "DipoleMagnet plugin missing <updateRate>, defaults to 0.0"
        " (as fast as possible)" << std::endl;
    this->update_rate = 0;
  }
  else
    this->update_rate = _sdf->GetElement("updateRate")->Get<double>();

  ROS_INFO("Debug Point 3");

  if (_sdf->HasElement("calculate")){
    this->mag->calculate = _sdf->Get<bool>("calculate");
  } else
    this->mag->calculate = true;
  
  ROS_INFO("Debug Point 4");

  if (_sdf->HasElement("dipole_moment")){
    this->mag->moment = _sdf->Get<ignition::math::Vector3d>("dipole_moment");
  }

  ROS_INFO("Debug Point 5");

  if (_sdf->HasElement("xyzOffset") && _sdf->HasElement("rpyOffset")){
    ignition::math::Vector3d rpy_offset = _sdf->Get<ignition::math::Vector3d>("rpyOffset");
    this->mag->offset.Set(_sdf->Get<ignition::math::Vector3d>("xyzOffset"), ignition::math::Quaternion(rpy_offset));
  }

  ROS_INFO("Debug Point 6");
  

  if (this->should_publish) {
    if (!_sdf->HasElement("topicNs"))
    {
      gzmsg << "DipoleMagnet plugin missing <topicNs>," 
          "will publish on namespace " << this->link_name << std::endl;
    }
    else {
      this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();
    }

    if (!ros::isInitialized())
    {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
        "the gazebo_ros package. If you want to use this plugin without ROS, "
        "set <shouldPublish> to false" << std::endl;
      return;
    }

    // this->rosnode = new ros::NodeHandle(this->robot_namespace);
    this->_nh.setCallbackQueue(&this->queue);

     // Custom Callback Queue
    this->callback_queue_thread = boost::thread(boost::bind( &DipoleMagnet::QueueThread, this));
  }

  ROS_INFO("Debug Point 7");

  this->magnetAttachService = this->_nh.advertiseService("link/set_dipole/on", &DipoleMagnet::MagnetAttachCallback, this);

  this->magnetDetachService = this->_nh.advertiseService("link/set_dipole/off", &DipoleMagnet::MagnetDetachCallback, this);


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DipoleMagnet::OnUpdate, this,_1));
}

void DipoleMagnet::QueueThread() {
      static const double timeout = 0.01;

      while (this->_nh.ok())
      {
        this->queue.callAvailable(ros::WallDuration(timeout));
      }
    }



bool DipoleMagnet::MagnetAttachCallback(fullbot_magnetic_plugin::Magnet::Request &req, fullbot_magnetic_plugin::Magnet::Response &res) {
    this->model = this->world->ModelByName(req.model_name);
    this->magneticLink = this->model->GetLink(req.link_name);

    ROS_INFO("Attaching Magnet on Model: %s Link: %s", this->model->GetName().c_str(), this->magneticLink->GetName().c_str());    

    float dipoleX = req.dipole_x;
    float dipoleY = req.dipole_y;
    float dipoleZ = req.dipole_z;

    // ROS_INFO("Magnetic Force of Link is %s", this->model->GetName().c_str()); 
    // ROS_INFO("Magnetic Force of Link is %s", this->magneticLink->GetName().c_str()); 
    // ROS_INFO("Magnetic Force of Link is %f", dipoleZ); 
    this->mag->moment = ignition::math::Vector3d(dipoleX, dipoleY, dipoleZ);

    this->mag->model_id = this->model->GetId();

    // this->mag->offset.Set(ignition::math::Vector3d>(offsetX, offsetY, offsetZ), ignition::math::Quaternion(rpy_offset));

    std::shared_ptr<MagnetContainer::Magnet> mag2 = std::make_shared<MagnetContainer::Magnet>();
    mag2->moment = this->mag->moment;
    mag2->offset = this->mag->offset;
    mag2->pose = this->mag->pose;
    mag2->model_id = this->mag->model_id;


    MagnetContainer::Get().Add(mag2);

    res.status = true;
    return true;

  }

  bool DipoleMagnet::MagnetDetachCallback(fullbot_magnetic_plugin::Magnet::Request &req, fullbot_magnetic_plugin::Magnet::Response &res) {

    this->mag->moment = ignition::math::Vector3d(0.0, 0.0, 0.0);


    res.status = true;
    return true;

  }


// Called by the world update start event
void DipoleMagnet::OnUpdate(const common::UpdateInfo & /*_info*/) {

  // Calculate the force from all other magnets
  ignition::math::Pose3d p_self = this->magneticLink->WorldPose();
  ignition::math::Quaterniond rot_;
  std::stringstream ss;
  
  p_self.Set(p_self.Pos() + -p_self.Rot().RotateVector(this->mag->offset.Pos()), p_self.Rot() * this->mag->offset.Rot().Inverse());

  this->mag->pose = p_self;

  if (!this->mag->calculate)
    return;

  MagnetContainer& dp = MagnetContainer::Get();


  ignition::math::Vector3d moment_world = p_self.Rot().RotateVector(this->mag->moment);

  ignition::math::Vector3d force(0, 0, 0);
  ignition::math::Vector3d torque(0, 0, 0);
  ignition::math::Vector3d mfs(0, 0, 0);
  
  for(MagnetContainer::MagnetPtrV::iterator it = dp.magnets.begin(); it < dp.magnets.end(); it++){
    size_t length = std::distance(dp.magnets.begin(), dp.magnets.end());
    ROS_INFO("MC Length: %ld", length);
    std::shared_ptr<MagnetContainer::Magnet> mag_other = *it;
    ROS_INFO("Model1: %d, Model2: %d", mag_other->model_id, this->mag->model_id);
    if (mag_other->model_id != this->mag->model_id) {
      ROS_INFO("If Called");
      ROS_INFO("Pose position (x,y,z) of mag_other: (%f, %f, %f)", mag_other->pose.Pos().X(), mag_other->pose.Pos().Y(), mag_other->pose.Pos().Z());
      ROS_INFO("Pose position (x,y,z) of mag: (%f, %f, %f)", mag->pose.Pos().X(), mag->pose.Pos().Y(), mag->pose.Pos().Z());

      ignition::math::Pose3d p_other = mag_other->pose;
      ignition::math::Vector3d m_other = p_other.Rot().RotateVector(mag_other->moment);

      ignition::math::Vector3d force_tmp;
      ignition::math::Vector3d torque_tmp;
      
      GetForceTorque(p_self, moment_world, p_other, m_other, force_tmp, torque_tmp);

      force += force_tmp;
      torque += torque_tmp;


      this->magneticLink->AddForce(force_tmp);
      this->magneticLink->AddTorque(torque_tmp);
      ss << force_tmp.X() << " " << force_tmp.Y() << " " << force_tmp.Z();
      std::string vector_str = ss.str();

      ROS_INFO("Magnetic Force on Link: %s -> %s", this->magneticLink->GetName().c_str(), vector_str.c_str()); 
    }

  }
}

void DipoleMagnet::GetForceTorque(const ignition::math::Pose3d& p_self,
    const ignition::math::Vector3d& m_self,
    const ignition::math::Pose3d& p_other,
    const ignition::math::Vector3d& m_other,
    ignition::math::Vector3d& force,
    ignition::math::Vector3d& torque) {

  bool debug = false;
  ignition::math::Vector3d p = p_self.Pos() - p_other.Pos();
  ignition::math::Vector3d p_unit = p/p.Length();

  ignition::math::Vector3d m1 = m_other;
  ignition::math::Vector3d m2 = m_self;
  if (debug)
    std::cout << "p: " << p << " m1: " << m1 << " m2: " << m2 << std::endl;

  double K = 3.0*1e-7/pow(p.Length(), 4);
  force = K * (m2 * (m1.Dot(p_unit)) +  m1 * (m2.Dot(p_unit)) +
      p_unit*(m1.Dot(m2)) - 5*p_unit*(m1.Dot(p_unit))*(m2.Dot(p_unit)));

  double Ktorque = 1e-7/pow(p.Length(), 3);
  ignition::math::Vector3d B1 = Ktorque*(3*(m1.Dot(p_unit))*p_unit - m1);
  torque = m2.Cross(B1);
  if (debug)
    std::cout << "B: " << B1 << " K: " << Ktorque << " t: " << torque << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DipoleMagnet)

}