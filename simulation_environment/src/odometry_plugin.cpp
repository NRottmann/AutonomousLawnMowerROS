#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "interfaces/Odometry.h"
#include "interfaces/Control.h"
#include <string.h>

namespace gazebo
{
  class RobotControl : public ModelPlugin
  {
	  
	// Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to joint
    private: physics::JointPtr jointLeft;
    private: physics::JointPtr jointRight;
    
    /*// PID controller
    private: common::PID pid;*/
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Handle for the gazebo ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    // old joint positions
    private: double posLeftOld;
    private: double posRightOld;
    
    // Radius of the wheels and axis distance
    double axisDistance;
    double radius;
    
    // ROS publisher
    ros::Publisher rosPub;
    
    // ROS subscriber
    ros::Subscriber rosSub;
	  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {	
	  // Load parameters
	  std::string leftJoint;
	  if (!_sdf->HasElement("leftJoint")) {
		ROS_ERROR("Missing parameter <leftJoint> in odometry_plugin, default to standard");
		leftJoint = "standard";
	  }
	  else {
	    leftJoint = _sdf->GetElement("leftJoint")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: leftJoint = %s", leftJoint.c_str());
	  }
	  std::string rightJoint;
	  if (!_sdf->HasElement("rightJoint")) {
		ROS_ERROR("Missing parameter <rightJoint> in odometry_plugin, default to standard");
		rightJoint = "standard";
	  }
	  else {
	    rightJoint = _sdf->GetElement("rightJoint")->GetValue()->GetAsString();
	    ROS_INFO("odometry_plugin: rightJoint = %s", rightJoint.c_str());
	  }
	  double torque = 1000;
	  if (_sdf->HasElement("torque")) {
		torque = _sdf->Get<double>("torque");
		ROS_INFO("odoemtry_plugin: Set torque to %f", torque);
	  }
	  else
	    ROS_INFO("odoemtry_plugin: Default torque is %f", torque);
	  if (_sdf->HasElement("radius")) {
		radius = _sdf->Get<double>("radius");
		ROS_INFO("odoemtry_plugin: Set radius to %f", radius);
	  }
	  else
	  {
		radius = 0.1075;
	    ROS_INFO("odoemtry_plugin: Default radius is %f", radius);  
	  }
	  if (_sdf->HasElement("axisDistance")) {
		axisDistance = _sdf->Get<double>("axisDistance");
		ROS_INFO("odoemtry_plugin: Set axisDistance to %f", axisDistance);
	  }
	  else
	  {
		axisDistance = 0.1075;
	    ROS_INFO("odoemtry_plugin: Default axisDistance is %f", axisDistance);  
	  }
	  
	  // Initialize values
	  posLeftOld = 0;
	  posRightOld = 0;
			
      // Store the pointer to the model
      this->model = _parent;
      
      // Get the joint
      this->jointLeft = this->model->GetJoint(leftJoint);
      this->jointRight = this->model->GetJoint(rightJoint);
      
      // Configure joint motor, TODO: Get realistic Moment M = (U*I)/(2*pi*n)
      this->jointLeft->SetParam("fmax", 0, torque);
      this->jointRight->SetParam("fmax", 0,torque);
      
      /*//Setup a P-controller, with a gain of 100.
	  this->pid = common::PID(10.0, 0.0, 0.0);
	  // Apply the P-controller to the joints.
      this->model->GetJointController()->SetVelocityPID(this->jointLeft->GetScopedName(), this->pid);
      this->model->GetJointController()->SetVelocityPID(this->jointRight->GetScopedName(), this->pid);*/

	  // Make sure the ROS node for Gazebo has already been initialized  
	  if (!ros::isInitialized())
      {
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
      }
      
      // Reset the ros node name and initialize subscriber and publisher
      this->rosNode.reset(new ros::NodeHandle(""));
      rosPub = this->rosNode->advertise<interfaces::Odometry>("odometryData", 1000);
      rosSub = this->rosNode->subscribe("controlData", 1000, &RobotControl::ROSCallback, this);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RobotControl::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
	  // this->model->GetJointController()->Update();
	  // Get position
	  double posLeft = this->jointLeft->GetAngle(0).Radian();
	  double posRight = this->jointRight->GetAngle(0).Radian();
      // Publish to rostopic odometryData
      interfaces::Odometry msg;
      msg.l_R = radius * (posRight - posRightOld);
      msg.l_L = radius * (posLeft - posLeftOld);
      posRightOld = posRight;
      posLeftOld = posLeft;
      this->rosPub.publish(msg);   
    }
    
    public: void ROSCallback(const interfaces::Control& msg)
	{
	  double velRight = (msg.v + axisDistance * msg.w) / radius;
      double velLeft = (msg.v - axisDistance * msg.w) / radius;
      //ROS_INFO("%f", this->velRight);
      //ROS_INFO("%f", this->velLeft);
      // Set velocities
	  this->jointRight->SetParam("vel", 0, velRight);
	  this->jointLeft->SetParam("vel", 0, velLeft);
      // Set the joint's target velocity
	  /*this->model->GetJointController()->SetVelocityTarget(this->jointLeft->GetScopedName(), 0.1);
	  this->model->GetJointController()->SetVelocityTarget(this->jointRight->GetScopedName(), 0.1);*/
	}

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotControl)
}
