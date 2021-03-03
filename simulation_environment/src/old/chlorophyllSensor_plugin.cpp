#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "interfaces/Sensor.h"
#include <string.h>

namespace gazebo
{
  class RobotChloroSensor : public ModelPlugin
  {
	  
	// Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Handle for the gazebo ros node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    // Radius of the wheels and axis distance
    double x_pos;
    double y_pos;
	// TODO: Get the map as polygon parameters
	double x_map[6] = {-5, 5, 5, 1, 1, -5};
	double y_map[6] = {-5, -5, 1, 1, 5, 5};
	int nvert;
			
    // ROS publisher
    ros::Publisher rosPub;
	  
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {	
	  // Load parameters
	  if (_sdf->HasElement("x_pos")) {
		x_pos = _sdf->Get<double>("x_pos");
		ROS_INFO("chlorophyllSensor_plugin: Set x_pos to %f", x_pos);
	  }
	  else
	  {
		x_pos = 0.0;
	    ROS_ERROR("chlorophyllSensor_plugin: x_pos is not set!");  
	  }
	  if (_sdf->HasElement("y_pos")) {
		y_pos = _sdf->Get<double>("y_pos");
		ROS_INFO("chlorophyllSensor_plugin: Set y_pos to %f", x_pos);
	  }
	  else
	  {
		y_pos = 0.0;
	    ROS_ERROR("chlorophyllSensor_plugin: y_pos is not set!");  
	  }
			
      // Store the pointer to the model
      this->model = _parent;
      
      // Get array length
      nvert = sizeof(x_map)/sizeof(*x_map);
      ROS_INFO("nvert: %i", nvert);

	  // Make sure the ROS node for Gazebo has already been initialized  
	  if (!ros::isInitialized())
      {
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
      }
      
      // Reset the ros node name and initialize subscriber and publisher
      this->rosNode.reset(new ros::NodeHandle(""));
      rosPub = this->rosNode->advertise<interfaces::Sensor>("sensorData", 10);
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RobotChloroSensor::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
	  // Get pose
	  ignition::math::Pose3d pose = model->WorldPose();
      // Publish to rostopic sensorData, TODO: add noise
      interfaces::Sensor msg;
      ignition::math::Vector3d eulerAngles = pose.Rot().Euler();
	  double x_r = pose.Pos().X() + cos(eulerAngles.Z())*x_pos + sin(eulerAngles.Z())*y_pos;
	  double y_r = pose.Pos().Y() - cos(eulerAngles.Z())*y_pos + sin(eulerAngles.Z())*x_pos;
	  double x_l = pose.Pos().X() + cos(eulerAngles.Z())*x_pos - sin(eulerAngles.Z())*y_pos;
	  double y_l = pose.Pos().Y() + cos(eulerAngles.Z())*y_pos + sin(eulerAngles.Z())*x_pos;
      if (pnpoly(nvert, x_map, y_map, x_r, y_r))
        msg.right = 200;
      else
        msg.right = 0;
      if (pnpoly(nvert, x_map, y_map, x_l, y_l))
        msg.left = 200;
      else
        msg.left = 0;
      this->rosPub.publish(msg);   
    }
    
    // Ray casting algorithm to check if point is in polygon, TODO: understand and improve Matlab algorithm
    int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
	{
	  int i, j, c = 0;
      for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verty[i]>testy) != (verty[j]>testy)) &&
		  (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
		  c = !c;
		}
	  return c;
	}
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RobotChloroSensor)
}
