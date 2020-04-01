/*
	coverage obstacleMap 
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <interfaces/IMU.h>
#include <interfaces/Odometry.h>
#include <interfaces/Sensor.h>
#include <localization/Pose.h>
#include <Eigen/Eigen>
#include <random>
#include <cmath>

using namespace Eigen;

class Coverage
{
    public:
        Coverage();
    private:
        nav_msgs::OccupancyGrid coverageMap;
        Vector2f pose;

        nav_msgs::OccupancyGrid initCoverageMap();
        void updateCoverageMap(MatrixXf particles, Vector2f meanPose);
        //Todo subscribe to localization pose
};

Coverage::Coverage() {
  pose << std::numeric_limits<float>::infinity(), 
			std::numeric_limits<float>::infinity();
  coverageMap = initCoverageMap();
}

void Coverage::updateCoverageMap(MatrixXf particles, Vector2f meanPose) {
  Vector2f estimate;
    estimate << ceil((meanPose[0]-coverageMap.info.origin.position.x)*(1/coverageMap.info.resolution)),
             ceil((meanPose[1]-coverageMap.info.origin.position.y)*(1/coverageMap.info.resolution));
  if((estimate-pose).norm() > coverageMap.info.resolution) {
    nav_msgs::OccupancyGrid prob = initCoverageMap();
    pose = estimate;
    int nP = particles.rows();
    for (unsigned int i = 0; i < nP; i++) {
      unsigned int idx = static_cast<unsigned char>(ceil((particles(i,0)-prob.info.origin.position.x)*(1/prob.info.resolution)));
      unsigned int idy = static_cast<unsigned char>(ceil((particles(i,1)-prob.info.origin.position.y)*(1/prob.info.resolution)));
      unsigned int id = idx + idy*coverageMap.info.width;
      prob.data[id] += 1/nP;
    }
    for(int jx=0; jx<coverageMap.info.width; jx++) {
      for(int jy=0; jy<coverageMap.info.height; jy++) {
        // Check all positions
        int idx = jx + jy*coverageMap.info.width;
        coverageMap.data[idx] += static_cast<unsigned char>(prob.data[idx] - prob.data[idx]*coverageMap.data[idx]);
      }
    }
  }
}

nav_msgs::OccupancyGrid Coverage::initCoverageMap() {
  nav_msgs::OccupancyGrid map;
  // TODO: Load Occupancy Grid from Topic on request!
  map.info.resolution = 0.1;
  map.info.width = static_cast<unsigned int>(12 / map.info.resolution);
  map.info.height = static_cast<unsigned int>(12 / map.info.resolution);
	map.info.origin.position.x =  -6;							//origin : The 2-D pose of the lower-left pixel in the coverageMap
	map.info.origin.position.y = -6;
	map.info.origin.orientation.w = 1.0f;
  // Resize the data structure
	map.data.resize(map.info.width*map.info.height);
  // init with zeros
  double x0 = -6;
  double y0 = -6;
  for(int jx=0; jx<map.info.width; jx++) {
    for(int jy=0; jy<map.info.height; jy++) {
      // Check all positions
      int idx = jx + jy*map.info.width;
      map.data[idx] = static_cast<unsigned char> (0x00);
    }
  }
  return map;
}


// TODO: Change this
int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "coverage");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the Listener
	Coverage coverage();
	// Start the loop
	ros::spin();
	return 0;
}

