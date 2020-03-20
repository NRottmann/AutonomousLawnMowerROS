/*
	Particle filter for localization from odometry and sensordata
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <interfaces/IMU.h>
#include <interfaces/Odometry.h>
#include <interfaces/Sensor.h>
#include <localization/Pose.h>
#include <Eigen/Eigen>
#include <random>
#include <cmath>

using namespace Eigen;

class ParticleFilter
{
    public:
        Vector3f odometryPose(Vector3f pos0);
        void callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in);
        void callbackSensor(const interfaces::Sensor::ConstPtr& msg_in);
        ParticleFilter(ros::NodeHandle nh, ros::NodeHandle nhp);
    private:
        interfaces::Odometry msg_odometry;
        // TODO: We need to options: (1) Give a predefined position, (2) No positions at all, all particles equally distributed
        Vector3f pos0;
        float L = 0.1826f;
        // TODO: Parameter in config
        float A[4] = { 0.0849f, 0.0412f, 0.0316f, 0.0173f };
        float DeltaR1;
        float DeltaT;
        float DeltaR2;

        nav_msgs::OccupancyGrid map;

        unsigned int n_P;
        float threshholdReampling;
        MatrixXf Particles;
        MatrixXi ParticleMeasurements;
        Vector2f posLeftSensor;
        Vector2f posRightSensor;
        bool firstTimeOdom = false;

        void initParticles();
        void updateParticles();
        float weightParticles(int sensorData[2]);
        void resampleParticles();
        void estimatePose();
        void getOccupancyGrid();
        bool inPolygon(float x, float y);
        void odometryData(Vector3f pos0, float l_R, float l_L);
        void publishVizParticles();
        ros::Subscriber sub_odometry;
        ros::Subscriber sub_sensor;
        ros::Publisher posePub;
        ros::Publisher particlePub;
};

ParticleFilter::ParticleFilter(ros::NodeHandle nh, ros::NodeHandle nhp) {
	  sub_odometry = nh.subscribe("odometryData", 1000, &ParticleFilter::callbackOdometry, this);
    sub_sensor = nh.subscribe("sensorData", 1000, &ParticleFilter::callbackSensor, this);
    posePub = nh.advertise<localization::Pose>("particleFilterPose", 20);
    particlePub = nh.advertise<visualization_msgs::Marker>("particlePositions",20);
    pos0 = Vector3f::Zero();

    n_P = 100;
    threshholdReampling = 0.9f;

    Particles = MatrixXf::Zero(n_P, 4);
    ParticleMeasurements = MatrixXi::Zero(n_P, 2);
    initParticles();

    posRightSensor << 0.265f, -0.09f;
    posLeftSensor << 0.265f, 0.09f;

    getOccupancyGrid();
}

void ParticleFilter::callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in)
{
    firstTimeOdom = true;
    interfaces::Odometry msg_new = *msg_in;

    // Update the pose
    // TODO: Change this dirty fix
    if (fabsf(msg_new.l_L) > 0.2f)
      msg_new.l_L = 0.0f;
    if (fabsf(msg_new.l_R) > 0.2f)
      msg_new.l_R = 0.0f;
    Vector2f deltaOdo;
    deltaOdo << ((msg_new.l_R + msg_new.l_L) / 2),
          ((msg_new.l_R - msg_new.l_L) / (2 * L));
    MatrixXf R(3, 2);
    R << 1, 0,
         0, 0,
         0, 1;
    Vector3f dp;
    dp = R*deltaOdo;

    // TODO: Check this!
    if (deltaOdo(0) >= 0) {
      DeltaT = sqrtf(dp[0]*dp[0]+dp[1]*dp[1]);
      DeltaR1 = 0.5f * dp[2];
      DeltaR2 = dp[2] - DeltaR1;
    } else {
      DeltaT = -sqrtf(dp[0]*dp[0]+dp[1]*dp[1]);
      DeltaR1 = 0.5f * dp[2];
      DeltaR2 = dp[2] - DeltaR1;
    }

    // Update Particle Poses
    updateParticles();
    // Publish pose estimate
    estimatePose();
}

void ParticleFilter::callbackSensor(const interfaces::Sensor::ConstPtr& msg_in)
{
  if(!firstTimeOdom) {
    return;
  }
	interfaces::Sensor msg_sensor = *msg_in;
    int left = msg_sensor.left;
    int right = msg_sensor.right;
    if (left > 127) {
      left = 1;
    } else {
      left = 0;
    }
    if (right > 127) {
      right = 1;
    } else {
      right = 0;
    }
    // Update weights
    int sensorData[2];
    sensorData[0] = right;
    sensorData[1] = left;
    if(weightParticles(sensorData) > threshholdReampling) {
      resampleParticles();
    }
}

void ParticleFilter::getOccupancyGrid() {
  // TODO: Load Occupancy Grid from Topic on request!
  map.info.resolution = 0.1f;
  map.info.width = static_cast<unsigned int>(12 / map.info.resolution);
  map.info.height = static_cast<unsigned int>(12 / map.info.resolution);
	map.info.origin.position.x =  -6;							//origin : The 2-D pose of the lower-left pixel in the map
	map.info.origin.position.y = -6;
  map.info.origin.orientation.w = 1.0;
  // Resize the data structure
	map.data.resize(map.info.width*map.info.height);
  // Check which points are inside the polygon
  float x0 = -6;
  float y0 = -6;
  for(unsigned int jx=0; jx<map.info.width; jx++) {
    for(unsigned int jy=0; jy<map.info.height; jy++) {
      // Check all positions
      unsigned int idx = jx + jy*map.info.width;
      if(inPolygon(x0+jx*map.info.resolution, y0+jy*map.info.resolution)) {
        map.data[idx] = static_cast<unsigned char>(0x01);
      } else {
        map.data[idx] = static_cast<unsigned char>(0x00);
      }
    }
  }
}

void ParticleFilter::initParticles()
{
    for (unsigned int i = 0; i < n_P; i++) {
        Particles(i,0) = pos0(0);
        Particles(i,1) = pos0(1);
        Particles(i,2) = pos0(2);
        Particles(i,3) = 1/n_P;
    }
}

void ParticleFilter::updateParticles()
{
    for (unsigned int i = 0; i < n_P; i++) {
        // update particles and noise them
        Particles.block(i,0,1,3) = odometryPose(Particles.block(i,0,1,3).transpose()).transpose();

        // fill simulated measurements
        Matrix2f R;
        R << cosf(Particles(i,2)), -sinf(Particles(i,2)),
             sinf(Particles(i,2)), cosf(Particles(i,2));
        Vector2f pos = Particles.block(i,0,1,2).transpose();
        Vector2f pR = pos + R*posRightSensor;
        Vector2f pL = pos + R*posLeftSensor;
        unsigned int idx_pR = static_cast<unsigned char>(floorf((pR(0) - static_cast<float>(map.info.origin.position.x))/map.info.resolution));
        unsigned int idy_pR = static_cast<unsigned char>(floorf((pR(1) - static_cast<float>(map.info.origin.position.y))/map.info.resolution));
        unsigned int idx_pL = static_cast<unsigned char>(floorf((pL(0) - static_cast<float>(map.info.origin.position.x))/map.info.resolution));
        unsigned int idy_pL = static_cast<unsigned char>(floorf((pL(1) - static_cast<float>(map.info.origin.position.y))/map.info.resolution));

        ParticleMeasurements(i,0) = map.data[idx_pR + map.info.width * idy_pR];
        ParticleMeasurements(i,1) = map.data[idx_pL + map.info.width * idy_pL];
    }
    publishVizParticles();
}


float ParticleFilter::weightParticles(int sensorData[2]) {

    float weightSum = 0;
    for (unsigned int i = 0; i < n_P; i = i+1) {
        // update weights
        int w1 = 1 - abs(sensorData[0] - ParticleMeasurements(i,0));
        int w2 = 1 - abs(sensorData[1] - ParticleMeasurements(i,1));
        Particles(i,3) = ((w1+w2)/2)+0.1f;
        weightSum = weightSum + Particles(i,3);
    }

    float squareSum = 0;
    for (unsigned int i = 0; i < n_P; i = i+1) {
        Particles(i,3) = Particles(i,3)/weightSum;
        squareSum = squareSum + (Particles(i,3)*Particles(i,3));
    }

    return (squareSum/n_P);
  }


void ParticleFilter::resampleParticles() {
    // resampling
    // TODO:  For random resampling:
    //       https://github.com/haithamkhedr/Particle-Filter/blob/master/src/particle_filter.cpp
    //       https://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    // Add also later systematic resampling adden

    std::vector<float> weightVec(n_P+1, 0.0f);
    for(unsigned int i = 0; i < n_P; i++){
        weightVec[i+1] = weightVec[i] + Particles(i,3);
    }
        /*VectorXf X = VectorXf::Zero(n_P);
        float random = rand();
        for(int i = 0; i < n_P; i=i+1){
            X(i) = i*(1/n_P) + random;
            if (X(i) > 1) {
                X(i) - 1;
            }
        }*/

    std::default_random_engine gen;
    std::discrete_distribution<int> d(weightVec.begin(), weightVec.end());
    MatrixXf resampled_Particles = MatrixXf::Zero(n_P, 4);
    for(unsigned int i = 0; i < n_P; i++){
        int idx = d(gen);
        resampled_Particles.block(i,0,1,4) = Particles.block(idx-1,0,1,4);
    }
    Particles = resampled_Particles;
}

void ParticleFilter::odometryData(Vector3f pos0, float l_R, float l_L)
{
  // TODO: USe always [0,0,0] as staring position for determine the Deltas
    float ds = (l_R + l_L) / 2;
    float dphi = (l_R - l_L) / (2*L);
    Vector2f d;
    d << ds,
         dphi;
    MatrixXf R(3, 2);
    R << cosf(pos0(2)), 0,
         sinf(pos0(2)), 0,
         0,1;
    Vector3f dp = R*d;

    if (ds >= 0) {
        DeltaR1 = atan2f(dp[1], dp[0]-pos0[2]);
        DeltaT = sqrtf(dp[0]*dp[1]+dp[1]*dp[1]);
        if (fabsf(DeltaR1) > fabsf(dp[2]))
            DeltaR1 = 0.5f * dp[2];
        DeltaR2 = dp[2] - DeltaR1;
    } else {
        DeltaR1 = atan2f(-dp[1], -dp[0]-pos0[2]);
        DeltaT = -sqrtf(dp[0]*dp[0]+dp[1]*dp[1]);
        if (fabsf(DeltaR1) > fabsf(dp[2]))
            DeltaR1 = 0.5f * dp[2];
        DeltaR2 = dp[2] - DeltaR1;
    }
}

Vector3f ParticleFilter::odometryPose(Vector3f pos0)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d(0, A[0]*fabsf(DeltaR1) + A[1]*DeltaT);
    float sample = d(gen);
    float dR1 = DeltaR1 + sample;
    std::normal_distribution<float> b(0, A[2]*DeltaT + A[3]*fabsf(DeltaR1+DeltaR2));
    sample = b(gen);
    float dT = DeltaT + sample;
    std::normal_distribution<float> c(0, A[0]*fabsf(DeltaR2) + A[1]*DeltaT);
    sample = c(gen);
    float dR2 = DeltaR2 + sample;
    Vector2f delta;
    delta << dT,
             dR1+dR2;
    MatrixXf deltaP(3, 2);
    deltaP << cosf(pos0[2]+dR1), 0,
              sinf(pos0[2]+dR1), 0,
              0,1;
    Vector3f pos1 = pos0 + deltaP*delta;
    return pos1;
}

void ParticleFilter::estimatePose() {
  localization::Pose _meanPose;
  _meanPose.pose[0] = Particles.block(0,0,n_P,1).mean();
  _meanPose.pose[1] = Particles.block(0,1,n_P,1).mean();
  _meanPose.pose[2] = Particles.block(0,2,n_P,1).mean();
  posePub.publish(_meanPose);
}

// Check if a point
bool ParticleFilter::inPolygon(float x, float y) {
  // Define polygon structure for the map
  MatrixXf polygons = MatrixXf::Zero(2,6);
  polygons << -5, 5, 5, 1, 1, -5,
              -5, -5, 1, 1, 5, 5;
  int n_vertices = 6;
	int i, j;
	bool c = false;
  for (i = 0, j = n_vertices-1; i < n_vertices; j = i++) {
    if ( ((polygons(1,i) > y) != (polygons(1,j) > y)) &&
     (x < (polygons(0,j) - polygons(0,i)) * (y - polygons(1,i)) / (polygons(1,j) - polygons(1,i)) + polygons(0,i)) )
       c = !c;
  }
  return c;
}

void ParticleFilter::publishVizParticles() {
  visualization_msgs::Marker points;
  points.header.frame_id = "/map";
  points.header.stamp = ros::Time::now();
  points.ns = "particleFilter";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.color.r = 1.0;
  points.color.a = 1.0;
  for (unsigned int i=0; i<n_P; i++) {
    geometry_msgs::Point p;
    p.x = static_cast<double>(Particles(i,0));
    p.y = static_cast<double>(Particles(i,1));
    p.z = 0.0;
    points.points.push_back(p);
  }
  particlePub.publish(points);
}

int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "particleFilter");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the publisher for the control commands, tell them which message should be published
	// Initialize the subscriber to get the teleop data
	ParticleFilter ParticleFilter(nh, nhp);
	// Start the loop
  ros::spin();
	return 0;
}
