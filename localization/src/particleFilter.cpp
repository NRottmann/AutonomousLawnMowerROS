/*
	Particle filter for localization from odometry and sensordata
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <interfaces/IMU.h>
#include <interfaces/Odometry.h>
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
        ParticleFilter(ros::NodeHandle nh, ros::NodeHandle nhp);
    private:
        interfaces::Odometry msg_odometry;
        // TODO: We need to options: (1) Give a predefined position, (2) No positions at all, all particles equally distributed
        Vector3f pos0;
        float L;
        // TODO: Parameter in config
        float A[4] = { 0.0849, 0.0412, 0.0316, 0.0173 };
        float DeltaR1;
        float DeltaT;
        float DeltaR2;

        int n_P;
        int n_S;
        int threshholdReampling;
        MatrixXf Particles = MatrixXf::Zero(n_P, 4);
        MatrixXf ParticleMeasurements = MatrixXf::Zero(n_P, 2);

        void initParticles();
        void updateParticles(int sensorData[2]);
        void odometryData(Vector3f pos0, float l_R, float l_L);
        ros::Subscriber sub_odometry;
        ros::Publisher posePub;
};

ParticleFilter::ParticleFilter(ros::NodeHandle nh, ros::NodeHandle nhp) {
	sub_odometry = nh.subscribe("odometryData", 1000, &ParticleFilter::callbackOdometry, this);
    posePub = nh.advertise<localization::Pose>("particleFilterPose", 1000);
    pos0 = Vector3f::Zero();
}

void ParticleFilter::callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in)
{
	  interfaces::Odometry msg_new = *msg_in;
    float l_R = msg_new.l_R;
    float l_L = msg_new.l_L;
    odometryData(pos0, l_R, l_L);
}

void ParticleFilter::initParticles()
{
    for (int i = 0; i < n_P; i = i+1) {
        Particles(i,1) = pos0(1);
        Particles(i,2) = pos0(2);
        Particles(i,3) = pos0(3);
    }
}

void ParticleFilter::updateParticles(int sensorData[2])
{
    for (int i = 0; i < n_P; i = i+1) {
        // update particles and noise them
        Vector3f odomPose = odometryPose(pos0);
        Particles(i,1) = odomPose(1);
        Particles(i,2) = odomPose(2);
        Particles(i,3) = odomPose(3);

        // fill simulated measurements
    }

    float weightSum = 0;
    for (int i = 0; i < n_P; i = i+1) {
        // update weights
        int w1 = 1 - abs(sensorData[1] - ParticleMeasurements(i,1));
        int w2 = 1 - abs(sensorData[2] - ParticleMeasurements(i,2));
        Particles(i,4) = ((w1+w2)/2)+0.1;
        weightSum = weightSum + Particles(i,4);
    }

    float squareSum = 0;
    for (int i = 0; i < n_P; i = i+1) {
        Particles(i,4) = Particles(i,4)/weightSum;
        squareSum = squareSum + (Particles(i,4)*Particles(i,4));
    }

    float n_Eff = squareSum/n_P;

    // resampling
    // TODO:  For random resampling:
    //       https://github.com/haithamkhedr/Particle-Filter/blob/master/src/particle_filter.cpp
    //       https://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    // Add also later systematic resampling adden
    if(n_Eff < threshholdReampling) {
        VectorXf weightVec = VectorXf::Zero(n_P+1);
        for(int i = 0; i < n_P; i = i+1){
            weightVec(i+1) = weightVec(i) + Particles(i,4);
        }
        VectorXf X = VectorXf::Zero(n_P);
        float random = rand();
        for(int i = 0; i < n_P; i=i+1){
            X(i) = i*(1/n_P) + random;
            if (X(i) > 1) {
                X(i) - 1;
            }
        }

    }
}

void ParticleFilter::odometryData(Vector3f pos0, float l_R, float l_L)
{
  // TODO: USe always [0,0,0] as staring position for determine the Deltas
    float ds = (l_R - l_L) / 2;
    float dphi = (l_R + l_L) / (2*L);
    Vector2f d;
    d << ds,
         dphi;
    MatrixXf R(3, 2);
    R << std::cos(pos0[2]), 0,
         std::sin(pos0[2]), 0,
         0,1;
    Vector3f dp = R*d;

    if (ds >= 0) {
        DeltaR1 = std::atan2(dp[1], dp[0]-pos0[2]);
        DeltaT = std::sqrt(dp[0]*dp[1]+dp[1]*dp[1]);
        if (std::abs(DeltaR1) > std::abs(dp[2]))
            DeltaR1 = 0.5 * dp[2];
        DeltaR2 = dp[2] - DeltaR1;
    } else {
        DeltaR1 = std::atan2(-dp[1], -dp[0]-pos0[2]);
        DeltaT = -std::sqrt(dp[0]*dp[0]+dp[1]*dp[1]);
        if (std::abs(DeltaR1) > std::abs(dp[2]))
            DeltaR1 = 0.5 * dp[2];
        DeltaR2 = dp[2] - DeltaR1;
    }
}

Vector3f ParticleFilter::odometryPose(Vector3f pos0)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> d(0, A[0]*abs(DeltaR1) + A[1]*DeltaT);
    float sample = d(gen);
    float deltaR1 = DeltaR1 + sample;
    std::normal_distribution<float> b(0, A[2]*DeltaT + A[3]*abs(DeltaR1+DeltaR2));
    sample = b(gen);
    float deltaT = DeltaT + sample;
    std::normal_distribution<float> c(0, A[0]*abs(DeltaR2) + A[1]*DeltaT);
    sample = c(gen);
    float deltaR2 = DeltaR2 + sample;
    Vector2f delta;
    delta << deltaT,
             deltaR1+deltaR2;
    MatrixXf deltaP(3, 2);
    deltaP << cos(pos0[2]+deltaR1), 0,
              sin(pos0[2]+deltaR1), 0,
              0,1;
    Vector3f pos1 = pos0 + deltaP*delta;
    return pos1;
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
	// Loop rate
    ros::Rate loop_rate(10);
	// Start the loop
    ros::spin();
	return 0;
}
