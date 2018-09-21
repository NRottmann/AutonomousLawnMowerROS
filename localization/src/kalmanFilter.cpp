/*
	Extended Kalman Filter for fusing odometry and IMU data
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <interfaces/IMU.h>
#include <interfaces/Odometry.h>
#include <localization/Pose.h>
#include <Eigen/Eigen>

using namespace Eigen;


// Define callback class to listen to IMU and Odometry
class Listener
{
	public:
		void callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in);	// callback function to receive odometry data from the motorInterface
		void callbackIMU(const interfaces::IMU::ConstPtr& msg_in);				// callback function to receive IMU data from imuInterface
		Listener(ros::NodeHandle nh, ros::NodeHandle nhp);						// this is the constructor
	private:
		interfaces::Odometry msg_odometry;
		interfaces::IMU msg_imu;												// message of the IMU at time i-1
		float time;
		// Define vectors and matrices using the Eigen library
		Vector3f p0;															// Pose at i-1
		Vector3f p1;															// Pose at i
		Vector3f p1_imu;														// Pose estimation IMU at i, the IMU samples faster
		Vector3f v_imu;															// Velocities in x and y direction
		Matrix3f P;																// Covariance matrix
		// Parameter
		float L;																// Half of the distance between the wheels
		Vector4f a;																// Odometry covariance parameters
		Vector3f allanN;														// Parameters of Alla deviation for calculating covariance matrix of the IMU
		Vector3f allanB;
		// functions
		Vector3f transformIMU(Vector3f acc, float phi);							// Transformation function for IMU onto world frame
		Vector3f transformOdometry();											// Calculate deltas for odometry model
		Matrix3f calculateQ(float dt, float t);									// Calculate covariance matrix for IMU
		// Subscriber and Publisher
		ros::Subscriber sub_odometry;
		ros::Subscriber sub_imu;
		ros::Publisher posePub;
};
// Constructor
Listener::Listener(ros::NodeHandle nh, ros::NodeHandle nhp) {
	// Get Parameters
	if (!nhp.getParam("robot/L", L)) ROS_ERROR("kalmanFilter: Could not find robot/L parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n L: %f", L);
	if (!nhp.getParam("filter/a1", a(0))) ROS_ERROR("kalmanFilter: Could not find filter/a1 parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n a1: %f", a(0));
	if (!nhp.getParam("filter/a2", a(1))) ROS_ERROR("kalmanFilter: Could not find filter/a2 parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n a2: %f", a(1));
	if (!nhp.getParam("filter/a3", a(2))) ROS_ERROR("kalmanFilter: Could not find filter/a3 parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n a3: %f", a(2));
	if (!nhp.getParam("filter/a4", a(3))) ROS_ERROR("kalmanFilter: Could not find filter/a4 parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n a4: %f", a(3));
	if (!nhp.getParam("filter/nx", allanN(0))) ROS_ERROR("kalmanFilter: Could not find filter/nx parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n nx: %f", allanN(0));
	if (!nhp.getParam("filter/bx", allanB(0))) ROS_ERROR("kalmanFilter: Could not find filter/bx parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n bx: %f", allanB(0));
	if (!nhp.getParam("filter/ny", allanN(1))) ROS_ERROR("kalmanFilter: Could not find filter/ny parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n ny: %f", allanN(1));
	if (!nhp.getParam("filter/by", allanB(1))) ROS_ERROR("kalmanFilter: Could not find filter/by parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n by: %f", allanB(1));
	if (!nhp.getParam("filter/nphi", allanN(2))) ROS_ERROR("kalmanFilter: Could not find filter/nphi parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n nphi: %f", allanN(2));
	if (!nhp.getParam("filter/bphi", allanB(2))) ROS_ERROR("kalmanFilter: Could not find filter/bphi parameter!");
	ROS_INFO("kalmanFilter: Loaded Parameter\n bphi: %f", allanB(2));
	// Initialize Variables
	time = 0;
	p0 = Vector3f::Zero();
	p1 = Vector3f::Zero();
	p1_imu = Vector3f::Zero();
	v_imu = Vector3f::Zero();
	P = Matrix3f::Zero();
	msg_odometry.header.stamp  = ros::Time::now();
	msg_imu.header.stamp = ros::Time::now();
	// Initialize subscribers
	sub_odometry = nh.subscribe("odometryData", 1000, &Listener::callbackOdometry, this);
	sub_imu = nh.subscribe("imuData", 1000, &Listener::callbackIMU, this);
	// Initialize publisher
	posePub = nh.advertise<localization::Pose>("kalmanFilterPose", 1000);
}
// Callback functions
void Listener::callbackOdometry(const interfaces::Odometry::ConstPtr& msg_in)
{
	// In this function the Kalman Filter is implemented
	interfaces::Odometry msg_new = *msg_in;
	float dt_n = (((float)msg_new.header.stamp.nsec - (float)msg_odometry.header.stamp.nsec) / 1000000000.0f);
	if (dt_n < 0)
		(dt_n = 1.0f - dt_n);
	float dt = ((float)msg_new.header.stamp.sec - (float)msg_odometry.header.stamp.sec) + dt_n;
	time = time + dt;
	// Calculate estimated pose of odometry, this is also the estimate used for the Kalman Filter
	// TODO: Change this dirty fix
	if (fabs(msg_new.l_L) > 0.2)
		msg_new.l_L = 0.0f;
	if (fabs(msg_new.l_R) > 0.2)
		msg_new.l_R = 0.0f;
	Vector2f deltaOdo;															// holds ds and dphi for the odometry
	deltaOdo << ((msg_new.l_R - msg_new.l_L) / 2), 
				((msg_new.l_R + msg_new.l_L) / (2 * L));
	MatrixXf T(3, 2);
	T << cosf(p0(2)), 0,
		 sinf(p0(2)), 0,
		 0, 1;
	p1 = p0 + T * deltaOdo;
	// Calculate covariance matrix for the odometry
	Vector3f delta = transformOdometry();
	Matrix3f U;
	U << a(0)*delta(0) + a(1)*delta(1), 0, 0,
		 0, a(2)*delta(1) + a(3)*(delta(0) + delta(2)), 0,
		 0, 0, a(0)*delta(2) + a(1)*delta(1);
	// Calculate Jacobi matrices
	Matrix3f Fx;
	Fx << 1, 0, -delta(1)*sinf(p0(2) + delta(0)),
		  0, 1, delta(1)*cosf(p0(2) + delta(0)),
		  0, 0, 1;
	Matrix3f Fu;
	Fu << -delta(1)*sinf(p0(2) + delta(0)), cosf(p0(2) + delta(0)), 0,
		  delta(1)*cosf(p0(2) + delta(0)), sinf(p0(2) + delta(0)), 0,
		  1, 0, 1;
	// Calculate covariance estimates
	Matrix3f P_hat = Fx * P * Fx.transpose() + Fu * U * Fu.transpose();
	// Calculate IMU covariance matrix
	Matrix3f Q = calculateQ(dt, time);
	// Update estimate
	Matrix3f S = P_hat + Q;
	Matrix3f K = P_hat * S.inverse();
	Matrix3f I = Matrix3f::Identity();
	P = (I - K) * P_hat;
	// Sanity check, TODO: Check out if there is something better
	if (P.minCoeff() < 0) {
		P = P_hat;
	}
	else {
		p1 = p1 + K * p1_imu;
	}
	// Publish pose information
	localization::Pose msg_out;
	for (int i = 0; i<3; i++)
	{
		msg_out.pose[i] = p1(i);
		for (int j = 0; j < 3; j++)
		{
			msg_out.covariance[j + i] = P(i, j);
		}
	}
	msg_out.header.stamp = ros::Time::now();		// Add time data
	posePub.publish(msg_out);
	// Calculate velocities, to correct IMU velocities and update message
	v_imu(0) = (p1(0) - p0(0)) / dt;
	v_imu(1) = (p1(1) - p0(1)) / dt;
	msg_odometry = msg_new;
	p1_imu = p1;
	p0 = p1;
}
void Listener::callbackIMU(const interfaces::IMU::ConstPtr& msg_in)
{
	interfaces::IMU msg_new = *msg_in;
	float dt_n = (((float)msg_new.header.stamp.nsec - (float)msg_imu.header.stamp.nsec) / 1000000000.0f);
	if (dt_n < 0)
		(dt_n = 1.0f - dt_n);
	float dt = ((float)msg_new.header.stamp.sec - (float)msg_imu.header.stamp.sec) + dt_n;						// get time between IMU messages in seconds
	// Transform coordinate system of IMU onto world frame
	Vector3f acc_imu;
	acc_imu << msg_imu.linear_acceleration.x, msg_imu.linear_acceleration.y, 0;				// Linear accelerations in IMU frame
	acc_imu = transformIMU(acc_imu, p1_imu(2));
	// Calculate pose estimate
	p1_imu = p1_imu + v_imu * dt;
	// Update velocities
	v_imu(0) = v_imu(0) + acc_imu(0) * dt;													// velocity in x-direction (world frame)
	v_imu(1) = v_imu(1) + acc_imu(1) * dt;													// velocity in y-direction (world frame)
	v_imu(2) = msg_imu.angular_velocity.z;												    // angular velocity around z-axi
	// Update messages
	msg_imu = msg_new;
}
// Functions
Vector3f Listener::transformIMU(Vector3f acc, float phi) {
	Matrix3f R1; 
	R1 <<	0, -1, 0,
			-1, 0, 0,
			0, 0, -1;						//Rotation matrix around y and z axis
	Matrix3f R2;
	R2 <<	cosf(phi), -sinf(phi), 0,
			sinf(phi), cosf(phi), 0,
			0, 0, 1;						// Rotation around z-axis
	Vector3f result = R2 * R1 * acc;
	return result;
}
Vector3f Listener::transformOdometry() 
{
	Vector3f delta;
	delta(1) = sqrtf(powf((p0(0) - p1(0)), 2) + powf((p0(1) - p1(1)), 2));
	if (delta(1) <= 0.0001)
		delta(0) = 0;
	else
		delta(0) = atan2f(p1(1) - p0(1), p1(0) - p0(0)) - p0(2);
	delta(2) = p1(0) - p0(0) - delta(0);
	return delta;
}
Matrix3f Listener::calculateQ(float dt, float t)
{
	float vX = allanN(0)*(1 / sqrtf(dt));
	float bX = allanB(0)*sqrtf(dt);
	float vY = allanN(1)*(1 / sqrtf(dt));
	float bY = allanB(1)*sqrtf(dt);
	float vPhi = allanN(2)*(1 / sqrtf(dt));
	float bPhi = allanB(2)*sqrtf(dt);
	Matrix3f Q;
	Q << vX * powf(t, 1.5f) + sqrtf(dt / 3.0f) + bX * (powf(t, 2.0f) * 0.5f), 0, 0,
		 0, vY * powf(t, 1.5f) + sqrtf(dt / 3.0f) + bY * (powf(t, 2.0f) * 0.5f), 0,
		 0, 0, (vPhi * sqrtf(dt * t) + bPhi * t);
	return Q;
}



int main(int argc, char **argv)
{
	// Initialize the ROS System
	ros::init(argc, argv, "kalmanFilter");
	// Initialize node handle
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	// Initialize the publisher for the control commands, tell them which message should be published
	// Initialize the subscriber to get the teleop data
	Listener listener(nh, nhp);
	// Loop rate
	double frequency;
	if (!nhp.getParam("system/frequencyLoc", frequency)) ROS_ERROR("kalmanFilter: Could not find system/frequencyLoc parameter!");
	ros::Rate loop_rate(frequency);
	// Start the loop
	ros::Duration(5).sleep();
	while (ros::ok())
	{
		// Subscribe using the callback functions
		ros::spinOnce();
		// Sleep until rate is reached
		loop_rate.sleep();
	}
	return 0;
}

