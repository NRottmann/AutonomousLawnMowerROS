#!/usr/bin/env python
# Simple Adafruit BNO055 sensor reading example.  Will print the orientation
# and calibration data every second.

import logging
import sys
import rospy
import pickle

from Adafruit_BNO055 import BNO055
from interfaces.msg import IMU

# Create and configure the BNO sensor connection. 
bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=10)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

def imuInterface():
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        rospy.logerr('Failed to initialize BNO055! Is the sensor connected?')
		
    pub = rospy.Publisher('imuData', IMU, queue_size=10)
    rospy.init_node('imuInterface', anonymous=True)

    	# load the calibration
    with open('/home/pi/catkin_ws/src/interfaces/bno055_driver/src/calibData.yaml', 'rb') as fp:
        itemlist = pickle.load(fp)
    bno.set_calibration(itemlist)

    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))

    # frequency = rospy.get_param('system/frequency')
    rate = rospy.Rate(100)
	
    # define message
    msg = IMU()

    while not rospy.is_shutdown():
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated. 
        msg.sys, msg.gyro, msg.accel, msg.mag = bno.get_calibration_status()
		# Orientation as a quaternion:
        msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w = bno.read_quaternion()
		# Magnetometer data (in micro-Teslas):
        msg.raw_magnetometer.x,msg.raw_magnetometer.y,msg.raw_magnetometer.z = bno.read_magnetometer()
		# Gyroscope data (in degrees per second):
        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z = bno.read_gyroscope()
		# Linear acceleration data (i.e. acceleration from movement, not gravity--
		# returned in meters per second squared):
        msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z= bno.read_linear_acceleration()
		# Get time for header and publish message
        msg.header.stamp = rospy.get_rostime()
        pub.publish(msg)
        rate.sleep()
		
if __name__ == '__main__':
    try:
        imuInterface()
    except rospy.ROSInterruptException:
        pass
