#!/usr/bin/env python
# Calibration program for the BNO055 Sensor
import logging
import sys
import time
import pickle

from Adafruit_BNO055 import BNO055

# Raspberry Pi configuration with serial UART and RST connected to GPIO 10:
bno = BNO055.BNO055(serial_port='/dev/ttyS0', rst=10)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))
    # check if calibration has been successful, if so, store the calibration data and exit the loop
    if (sys == 3 & accel == 3 & gyro == 3 & mag == 3):
        cal_data = bno.get_calibration()
        with open('calibData.yaml', 'wb') as fp:
            pickle.dump(cal_data, fp)
        fp.close()
        print('Stored calibration data!')
        exit()
    else:
        # Print calibration status
        print('Sys_cal={0} Gyro_cal={1} Accel_cal={2} Mag_cal={3}'.format(sys, gyro, accel, mag))

    time.sleep(1)
