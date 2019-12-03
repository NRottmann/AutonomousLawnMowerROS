package interfaces;

public interface IMU extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/IMU";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\n# Orientation as Quaternion (in degrees)\r\ngeometry_msgs/Quaternion orientation\r\n\r\n# Angular Velocity (in degrees per second)\r\ngeometry_msgs/Vector3 angular_velocity\r\n\r\n# Linear Acceleration (in metre per second squared)\r\ngeometry_msgs/Vector3 linear_acceleration\r\n\r\n# Raw magnetometer data (in micro-Tesla)\r\ngeometry_msgs/Vector3 raw_magnetometer\r\n\r\n# Calibration Status\r\nuint8 sys\r\nuint8 gyro\r\nuint8 accel\r\nuint8 mag";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getLinearAcceleration();
  void setLinearAcceleration(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getRawMagnetometer();
  void setRawMagnetometer(geometry_msgs.Vector3 value);
  byte getSys();
  void setSys(byte value);
  byte getGyro();
  void setGyro(byte value);
  byte getAccel();
  void setAccel(byte value);
  byte getMag();
  void setMag(byte value);
}
