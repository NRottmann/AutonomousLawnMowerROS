package localization;

public interface Pose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "localization/Pose";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\n# Pose as vector (x,y,phi)\r\nfloat32[3] pose\r\n\r\n# Covariance as 3x3 matrix\r\nfloat32[9] covariance";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getPose();
  void setPose(float[] value);
  float[] getCovariance();
  void setCovariance(float[] value);
}
