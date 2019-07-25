package interfaces;

public interface Control extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Control";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nfloat32 v\t\t# velocity in m/s in x-direction\r\nfloat32 w\t\t# angular velocity in rad/s";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getV();
  void setV(float value);
  float getW();
  void setW(float value);
}
