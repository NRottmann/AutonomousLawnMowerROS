package interfaces;

public interface Decawave extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Decawave";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nfloat32 x\t\t# position in m\r\nfloat32 y\r\nfloat32 z\r\nfloat32 confidence\t# measure of confidence";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
  float getConfidence();
  void setConfidence(float value);
}
