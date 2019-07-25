package interfaces;

public interface Bumper extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Bumper";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nbool bump\r\nfloat32 phi";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getBump();
  void setBump(boolean value);
  float getPhi();
  void setPhi(float value);
}
