package interfaces;

public interface Radar extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Radar";
  static final java.lang.String _DEFINITION = "Header header\n\nuint16 data";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short getData();
  void setData(short value);
}
