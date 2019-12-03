package interfaces;

public interface Chlorophyll extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Chlorophyll";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nuint8 data\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getData();
  void setData(byte value);
}
