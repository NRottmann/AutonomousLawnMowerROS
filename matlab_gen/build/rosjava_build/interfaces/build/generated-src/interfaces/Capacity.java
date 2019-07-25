package interfaces;

public interface Capacity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Capacity";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nuint16[5] data";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short[] getData();
  void setData(short[] value);
}
