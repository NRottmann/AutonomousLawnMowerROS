package interfaces;

public interface Sensor extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Sensor";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nuint8 l1\r\nuint8 l2\r\nuint8 r1\r\nuint8 r2";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getL1();
  void setL1(byte value);
  byte getL2();
  void setL2(byte value);
  byte getR1();
  void setR1(byte value);
  byte getR2();
  void setR2(byte value);
}
