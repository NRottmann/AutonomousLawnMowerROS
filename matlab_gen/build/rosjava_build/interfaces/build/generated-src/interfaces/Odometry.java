package interfaces;

public interface Odometry extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "interfaces/Odometry";
  static final java.lang.String _DEFINITION = "Header header\r\n\r\nfloat32 l_R\t\t# distance wheel right, in m\r\nfloat32 l_L\t\t# distance wheel left, in m\r\nuint16 i_R\t\t# motor currents\t\r\nuint16 i_L\r\nint16 v_R\t\t# wheel velocities caluclated by the motor driver\r\nint16 v_L\r\nuint16 rawR\t\t# raw position of the wheels in ticks of the Hall Sensor A\r\nuint16 rawL";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getLR();
  void setLR(float value);
  float getLL();
  void setLL(float value);
  short getIR();
  void setIR(short value);
  short getIL();
  void setIL(short value);
  short getVR();
  void setVR(short value);
  short getVL();
  void setVL(short value);
  short getRawR();
  void setRawR(short value);
  short getRawL();
  void setRawL(short value);
}
