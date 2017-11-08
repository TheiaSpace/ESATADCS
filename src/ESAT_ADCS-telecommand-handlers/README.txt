ADCS telecommand handlers provide telecommand responses.  Each
telecommand handler is intended to manage a group of related
telecommands, identified by the packet identifier field of the
secondary header of the telecommand packet.


# ESAT_ADCSTelecommandHandler

Telecommand handlers must implement this interface, which exposes one
method:

  boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

The telecommand handler must identify the telecommand with the
packetIdentifier field of the secondary header (obtained with
telecommand.readSecondaryHeader()) and:

  * if the telecommand handler owns this telecommand, handle it
    and return true;

  * if the telecommand handler doesn't own this telecommand,
    return false.

Telecommand arguments are part of the user data field (the part of the
packet data that comes after the secondary header).

To register a new telecommand handler, use
ESAT_ADCS.registerTelecommandHandler()


# ESAT_AttitudeTelecommandHandler

Telecommands related to attitude control:

  FOLLOW_MAGNETIC_TARGET = 0x00
  FOLLOW_SOLAR_TARGET = 0x01
  ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN = 0x10
  ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN = 0x11
  ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN = 0x12
  ATTITUDE_CONTROLLER_RESET_ERROR_INTEGRAL = 0x13
  ATTITUDE_CONTROLLER_USE_GYROSCOPE = 0x14
  ATTITUDE_CONTROLLER_SET_ACTUATOR = 0x15
  ATTITUDE_CONTROLLER_SET_DEADBAND = 0x16
  ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD = 0x17


# ESAT_WheelTelecommandHandler

Telecommands related to the wheel:

  WHEEL_SET_DUTY_CYCLE = 0x20
  WHEEL_SET_SPEED = 0x21
  WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN = 0x22
  WHEEL_CONTROLLER_SET_INTEGRAL_GAIN = 0x23
  WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN = 0x24
  WHEEL_CONTROLLER_RESET_ERROR_INTEGRAL = 0x25


# ESAT_MagnetorquerTelecommandHandler

Telecommands related to the magnetorquer:

  MAGNETORQUER_ENABLE = 0x30
  MAGNETORQUER_SET_X_POLARITY = 0x31
  MAGNETORQUER_SET_Y_POLARITY = 0x32
  MAGNETORQUER_APPLY_MAXIMUM_TORQUE = 0x33
  MAGNETORQUER_DEMAGNETIZE = 0x34


# ESAT_StopActuatorsTelecommandHandler

Just one telecommand for stopping the actuators (mangetorquer and
wheel):

  STOP_ACTUATORS = 0xFF
