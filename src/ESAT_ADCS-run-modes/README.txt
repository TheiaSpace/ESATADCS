ADCS run modes abstract the modes of operation of the ADCS.


# ESAT_ADCSRunMode

Run modes must implement this interface, which exposes two methods:

  byte identifier();

  void run();

The identifier must be unique for each run mude.

The run() method performs whatever actions are relevant to the run
mode, like running the attitude control loop or simply switching on
and off actuators.

To change the run mode, use ESAT_ADCS.setRunMode()


# Run modes

  FOLLOW_MAGNETIC_TARGET: 0x00
  FOLLOW_SOLAR_TARGET: 0x01
  DETUMBLE: 0x02
  WHEEL_SET_DUTY_CYCLE: 0x20
  WHEEL_SET_SPEED: 0x21
  MAGNETORQUER_ENABLE: 0x30
  MAGNETORQUER_SET_XPOLARITY: 0x31
  MAGNETORQUER_SET_YPOLARITY: 0x32
  MAGNETORQUER_APPLY_MAXIMUM_TORQUE: 0x33
  MAGNETORQUER_DEMAGNETIZE: 0x34
  STOP_ACTUATORS: 0xFF
