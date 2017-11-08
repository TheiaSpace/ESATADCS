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

  FOLLOW_MAGNETIC_TARGET: 0X00
  FOLLOW_SOLAR_TARGET: 0X01
  WHEEL_SET_DUTY_CYCLE: 0X20
  WHEEL_SET_SPEED: 0X21
  MAGNETORQUER_ENABLE: 0X30
  MAGNETORQUER_SET_XPOLARITY: 0X31
  MAGNETORQUER_SET_YPOLARITY: 0X32
  MAGNETORQUER_APPLY_MAXIMUM_TORQUE: 0X33
  MAGNETORQUER_DEMAGNETIZE: 0X34
  STOP_ACTUATORS: 0XFF
